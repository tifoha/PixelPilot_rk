#include "stream_pipeline.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "spdlog/spdlog.h"

// time_util.h's get_time_ms() is defined directly in the header (not
// inline), so it can only be included from one translation unit -- main.cpp
// already does. A local equivalent here avoids a multiple-definition link
// error (same approach the multistream-poc used for the same reason).
static uint64_t get_time_ms() {
    struct timespec spec;
    clock_gettime(CLOCK_MONOTONIC, &spec);
    return (uint64_t)spec.tv_sec * 1000 + spec.tv_nsec / 1000000;
}

StreamPipeline::StreamPipeline(int index, int udp_port, VideoCodec codec, int drm_fd, struct modeset_output *output_list)
    : index_(index), udp_port_(udp_port), codec_(codec), drm_fd_(drm_fd), output_list_(output_list) {}

StreamPipeline::~StreamPipeline() {
    stop();
}

void StreamPipeline::start() {
    start_mpp();
    start_gst();
    running_ = true;
    decode_thread_ = std::thread(&StreamPipeline::decode_loop, this);
    gst_thread_ = std::thread(&StreamPipeline::gst_pull_loop, this);
}

void StreamPipeline::stop() {
    if (!running_.load()) return;
    running_ = false;
    if (pipeline_) gst_element_set_state(pipeline_, GST_STATE_NULL);
    if (gst_thread_.joinable()) gst_thread_.join();
    if (decode_thread_.joinable()) decode_thread_.join();
    free_drm_buffers();
    if (frm_grp_) { mpp_buffer_group_put(frm_grp_); frm_grp_ = nullptr; }
    if (packet_) { mpp_packet_deinit(&packet_); packet_ = nullptr; }
    if (nal_buffer_) { free(nal_buffer_); nal_buffer_ = nullptr; }
    if (ctx_) { mpp_destroy(ctx_); ctx_ = nullptr; }
    if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
}

uint32_t StreamPipeline::latest_fb_id() const {
    if (get_time_ms() - latest_update_ms_.load() > kStaleMs) return 0;
    return latest_fb_id_.load();
}

void StreamPipeline::request_idr(const char *reason) {
    spdlog::debug("[stream {}] request_idr({}) -- mock, not wired to anything yet", index_, reason);
}

// ── MPP setup -- mirrors today's single-stream set_mpp_decoding_parameters() ──
void StreamPipeline::start_mpp() {
    MppCodingType mpp_type = (codec_ == VideoCodec::H264) ? MPP_VIDEO_CodingAVC : MPP_VIDEO_CodingHEVC;
    int ret = mpp_create(&ctx_, &mpi_);
    assert(!ret);
    set_mpp_decoding_parameters();
    ret = mpp_init(ctx_, MPP_CTX_DEC, mpp_type);
    assert(!ret);
    set_mpp_decoding_parameters();

    // Non-blocking, unlike the single-stream path's MPP_POLL_BLOCK: with N
    // independent per-stream decode threads each independently start()able/
    // stop()able, a blocking decode_get_frame() on a stream that's stopped
    // receiving data would hang decode_thread_.join() in stop() forever.
    // decode_loop() polls running_ instead.
    int param = MPP_POLL_NON_BLOCK;
    ret = mpi_->control(ctx_, MPP_SET_OUTPUT_BLOCK, &param);
    assert(!ret);

    nal_buffer_ = (uint8_t *)malloc(1024 * 1024);
    ret = mpp_packet_init(&packet_, nal_buffer_, 1024 * 1024);
    assert(!ret);
}

void StreamPipeline::set_mpp_decoding_parameters() {
    MppDecCfg cfg = nullptr;
    mpp_dec_cfg_init(&cfg);
    if (mpi_->control(ctx_, MPP_DEC_GET_CFG, cfg) == 0) {
        mpp_dec_cfg_set_u32(cfg, "base:split_parse", 1);
        mpi_->control(ctx_, MPP_DEC_SET_CFG, cfg);
    }
    mpp_dec_cfg_deinit(cfg);
    int on = 0xffff, off_split = 0;
    mpi_->control(ctx_, MPP_DEC_SET_PARSER_SPLIT_MODE, &off_split);
    mpi_->control(ctx_, MPP_DEC_SET_DISABLE_ERROR, &on);
    mpi_->control(ctx_, MPP_DEC_SET_IMMEDIATE_OUT, &on);
    mpi_->control(ctx_, MPP_DEC_SET_ENABLE_FAST_PLAY, &on);
}

// ── GStreamer setup -- minimal RTP-in pipeline, no restream/IDR/liveness ────
void StreamPipeline::start_gst() {
    const char *depay = (codec_ == VideoCodec::H264) ? "rtph264depay" : "rtph265depay";
    const char *parse = (codec_ == VideoCodec::H264) ? "h264parse" : "h265parse";
    const char *enc_name = (codec_ == VideoCodec::H264) ? "H264" : "H265";
    const char *caps_fmt = (codec_ == VideoCodec::H264) ? "h264" : "h265";

    char pipeline_str[512];
    snprintf(pipeline_str, sizeof(pipeline_str),
        "udpsrc port=%d caps=\"application/x-rtp, media=(string)video, "
        "encoding-name=(string)%s, clock-rate=(int)90000\" ! "
        "%s ! %s config-interval=-1 ! "
        "video/x-%s,stream-format=byte-stream,alignment=au ! "
        "appsink drop=true sync=false name=out_appsink",
        udp_port_, enc_name, depay, parse, caps_fmt);

    spdlog::info("[stream {}] pipeline: {}", index_, pipeline_str);
    GError *error = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str, &error);
    if (error) {
        spdlog::error("[stream {}] gst_parse_launch error: {}", index_, error->message);
    }
    if (!pipeline_ || error) {
        spdlog::error("[stream {}] gst_parse_launch failed, aborting stream", index_);
        return;
    }
    appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "out_appsink");
    assert(appsink_);
    GstStateChangeReturn sret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (sret == GST_STATE_CHANGE_FAILURE) {
        spdlog::error("[stream {}] failed to set pipeline to PLAYING", index_);
    }
    spdlog::info("[stream {}] listening on udp:{} ({})", index_, udp_port_, enc_name);
}

void StreamPipeline::check_bus_errors() {
    GstBus *bus = gst_element_get_bus(pipeline_);
    GstMessage *msg;
    while ((msg = gst_bus_pop_filtered(bus, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING)))) {
        GError *err = nullptr;
        gchar *dbg = nullptr;
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
            gst_message_parse_error(msg, &err, &dbg);
            spdlog::error("[stream {}] GST ERROR: {} ({})", index_, err->message, dbg ? dbg : "");
        } else {
            gst_message_parse_warning(msg, &err, &dbg);
            spdlog::warn("[stream {}] GST WARNING: {} ({})", index_, err->message, dbg ? dbg : "");
        }
        g_error_free(err);
        g_free(dbg);
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
}

void StreamPipeline::gst_pull_loop() {
    const uint64_t timeout_ns = 100ULL * 1000 * 1000; // 100ms
    int poll_count = 0;
    while (running_.load()) {
        if (++poll_count % 20 == 0) check_bus_errors(); // ~every 2s
        GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), timeout_ns);
        if (!sample) continue;
        GstBuffer *buf = gst_sample_get_buffer(sample);
        if (buf) {
            GstMapInfo map;
            if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
                if (!first_sample_seen_) {
                    first_sample_seen_ = true;
                    spdlog::info("[stream {}] first appsink sample: {} bytes", index_, map.size);
                }
                feed_packet(map.data, (int)map.size);
                gst_buffer_unmap(buf, &map);
            }
        }
        gst_sample_unref(sample);
    }
}

void StreamPipeline::feed_packet(const uint8_t *data, int len) {
    mpp_packet_set_data(packet_, (void *)data);
    mpp_packet_set_size(packet_, len);
    mpp_packet_set_pos(packet_, (void *)data);
    mpp_packet_set_length(packet_, len);
    mpp_packet_set_pts(packet_, (RK_S64)get_time_ms());
    uint64_t start = get_time_ms();
    while (running_.load() && MPP_OK != mpi_->decode_put_packet(ctx_, packet_)) {
        if (get_time_ms() - start > 100) {
            feed_fail_count_++;
            if (!first_feed_fail_logged_) {
                first_feed_fail_logged_ = true;
                spdlog::warn("[stream {}] decode_put_packet stalled (len={})", index_, len);
            }
            return; // drop on stall, same as the existing single-stream path
        }
        usleep(2000);
    }
    if (!first_feed_ok_logged_) {
        first_feed_ok_logged_ = true;
        spdlog::info("[stream {}] first decode_put_packet OK (len={})", index_, len);
    }
    ++fed_count_;
}

// ── Decode loop -- mirrors today's single-stream __FRAME_THREAD__, per stream ──
void StreamPipeline::decode_loop() {
    while (running_.load()) {
        MppFrame frame = nullptr;
        int ret = mpi_->decode_get_frame(ctx_, &frame);
        if (ret != MPP_OK || !frame) { usleep(2000); continue; }

        if (mpp_frame_get_info_change(frame)) {
            init_buffers(frame);
        } else {
            const RK_U32 errinfo = mpp_frame_get_errinfo(frame);
            const RK_U32 discard = mpp_frame_get_discard(frame);
            if (errinfo || discard) {
                request_idr("decoder-issue");
            }
            MppBuffer buffer = mpp_frame_get_buffer(frame);
            if (buffer) {
                MppBufferInfo info;
                mpp_buffer_info_get(buffer, &info);
                int idx = -1;
                for (int i = 0; i < STREAM_PIPELINE_MAX_FRAMES; i++) {
                    if (frame_to_drm_[i].prime_fd == info.fd) { idx = i; break; }
                }
                if (idx >= 0) {
                    uint32_t fb_id = frame_to_drm_[idx].fb_id;
                    uint64_t pts = (uint64_t)mpp_frame_get_pts(frame);
                    latest_fb_id_.store(fb_id);
                    latest_pts_.store(pts);
                    latest_update_ms_.store(get_time_ms());
                    if (active_.load() && on_active_frame_) {
                        on_active_frame_(fb_id, pts);
                    }
                    if (++decoded_count_ % 150 == 0)
                        spdlog::debug("[stream {}] decoded {} frames so far (active={})",
                                      index_, decoded_count_, active_.load());
                }
            }
        }
        bool eos = mpp_frame_get_eos(frame);
        mpp_frame_deinit(&frame);
        if (eos) break;
    }
}

// ── Buffer pool (re)allocation -- runs for every stream regardless of active
//    state. Does NOT touch the shared DRM plane/modeset at all -- that's the
//    switcher/DirectSink's job (see stream_manager.cpp), driven by whichever
//    stream is active and only on an actual resolution change. ──
void StreamPipeline::init_buffers(MppFrame frame) {
    frm_width_  = mpp_frame_get_width(frame);
    frm_height_ = mpp_frame_get_height(frame);
    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
    MppFrameFormat fmt = mpp_frame_get_fmt(frame);
    assert(fmt == MPP_FMT_YUV420SP || fmt == MPP_FMT_YUV420SP_10BIT);
    hor_stride_ = hor_stride;
    ver_stride_ = ver_stride;

    spdlog::info("[stream {}] frame info changed {}x{} (stride {}x{})",
                 index_, frm_width_, frm_height_, hor_stride, ver_stride);

    free_drm_buffers();

    int ret = mpp_buffer_group_get_external(&frm_grp_, MPP_BUFFER_TYPE_DRM);
    assert(!ret);

    for (int i = 0; i < STREAM_PIPELINE_MAX_FRAMES; i++) {
        struct drm_mode_create_dumb dmcd;
        memset(&dmcd, 0, sizeof(dmcd));
        dmcd.bpp = (fmt == MPP_FMT_YUV420SP) ? 8 : 10;
        dmcd.width = hor_stride;
        dmcd.height = ver_stride * 2;
        do { ret = ioctl(drm_fd_, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd); }
        while (ret == -1 && (errno == EINTR || errno == EAGAIN));
        assert(!ret);
        frame_to_drm_[i].handle = dmcd.handle;

        struct drm_prime_handle dph;
        memset(&dph, 0, sizeof(dph));
        dph.handle = dmcd.handle;
        dph.fd = -1;
        do { ret = ioctl(drm_fd_, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph); }
        while (ret == -1 && (errno == EINTR || errno == EAGAIN));
        assert(!ret);

        MppBufferInfo info;
        memset(&info, 0, sizeof(info));
        info.type = MPP_BUFFER_TYPE_DRM;
        info.size = dmcd.width * dmcd.height;
        info.fd = dph.fd;
        ret = mpp_buffer_commit(frm_grp_, &info);
        assert(!ret);
        frame_to_drm_[i].prime_fd = info.fd;
        if (dph.fd != info.fd) close(dph.fd);

        uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
        handles[0] = frame_to_drm_[i].handle;
        pitches[0] = hor_stride;
        handles[1] = frame_to_drm_[i].handle;
        offsets[1] = pitches[0] * ver_stride;
        pitches[1] = pitches[0];
        ret = drmModeAddFB2(drm_fd_, frm_width_, frm_height_, DRM_FORMAT_NV12,
                             handles, pitches, offsets, &frame_to_drm_[i].fb_id, 0);
        assert(!ret);
    }

    ret = mpi_->control(ctx_, MPP_DEC_SET_EXT_BUF_GROUP, frm_grp_);
    ret = mpi_->control(ctx_, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
}

void StreamPipeline::free_drm_buffers() {
    for (int i = 0; i < STREAM_PIPELINE_MAX_FRAMES; i++) {
        if (frame_to_drm_[i].fb_id) {
            drmModeRmFB(drm_fd_, frame_to_drm_[i].fb_id);
            frame_to_drm_[i].fb_id = 0;
        }
        if (frame_to_drm_[i].prime_fd >= 0) {
            close(frame_to_drm_[i].prime_fd);
            frame_to_drm_[i].prime_fd = -1;
        }
        if (frame_to_drm_[i].handle) {
            struct drm_mode_destroy_dumb dmd = { .handle = frame_to_drm_[i].handle };
            ioctl(drm_fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &dmd);
            frame_to_drm_[i].handle = 0;
        }
    }
    if (frm_grp_) {
        mpp_buffer_group_clear(frm_grp_);
        mpp_buffer_group_put(frm_grp_);
        frm_grp_ = nullptr;
    }
}
