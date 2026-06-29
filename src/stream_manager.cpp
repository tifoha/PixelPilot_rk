#include "stream_manager.h"

#include <cstring>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "spdlog/spdlog.h"

// Owned by main.cpp; __DISPLAY_THREAD__ waits on these exactly as it does
// today for the single-stream case -- StreamManager only decides which
// stream's decode thread is allowed to signal them.
extern pthread_mutex_t video_mutex;
extern pthread_cond_t video_cond;

StreamManager::StreamManager(int drm_fd, struct modeset_output *output_list, int video_zpos)
    : drm_fd_(drm_fd), output_list_(output_list), video_zpos_(video_zpos) {}

void StreamManager::add_stream(int udp_port, VideoCodec codec) {
    int idx = (int)streams_.size();
    auto sp = std::make_unique<StreamPipeline>(idx, udp_port, codec, drm_fd_, output_list_);

    sp->set_active_frame_cb([this, idx](uint32_t fb_id, uint64_t pts) {
        // Only ever called while this stream is the active one (see
        // StreamPipeline::decode_loop's active_.load() guard) -- but by
        // the time this runs, a switch_to() on another thread could have
        // just flipped active_index_ already. Re-check here so a frame
        // that was "in flight" from the stream we're switching AWAY FROM
        // can't sneak through and overwrite the new stream's geometry/FB.
        if (idx != active_index_) return;

        StreamPipeline *sp_ = streams_[idx].get();
        uint32_t fw = sp_->frame_width(), fh = sp_->frame_height();
        if (fw != 0 && fh != 0 &&
            (fw != (uint32_t)output_list_->video_frm_width || fh != (uint32_t)output_list_->video_frm_height)) {
            // Same call, same thread-ownership pattern as today's single-
            // stream init_buffer(): a real, synchronous DRM modeset commit
            // from the decode thread, strictly before the video_cond
            // signal below -- so __DISPLAY_THREAD__ never touches
            // output_list->video_request concurrently with this.
            output_list_->video_frm_width = fw;
            output_list_->video_frm_height = fh;
            output_list_->video_fb_x = 0;
            output_list_->video_fb_y = 0;
            output_list_->video_fb_width = output_list_->mode.hdisplay;
            output_list_->video_fb_height = output_list_->mode.vdisplay;
            int ret = modeset_perform_modeset(drm_fd_, output_list_, output_list_->video_request,
                                               &output_list_->video_plane, fb_id, fw, fh, video_zpos_);
            if (ret < 0) {
                spdlog::error("[switcher] modeset_perform_modeset failed switching to stream {} ({}x{})", idx, fw, fh);
            }
            any_modeset_done_ = true;
            spdlog::info("[switcher] modeset for stream {}: {}x{}", idx, fw, fh);
        }

        pthread_mutex_lock(&video_mutex);
        output_list_->video_fb_id = (int)fb_id;
        output_list_->decoding_pts = pts;
        pthread_cond_signal(&video_cond);
        pthread_mutex_unlock(&video_mutex);
    });

    streams_.push_back(std::move(sp));
}

void StreamManager::start_all() {
    if (streams_.empty()) return;
    streams_[0]->set_active(true);
    active_index_ = 0;
    for (auto &s : streams_) s->start();
}

void StreamManager::stop_all() {
    for (auto &s : streams_) s->stop();
}

void StreamManager::switch_to(int idx) {
    if (idx < 0 || idx >= (int)streams_.size() || idx == active_index_) return;
    int old_idx = active_index_;
    streams_[idx]->set_active(true);
    active_index_ = idx;
    streams_[old_idx]->set_active(false);
    spdlog::info("[switcher] switching stream {} -> {}", old_idx, idx);
}

void StreamManager::switch_to_next() {
    int n = (int)streams_.size();
    if (n < 2) return;
    switch_to((active_index_ + 1) % n);
}

void StreamManager::switch_to_prev() {
    int n = (int)streams_.size();
    if (n < 2) return;
    switch_to((active_index_ + n - 1) % n);
}

bool StreamManager::is_active_stream_stale() const {
    if (streams_.empty()) return false;
    return streams_[active_index_]->latest_fb_id() == 0;
}

uint32_t StreamManager::ensure_no_signal_fb(uint32_t w, uint32_t h) {
    if (w == 0 || h == 0) return 0;
    if (no_signal_fb_id_ != 0 && w == no_signal_w_ && h == no_signal_h_) return no_signal_fb_id_;

    if (no_signal_fb_id_ != 0) {
        drmModeRmFB(drm_fd_, no_signal_fb_id_);
        no_signal_fb_id_ = 0;
    }
    if (no_signal_handle_ != 0) {
        struct drm_mode_destroy_dumb dmd = { .handle = no_signal_handle_ };
        ioctl(drm_fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &dmd);
        no_signal_handle_ = 0;
    }

    // NV12: Y plane (w x h, one byte/pixel) followed by interleaved UV
    // plane (w x h/2, half the rows since chroma is subsampled 2x2).
    // Y=16/UV=128 is "black" in limited-range YUV (the convention MPP's
    // decoded frames already use), not Y=0 -- Y=0 reads as crushed/wrong
    // black on most displays for limited-range content.
    struct drm_mode_create_dumb dmcd;
    memset(&dmcd, 0, sizeof(dmcd));
    dmcd.bpp = 8;
    dmcd.width = w;
    dmcd.height = h + h / 2;
    int ret;
    do { ret = ioctl(drm_fd_, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd); }
    while (ret == -1 && (errno == EINTR || errno == EAGAIN));
    if (ret) { spdlog::error("[switcher] no-signal buffer: create_dumb failed"); return 0; }
    no_signal_handle_ = dmcd.handle;

    struct drm_mode_map_dumb dmmd;
    memset(&dmmd, 0, sizeof(dmmd));
    dmmd.handle = dmcd.handle;
    ret = ioctl(drm_fd_, DRM_IOCTL_MODE_MAP_DUMB, &dmmd);
    if (ret) { spdlog::error("[switcher] no-signal buffer: map_dumb failed"); return 0; }
    void *map = mmap(0, dmcd.size, PROT_READ | PROT_WRITE, MAP_SHARED, drm_fd_, dmmd.offset);
    if (map == MAP_FAILED) { spdlog::error("[switcher] no-signal buffer: mmap failed"); return 0; }
    memset(map, 16, (size_t)dmcd.pitch * h);            // Y plane
    memset((uint8_t*)map + dmcd.pitch * h, 128, (size_t)dmcd.pitch * (h / 2)); // UV plane
    munmap(map, dmcd.size);

    uint32_t handles[4] = {0}, pitches[4] = {0}, offsets[4] = {0};
    handles[0] = dmcd.handle;
    pitches[0] = dmcd.pitch;
    handles[1] = dmcd.handle;
    offsets[1] = dmcd.pitch * h;
    pitches[1] = dmcd.pitch;
    ret = drmModeAddFB2(drm_fd_, w, h, DRM_FORMAT_NV12, handles, pitches, offsets, &no_signal_fb_id_, 0);
    if (ret) { spdlog::error("[switcher] no-signal buffer: addfb2 failed"); no_signal_fb_id_ = 0; return 0; }

    no_signal_w_ = w;
    no_signal_h_ = h;
    spdlog::info("[switcher] no-signal buffer ready: {}x{} fb_id={}", w, h, no_signal_fb_id_);
    return no_signal_fb_id_;
}
