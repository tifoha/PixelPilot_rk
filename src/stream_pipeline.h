#pragma once

// StreamPipeline -- one independent decode pipeline: its own minimal
// GStreamer RTP receive + its own MPP decode context + its own DRM
// dumb-buffer pool. Multiple instances run concurrently; only the
// switcher's currently-active one ever has its FB_ID committed to the
// single shared DRM video plane (see stream_manager.h).
//
// Deliberately NOT reusing GstRtpReceiver here: that class's IDR-burst /
// restream-to-phone / RTP-gap-detection sophistication is real, valuable
// behavior on the existing single-stream path, but threading all of that
// per-instance is a separate, larger task. For the multistream MVP, IDR
// handling is an explicit mock (see request_idr() below) -- this mirrors
// the already-proven multistream-poc design (GStreamer pull-loop + MPP
// decode loop), just adapted to PixelPilot_rk's own DRM buffer-pool
// conventions (MAX_FRAMES dumb buffers, NV12 FB per frame) instead of the
// POC's simplified versions of the same pattern.

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <rockchip/rk_mpi.h>

#include "gstrtpreceiver.h"  // VideoCodec (C++ enum class, not extern "C")

extern "C" {
#include "drm.h"   // struct modeset_output
}

#define STREAM_PIPELINE_MAX_FRAMES 24

class StreamPipeline {
public:
    // unix_socket: if non-empty, receive RTP from abstract AF_UNIX socket @name
    //              instead of UDP port. udp_port is ignored in that case.
    StreamPipeline(int index, int udp_port, const std::string& unix_socket,
                   VideoCodec codec, int drm_fd, struct modeset_output *output_list);
    ~StreamPipeline();

    void start();
    void stop();

    void set_active(bool active) { active_.store(active); }
    bool is_active() const { return active_.load(); }
    int index() const { return index_; }
    int udp_port() const { return udp_port_; }
    const std::string& unix_socket() const { return unix_socket_; }
    VideoCodec codec() const { return codec_; }

    // Invoked from decode_loop() only when is_active() is true at the
    // moment a new frame lands -- this is how the existing single-shared-
    // plane __DISPLAY_THREAD__/video_cond mechanism in main.cpp stays
    // completely unchanged: only the active stream ever wakes it, exactly
    // like today's single-stream code, just routed through whichever
    // StreamPipeline is currently active instead of one fixed global.
    using ActiveFrameCb = std::function<void(uint32_t fb_id, uint64_t pts)>;
    void set_active_frame_cb(ActiveFrameCb cb) { on_active_frame_ = std::move(cb); }

    // Delivers raw decoded frame data from the active stream to whoever needs
    // it (e.g. FrameProcessor for display restream / DVR re-encode). Called
    // from decode_loop() only when is_active() is true.
    using RawFrameCb = std::function<void(MppBuffer, uint32_t w, uint32_t h,
                                          uint32_t hs, uint32_t vs, MppFrameFormat)>;
    void set_raw_frame_cb(RawFrameCb cb) { on_raw_frame_ = std::move(cb); }

    // Delivers raw compressed H.264/H.265 byte-stream NAL units from the
    // GStreamer appsink, called for EVERY frame regardless of active_ state.
    // Used for per-stream raw DVR: all configured streams record simultaneously.
    using ByteStreamCb = std::function<void(std::shared_ptr<std::vector<uint8_t>>)>;
    void set_bs_frame_cb(ByteStreamCb cb) { on_bs_frame_ = std::move(cb); }

    // Fired once when the stream's decoded frame dimensions first become known
    // (or change). Used by raw DVR to call set_video_params() with real dims.
    using DimChangeCb = std::function<void(uint32_t w, uint32_t h, VideoCodec codec)>;
    void set_dim_change_cb(DimChangeCb cb) { on_dim_change_ = std::move(cb); }

    // Latest decoded frame, read by the switcher/display path. fb_id==0
    // means nothing decoded yet (or stream gone stale -- see kStaleMs).
    uint32_t latest_fb_id() const;
    uint32_t frame_width() const { return frm_width_; }
    uint32_t frame_height() const { return frm_height_; }
    uint64_t latest_decoding_pts() const { return latest_pts_.load(); }

    // Mock for now -- logs intent only. Real per-stream IDR requests need
    // either GstRtpReceiver's existing UDP-burst mechanism multiplied per
    // stream, or (future work, see docs/video-output-architecture-proposed.md)
    // an alink-based request to the air unit. Not wired to anything live.
    void request_idr(const char *reason);

    // Configure RTP restream target. Call before start(). ip="" disables.
    // Default port for the pipeline is 5600+index if not configured.
    void restream_configure(const std::string& ip, int port);
    void restream_close_valve();

private:
    void start_mpp();
    void start_gst();
    void check_bus_errors();
    void gst_pull_loop();
    void feed_packet(const uint8_t *data, int len);
    void decode_loop();
    void init_buffers(MppFrame frame);
    void free_drm_buffers();
    void set_mpp_decoding_parameters();

    struct RestreamState {
        GstElement *valve = nullptr;
        GstElement *sink  = nullptr;
        std::string ip;   // empty = disabled
        int port = 0;     // 0 = use default (5600 + index_)
    };

    int index_;
    int udp_port_;
    std::string unix_socket_;
    VideoCodec codec_;
    int drm_fd_;
    struct modeset_output *output_list_;
    RestreamState restream_;

    GstElement *pipeline_ = nullptr;
    GstElement *appsink_ = nullptr;
    GstElement *appsrc_ = nullptr;
    int sock_fd_ = -1;
    std::atomic<bool> sock_running_{false};
    std::thread sock_thread_;

    MppCtx ctx_ = nullptr;
    MppApi *mpi_ = nullptr;
    MppPacket packet_ = nullptr;
    uint8_t *nal_buffer_ = nullptr;
    MppBufferGroup frm_grp_ = nullptr;
    struct { int prime_fd = -1; uint32_t fb_id = 0; uint32_t handle = 0; } frame_to_drm_[STREAM_PIPELINE_MAX_FRAMES];

    uint32_t frm_width_ = 0, frm_height_ = 0;
    uint32_t hor_stride_ = 0, ver_stride_ = 0;

    std::atomic<bool> active_{false};
    std::atomic<uint32_t> latest_fb_id_{0};
    std::atomic<uint64_t> latest_pts_{0};
    std::atomic<uint64_t> latest_update_ms_{0};
    static constexpr uint64_t kStaleMs = 1500;

    std::atomic<bool> running_{false};
    std::thread decode_thread_;
    std::thread gst_thread_;
    ActiveFrameCb on_active_frame_;
    RawFrameCb on_raw_frame_;
    ByteStreamCb on_bs_frame_;
    DimChangeCb on_dim_change_;

    bool first_sample_seen_ = false;
    bool first_feed_ok_logged_ = false;
    bool first_feed_fail_logged_ = false;
    uint64_t fed_count_ = 0, feed_fail_count_ = 0, decoded_count_ = 0;

    void loop_unix_socket();
};
