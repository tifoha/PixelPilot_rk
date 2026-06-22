#pragma once

// StreamManager -- owns all StreamPipelines and the switching logic.
//
// This is the "DirectSink" half of the VideoSink split described in
// docs/video-output-architecture-proposed.md: no RGA, no compositor.
// Switching is a bare FB_ID swap on the existing single shared DRM plane
// when the target stream's resolution matches what's currently committed
// (the common case -- same/similar cameras), and a real
// modeset_perform_modeset() only when it doesn't (a real geometry change,
// inherently heavier, but rare). __DISPLAY_THREAD__ in main.cpp is
// completely unchanged -- it still just waits on video_cond and commits
// whatever FB_ID it's given; StreamManager's job is only to decide which
// stream's decode thread is allowed to drive that cond var (via each
// StreamPipeline's active-frame callback) and to handle the resolution
// check at the moment of switching.

#include <atomic>
#include <memory>
#include <vector>

#include "stream_pipeline.h"

extern "C" {
#include "drm.h"
}

class StreamManager {
public:
    StreamManager(int drm_fd, struct modeset_output *output_list, int video_zpos);

    // port[:codec] already parsed by the caller (see main.cpp's CLI parsing).
    void add_stream(int udp_port, VideoCodec codec);

    // Starts every configured stream's decode pipeline and activates the
    // first one. Call after all add_stream() calls.
    void start_all();
    void stop_all();

    void switch_to(int idx);
    void switch_to_next();
    void switch_to_prev();

    int active_index() const { return active_index_.load(); }
    int stream_count() const { return (int)streams_.size(); }
    StreamPipeline *stream(int idx) { return streams_.at(idx).get(); }

private:
    int drm_fd_;
    struct modeset_output *output_list_;
    int video_zpos_;
    std::vector<std::unique_ptr<StreamPipeline>> streams_;
    std::atomic<int> active_index_{0};
    bool any_modeset_done_ = false;
};
