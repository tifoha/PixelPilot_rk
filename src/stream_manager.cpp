#include "stream_manager.h"

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
