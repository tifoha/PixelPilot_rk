#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gstrtpreceiver.h"  // VideoCodec

// Sends re-encoded (OSD-blended) Annex-B NAL units as raw H.264/H.265
// byte-stream over UDP.  Compatible with QGC "UDP H.264/H.265 Video Stream"
// and ffplay -f h264/hevc udp://...
// Used for display restream (--restream display:<ip>[:<port>]).
class UdpRestream {
public:
    ~UdpRestream() { stop(); }

    bool init(const std::string& ip, int port, VideoCodec codec);
    void send_nal(std::shared_ptr<std::vector<uint8_t>> nal);
    void stop();

private:
    int sock_ = -1;
    bool running_ = false;
    VideoCodec codec_ = VideoCodec::H264;
    std::vector<uint8_t> cached_headers_;  // SPS+PPS (H.264) or VPS+SPS+PPS (H.265)
    int nal_log_count_ = 0;
};
