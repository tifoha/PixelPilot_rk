#include "udp_restream.h"

#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "spdlog/spdlog.h"

bool UdpRestream::init(const std::string& ip, int port, VideoCodec codec) {
    codec_ = codec;
    const char *fmt = (codec == VideoCodec::H264) ? "h264" : "h265";

    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) {
        spdlog::error("[display-restream] socket() failed: {}", strerror(errno));
        return false;
    }

    struct sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(port);
    if (inet_pton(AF_INET, ip.c_str(), &dst.sin_addr) != 1) {
        spdlog::error("[display-restream] invalid IP: {}", ip);
        close(sock_); sock_ = -1;
        return false;
    }
    if (connect(sock_, (struct sockaddr *)&dst, sizeof(dst)) < 0) {
        spdlog::error("[display-restream] connect() failed: {}", strerror(errno));
        close(sock_); sock_ = -1;
        return false;
    }

    // Allow IP fragmentation for large keyframes (PMTU discovery off)
    int pmtu = IP_PMTUDISC_DONT;
    setsockopt(sock_, IPPROTO_IP, IP_MTU_DISCOVER, &pmtu, sizeof(pmtu));

    // Increase send buffer to avoid drops on burst keyframes
    int sndbuf = 2 * 1024 * 1024;
    setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    running_ = true;
    spdlog::info("[display-restream] → {}:{} codec={} (raw byte-stream)", ip, port, fmt);
    return true;
}

static void udp_send_chunked(int sock, const uint8_t *data, size_t len) {
    // UDP hard limit is 65507 bytes. Split large NALs into chunks.
    static constexpr size_t kMaxUdp = 60000;
    size_t offset = 0;
    while (offset < len) {
        size_t chunk = std::min(kMaxUdp, len - offset);
        ssize_t s = send(sock, data + offset, chunk, MSG_DONTWAIT);
        if (s < 0) {
            if (errno == ECONNREFUSED) {
                // A previous datagram hit a closed port (ICMP); the error is now
                // cleared — retry immediately so this datagram is not lost.
                s = send(sock, data + offset, chunk, MSG_DONTWAIT);
            }
            if (s < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                spdlog::warn("[display-restream] send() failed: {}", strerror(errno));
                return;
            }
        }
        offset += chunk;
    }
}

void UdpRestream::send_nal(std::shared_ptr<std::vector<uint8_t>> nal) {
    if (!running_ || sock_ < 0 || !nal || nal->size() < 5) return;

    const uint8_t *data = nal->data();
    size_t len = nal->size();

    // Classify the buffer by scanning all NAL units.
    // Rockchip MPP prepends SPS+PPS to every IDR frame, so the IDR packet starts
    // with the same bytes as the standalone extra_data header buffer.  We must
    // distinguish "pure parameter sets" (cache-worthy) from "frame with embedded
    // headers" (send normally with cached headers prepended).
    // H.264: SPS=7, PPS=8, coded-slices=1-5.
    // H.265: VPS=32, SPS=33, PPS=34, coded-slices=0-21.
    bool has_param = false;
    bool has_sps_or_vps = false;
    bool has_coded_slice = false;
    bool is_h265 = (codec_ == VideoCodec::H265);
    for (size_t i = 0; i + 4 < len; i++) {
        if (data[i]==0 && data[i+1]==0 && data[i+2]==0 && data[i+3]==1) {
            if (is_h265) {
                int nt = (data[i+4] >> 1) & 0x3F;
                if (nt == 32)               { has_param = true; has_sps_or_vps = true; }
                else if (nt==33 || nt==34)  { has_param = true; }
                else if (nt <= 21)          { has_coded_slice = true; }
            } else {
                int nt = data[i+4] & 0x1F;
                if (nt == 7)                { has_param = true; has_sps_or_vps = true; }
                else if (nt == 8)           { has_param = true; }
                else if (nt >= 1 && nt <= 5){ has_coded_slice = true; }
            }
        }
    }

    // A buffer is "pure parameters" only when it contains SPS/PPS but NO coded
    // slices.  Cache it so it can be prepended before every subsequent frame.
    if (has_param && !has_coded_slice) {
        if (has_sps_or_vps) {
            cached_headers_.assign(data, data + len);
        } else {
            cached_headers_.insert(cached_headers_.end(), data, data + len);
        }
        udp_send_chunked(sock_, data, len);
        return;
    }

    // All other buffers (frames, or frames with embedded SPS+PPS like IDR):
    // prepend the cached parameter sets so any late-joining receiver can sync
    // immediately — one small UDP datagram before each encoded frame.
    if (!cached_headers_.empty()) {
        udp_send_chunked(sock_, cached_headers_.data(), cached_headers_.size());
    }
    udp_send_chunked(sock_, data, len);
}

void UdpRestream::stop() {
    if (!running_) return;
    running_ = false;
    if (sock_ >= 0) { close(sock_); sock_ = -1; }
}
