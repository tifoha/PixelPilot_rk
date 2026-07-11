#define MODULE_TAG "pixelpilot"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <inttypes.h>
#include <signal.h>
#include <fstream>
#include <filesystem>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>
#include <linux/videodev2.h>
#include <rockchip/rk_mpi.h>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

extern "C" {
#include "main.h"
#include "drm.h"

#include "mavlink/common/mavlink.h"
#include "mavlink.h"
#include "input.h"
}

#include "osd.h"
#include "osd.hpp"
#include "wfbcli.hpp"
#include "dvr.h"
#include "mpp_encoder.h"
#include "frame_processor.h"
#include "gstrtpreceiver.h"
#include "stream_manager.h"
#include "udp_restream.h"
#include "scheduling_helper.hpp"
#include "time_util.h"
#include "os_mon.hpp"
#include "pixelpilot_config.h"
#include <iostream>
#include "WiFiRSSIMonitor.hpp"
#include "gsmenu/gs_system.h"
#include "gsmenu/air_actions.h"
#include "gsmenu/gs_actions.h"
#include "menu.h"


#define READ_BUF_SIZE (1024*1024) // SZ_1M https://github.com/rockchip-linux/mpp/blob/ed377c99a733e2cdbcc457a6aa3f0fcd438a9dff/osal/inc/mpp_common.h#L179
#define MAX_FRAMES 24		// min 16 and 20+ recommended (mpp/readme.txt)

#define CODEC_ALIGN(x, a)   (((x)+(a)-1)&~((a)-1))

#define DEFAULT_CONFIG_PATH "/etc/pixelpilot.yaml"
YAML::Node config;

#define MSG_FIFO_NAME "/run/pixelpilot.msg"

struct {
	MppCtx		  ctx;
	MppApi		  *mpi;
	
	struct timespec first_frame_ts;

	MppBufferGroup	frm_grp;
	struct {
		int prime_fd;
		uint32_t fb_id;
		uint32_t handle;
	} frame_to_drm[MAX_FRAMES];
} mpi;

struct timespec frame_stats[1000];

struct modeset_output *output_list;
int frm_eos = 0;
int drm_fd = 0;
static StreamManager *g_stream_manager = nullptr; // set once, in main(), only in multistream mode
pthread_mutex_t video_mutex;
pthread_cond_t video_cond;
extern bool osd_update_ready;
extern bool gsmenu_enabled;
extern int gsmenu_transparency;
extern int gsmenu_error_timeout_ms;
void set_no_signal_indicator(bool show); // defined in osd.cpp
int video_zpos = 1;

void set_mpp_decoding_parameters(MppApi * mpi, MppCtx ctx);
static pthread_mutex_t mpp_reinit_mutex = PTHREAD_MUTEX_INITIALIZER;
static std::atomic<bool> mpp_reinit_pending{false};

bool mavlink_dvr_on_arm = false;
bool osd_custom_message = false;
bool disable_vsync = false;
bool disable_gregidr = false;
uint32_t refresh_frequency_ms = 1000;

VideoCodec codec = VideoCodec::H265;
uint16_t listen_port = 5600;
const char* unix_socket = NULL;
char* dvr_template = NULL;
Dvr *dvr_raw = NULL;
Dvr *dvr_reenc_inst = NULL;
MppEncoder *reencoder = NULL;
UdpRestream *display_restream = nullptr;
MppEncoderParams reenc_params;
DvrMode dvr_mode = DVR_MODE_RAW;
bool dvr_osd   = false;
bool display_reencode_osd = false;
static int video_framerate = -1;
static bool dvr_filenames_with_sequence = false;
static int mp4_fragmentation_mode = 0;
static int64_t dvr_max_file_size = 4000000000LL;  // 4 GB (decimal), safe margin for VFAT 4 GiB limit
FrameProcessor *frame_proc = nullptr;
// Thread handles for the encoder and pacer — file-scope so live mode toggle can join them.
static pthread_t g_tid_enc   = 0;
static pthread_t g_tid_fproc = 0;
static pthread_t g_tid_dvr_raw   = 0;
static pthread_t g_tid_dvr_reenc = 0;

// Decoded frame geometry – updated in init_buffer(), used in __FRAME_THREAD__
uint32_t decoded_hor_stride = 0;
uint32_t decoded_ver_stride = 0;
OsSensors os_sensors; // TODO: pass as argument to `main_loop`
MenuAction airactions[MAX_ACTIONS];
size_t airactions_count;
MenuAction gsactions[MAX_ACTIONS];
size_t gsactions_count;

// Add global variables for plane id overrides
uint32_t video_plane_id_override = 0;
uint32_t osd_plane_id_override = 0;

WiFiRSSIMonitor wifi_monitor;
extern enum RXMode RXMODE;

bool enable_live_colortrans = false;
float live_colortrans_offset = -0.15f;
float live_colortrans_gain = 2.5f;
gamma_lut_controller lut_ctrl;

// Helper: get target width/height for the current re-encode resolution setting.
static void reenc_target_dims(uint32_t &w, uint32_t &h) {
    if (reenc_params.resolution == EncResolution::Res720p) { w = 1280; h = 720; }
    else { w = 1920; h = 1080; }
}

void init_buffer(MppFrame frame) {
	output_list->video_frm_width = mpp_frame_get_width(frame);
	output_list->video_frm_height = mpp_frame_get_height(frame);
	RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
	RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
	MppFrameFormat fmt = mpp_frame_get_fmt(frame);
	assert((fmt == MPP_FMT_YUV420SP) || (fmt == MPP_FMT_YUV420SP_10BIT));

	decoded_hor_stride = hor_stride;
	decoded_ver_stride = ver_stride;

	spdlog::info("Frame info changed {}({})x{}({})",
				 output_list->video_frm_width, hor_stride, output_list->video_frm_height, ver_stride);

	output_list->video_fb_x = 0;
	output_list->video_fb_y = 0;
	output_list->video_fb_width = output_list->mode.hdisplay;
	output_list->video_fb_height =output_list->mode.vdisplay;	

	osd_publish_uint_fact("video.width", NULL, 0, output_list->video_frm_width);
	osd_publish_uint_fact("video.height", NULL, 0, output_list->video_frm_height);

	// Drain any decoder-buffer refs held by the encoder pacer before freeing
	// the group.  Without this the group teardown races with the pacer's copy
	// loop and the buffer fds become invalid while still in use.
	if (frame_proc) frame_proc->drain_decoder_refs();

	if (mpi.frm_grp) {
		spdlog::debug("Freeing current mpp_buffer_group");

		// First clean up all DRM resources for existing frames
		for (int i = 0; i < MAX_FRAMES; i++) {
			if (mpi.frame_to_drm[i].fb_id) {
				drmModeRmFB(drm_fd, mpi.frame_to_drm[i].fb_id);
				mpi.frame_to_drm[i].fb_id = 0;
			}
			if (mpi.frame_to_drm[i].prime_fd >= 0) {
				close(mpi.frame_to_drm[i].prime_fd);
				mpi.frame_to_drm[i].prime_fd = -1;
			}
			if (mpi.frame_to_drm[i].handle) {
				struct drm_mode_destroy_dumb dmd = {
					.handle = mpi.frame_to_drm[i].handle,
				};
				ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmd);
				mpi.frame_to_drm[i].handle = 0;
			}
		}
		
		mpp_buffer_group_clear(mpi.frm_grp);
		mpp_buffer_group_put(mpi.frm_grp);  // This is important to release the group
		mpi.frm_grp = NULL;
	}

	// create new external frame group and allocate (commit flow) new DRM buffers and DRM FB
	int ret = mpp_buffer_group_get_external(&mpi.frm_grp, MPP_BUFFER_TYPE_DRM);
	assert(!ret);			

	for (int i=0; i<MAX_FRAMES; i++) {
		
		// new DRM buffer
		struct drm_mode_create_dumb dmcd;
		memset(&dmcd, 0, sizeof(dmcd));
		dmcd.bpp = fmt==MPP_FMT_YUV420SP?8:10;
		dmcd.width = hor_stride;
		dmcd.height = ver_stride*2; // documentation say not v*2/3 but v*2 (additional info included)
		do {
			ret = ioctl(drm_fd, DRM_IOCTL_MODE_CREATE_DUMB, &dmcd);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		// assert(dmcd.pitch==(fmt==MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8));
		// assert(dmcd.size==(fmt == MPP_FMT_YUV420SP?hor_stride:hor_stride*10/8)*ver_stride*2);
		mpi.frame_to_drm[i].handle = dmcd.handle;
		
		// commit DRM buffer to frame group
		struct drm_prime_handle dph;
		memset(&dph, 0, sizeof(struct drm_prime_handle));
		dph.handle = dmcd.handle;
		dph.fd = -1;
		do {
			ret = ioctl(drm_fd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &dph);
		} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
		assert(!ret);
		MppBufferInfo info;
		memset(&info, 0, sizeof(info));
		info.type = MPP_BUFFER_TYPE_DRM;
		info.size = dmcd.width*dmcd.height;
		info.fd = dph.fd;
		ret = mpp_buffer_commit(mpi.frm_grp, &info);
		assert(!ret);
		mpi.frame_to_drm[i].prime_fd = info.fd; // dups fd						
		if (dph.fd != info.fd) {
			ret = close(dph.fd);
			assert(!ret);
		}

		// allocate DRM FB from DRM buffer
		uint32_t handles[4], pitches[4], offsets[4];
		memset(handles, 0, sizeof(handles));
		memset(pitches, 0, sizeof(pitches));
		memset(offsets, 0, sizeof(offsets));
		handles[0] = mpi.frame_to_drm[i].handle;
		offsets[0] = 0;
		pitches[0] = hor_stride;						
		handles[1] = mpi.frame_to_drm[i].handle;
		offsets[1] = pitches[0] * ver_stride;
		pitches[1] = pitches[0];
		ret = drmModeAddFB2(drm_fd, output_list->video_frm_width, output_list->video_frm_height, DRM_FORMAT_NV12, handles, pitches, offsets, &mpi.frame_to_drm[i].fb_id, 0);
		assert(!ret);
	}

	// register external frame group
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_EXT_BUF_GROUP, mpi.frm_grp);
	ret = mpi.mpi->control(mpi.ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);

	ret = modeset_perform_modeset(drm_fd, output_list, output_list->video_request, &output_list->video_plane, mpi.frame_to_drm[0].fb_id, output_list->video_frm_width, output_list->video_frm_height, video_zpos);
	assert(ret >= 0);

	// dvr setup
	if (dvr_raw != NULL) {
		dvr_raw->set_video_params(output_list->video_frm_width, output_list->video_frm_height, codec);
	}
	if (dvr_reenc_inst != NULL) {
		uint32_t rw, rh; reenc_target_dims(rw, rh);
		dvr_reenc_inst->set_video_params(rw, rh, reenc_params.codec);
	}
}

// __FRAME_THREAD__
//
// - allocate DRM buffers and DRM FB based on frame size
// - pick frame in blocking mode and output to screen overlay

void *__FRAME_THREAD__(void *param)
{
	SchedulingHelper::set_thread_params_max_realtime("FRAME_THREAD",SchedulingHelper::PRIORITY_REALTIME_MID);
	int i, ret;
	MppFrame  frame  = NULL;
	uint64_t last_frame_time;
	pthread_setname_np(pthread_self(), "__FRAME");

	while (!frm_eos) {
		struct timespec ts, ats;

		assert(!frame);
		pthread_mutex_lock(&mpp_reinit_mutex);
		if (mpp_reinit_pending.load(std::memory_order_acquire)) {
			// Decoder is being reinitialized — release lock and wait
			pthread_mutex_unlock(&mpp_reinit_mutex);
			usleep(5000);
			continue;
		}
		ret = mpi.mpi->decode_get_frame(mpi.ctx, &frame);
		pthread_mutex_unlock(&mpp_reinit_mutex);
		if (mpp_reinit_pending.load(std::memory_order_acquire)) {
			// Reinit started while we were blocked — discard result
			if (frame) {
				mpp_frame_deinit(&frame);
				frame = NULL;
			}
			continue;
		}
		assert(!ret);
		clock_gettime(CLOCK_MONOTONIC, &ats);
		if (frame) {
			if (mpp_frame_get_info_change(frame)) {
				// new resolution
				init_buffer(frame);
			} else {
				// regular frame received
				idr_notify_decoded_frame();
				const RK_U32 errinfo = mpp_frame_get_errinfo(frame);
				const RK_U32 discard = mpp_frame_get_discard(frame);
				if (errinfo || discard) {
					const char* reason = "decoder-issue";
					if (errinfo && discard) {
						reason = "decoder-errinfo+discard";
					} else if (errinfo) {
						reason = "decoder-errinfo";
					} else if (discard) {
						reason = "decoder-discard";
					}
					idr_request_decoder_issue(reason);
				}
				if (!mpi.first_frame_ts.tv_sec) {
					ts = ats;
					mpi.first_frame_ts = ats;
				}

				MppBuffer buffer = mpp_frame_get_buffer(frame);
				if (buffer) {
					output_list->video_poc = mpp_frame_get_poc(frame);
					uint64_t feed_data_ts =  mpp_frame_get_pts(frame);

					MppBufferInfo info;
					ret = mpp_buffer_info_get(buffer, &info);
					assert(!ret);
					for (i=0; i<MAX_FRAMES; i++) {
						if (mpi.frame_to_drm[i].prime_fd == info.fd) break;
					}
					assert(i!=MAX_FRAMES);

					ts = ats;

					// send DRM FB to display thread
					ret = pthread_mutex_lock(&video_mutex);
					assert(!ret);
					output_list->video_fb_id = mpi.frame_to_drm[i].fb_id;
                    //output_list->video_fb_index=i;
                    output_list->decoding_pts=feed_data_ts;
					ret = pthread_cond_signal(&video_cond);
					assert(!ret);
					ret = pthread_mutex_unlock(&video_mutex);
					assert(!ret);

					if (frame_proc != nullptr &&
					    decoded_hor_stride > 0 && decoded_ver_stride > 0) {
						MppFrameFormat fmt = mpp_frame_get_fmt(frame);
						frame_proc->push_latest(buffer,
						                       output_list->video_frm_width,
						                       output_list->video_frm_height,
						                       decoded_hor_stride,
						                       decoded_ver_stride, fmt);
					}
				}
			}
			
			frm_eos = mpp_frame_get_eos(frame);
			mpp_frame_deinit(&frame);
			frame = NULL;
		} else assert(0);
	}
	spdlog::info("Frame thread done.");
	return nullptr;
}


void *__DISPLAY_THREAD__(void *param)
{
	int ret;	
	pthread_setname_np(pthread_self(), "__DISPLAY");

	while (!frm_eos) {
		int fb_id;
		bool osd_update;

		ret = pthread_mutex_lock(&video_mutex);
		assert(!ret);
		while (output_list->video_fb_id==0 && !osd_update_ready) {
			// Timed, not a plain wait: with no --osd, nothing else would
			// ever wake this loop while the active stream is stale (no
			// new frames means no signal, ever, until the stream
			// recovers) -- the staleness check below needs a periodic
			// wake of its own to ever run.
			struct timespec ts;
			clock_gettime(CLOCK_REALTIME, &ts);
			ts.tv_nsec += 300L * 1000000L;
			if (ts.tv_nsec >= 1000000000L) { ts.tv_sec += 1; ts.tv_nsec -= 1000000000L; }
			int wret = pthread_cond_timedwait(&video_cond, &video_mutex, &ts);
			assert(!ret);
			if (output_list->video_fb_id == 0 && frm_eos) {
				ret = pthread_mutex_unlock(&video_mutex);
				assert(!ret);
				goto end;
			}
			if (wret == ETIMEDOUT && g_stream_manager && g_stream_manager->is_active_stream_stale()) {
				break; // handle stale-active-stream display below
			}
		}
		fb_id = output_list->video_fb_id;
		osd_update = osd_update_ready;

        uint64_t decoding_pts=fb_id != 0 ? output_list->decoding_pts : get_time_ms();
		output_list->video_fb_id=0;
		osd_update_ready = false;
		ret = pthread_mutex_unlock(&video_mutex);
		assert(!ret);

		// Multistream switcher only: the active stream's source can go
		// away (air unit off/out of range) with no event telling us so --
		// only an absence of new frames. Without this, fb_id stays 0 here
		// forever (see the loop above) and the video plane's FB_ID
		// property simply never gets touched again below, so it just
		// keeps showing whatever frame was already committed, frozen.
		bool stream_stale = g_stream_manager && g_stream_manager->is_active_stream_stale();
		if (stream_stale) {
			fb_id = (int)g_stream_manager->ensure_no_signal_fb(
				(uint32_t)output_list->video_frm_width, (uint32_t)output_list->video_frm_height);
		}
		set_no_signal_indicator(stream_stale);

		// create new video_request
		drmModeAtomicFree(output_list->video_request);
		output_list->video_request = drmModeAtomicAlloc();

		// show DRM FB in plane
		uint32_t flags = DRM_MODE_ATOMIC_NONBLOCK;
		if (fb_id != 0) {
			flags = disable_vsync ? DRM_MODE_ATOMIC_NONBLOCK : DRM_MODE_ATOMIC_ALLOW_MODESET;
			ret = set_drm_object_property(output_list->video_request, &output_list->video_plane, "FB_ID", fb_id);
			assert(ret>0);
		}

		if(enable_osd) {
			ret = pthread_mutex_lock(&osd_mutex);
			assert(!ret);
			if (enable_live_colortrans)
				ret = set_drm_object_property(output_list->video_request, &output_list->osd_plane, "FB_ID", output_list->osd_bufs[output_list->osd_buf_switch].gl_fb_id);
			else 
				ret = set_drm_object_property(output_list->video_request, &output_list->osd_plane, "FB_ID", output_list->osd_bufs[output_list->osd_buf_switch].fb);
			assert(ret>0);
		}
		drmModeAtomicCommit(drm_fd, output_list->video_request, flags, NULL);
		ret = pthread_mutex_unlock(&osd_mutex);
		assert(!ret);
		osd_publish_uint_fact("video.displayed_frame", NULL, 0, 1);
		uint64_t decode_and_handover_display_ms=get_time_ms()-decoding_pts;
		osd_publish_uint_fact("video.decode_and_handover_ms", NULL, 0, decode_and_handover_display_ms);
	}
end:	
	spdlog::info("Display thread done.");
	return nullptr;
}

// signal

int signal_flag = 0;
int return_value = 0;

void sig_handler(int signum)
{
	spdlog::info("Received signal {}", signum);
	signal_flag++;
	mavlink_thread_signal++;
	wfb_thread_signal++;
	osd_thread_signal++;
	if (dvr_raw != NULL) {
		dvr_raw->shutdown();
	}
	if (dvr_reenc_inst != NULL) {
		dvr_reenc_inst->shutdown();
	}
	if (frame_proc != NULL) {
		frame_proc->shutdown();
	}
	if (reencoder != NULL) {
		reencoder->shutdown();
	}
	return_value = signum;
}

void sigusr1_handler(int signum) {
	spdlog::info("Received signal {}", signum);
	bool was_enabled = dvr_enabled;
	if (was_enabled) {
		// Stopping
		if (dvr_raw) dvr_raw->stop_recording();
		if (dvr_reenc_inst) dvr_reenc_inst->stop_recording();
		dvr_enabled = 0;
		osd_publish_bool_fact("dvr.recording", NULL, 0, false);
	} else {
		// Starting
		dvr_enabled = 1;
		osd_publish_bool_fact("dvr.recording", NULL, 0, true);
		if (dvr_raw) dvr_raw->start_recording();
		if (dvr_reenc_inst) dvr_reenc_inst->start_recording();
		if (reencoder) reencoder->request_idr();
	}
}

void sigusr2_handler(int signum) {
    // Toggle the disable_vsync flag
    disable_vsync = disable_vsync ^ 1;

    // Open the file for writing
    std::ofstream outFile("/run/pixelpilot.msg");
    if (!outFile.is_open()) {
        spdlog::error("Error opening file!");
        return; // Exit the function if the file cannot be opened
    }

    // Write the formatted text to the file
    outFile << "disable_vsync: " << std::boolalpha << disable_vsync << std::endl;
    outFile.close();

    // Log the new state of disable_vsync
    spdlog::info("disable_vsync: {}", disable_vsync);
}

// Helper: create a filename template with a suffix inserted before the extension.
// Returns a strdup'd string — caller must free.
static char* dvr_template_with_suffix(const char *tpl, const char *suffix) {
    std::string s(tpl);
    auto dot = s.rfind('.');
    if (dot != std::string::npos)
        s.insert(dot, suffix);
    else
        s.append(suffix);
    return strdup(s.c_str());
}

// Shutdown helper for DVR + encoder teardown context.
struct DvrShutdownCtx {
    Dvr          *dvr_inst;
    FrameProcessor *p;
    MppEncoder   *e;
    pthread_t     td, tp, te;
};

static void *dvr_shutdown_worker(void *arg) {
    auto *ctx = static_cast<DvrShutdownCtx *>(arg);
    if (ctx->tp) pthread_join(ctx->tp, nullptr);
    if (ctx->te) pthread_join(ctx->te, nullptr);
    if (ctx->td) pthread_join(ctx->td, nullptr);
    delete ctx->p;
    delete ctx->e;
    delete ctx->dvr_inst;
    delete ctx;
    return nullptr;
}

// C-compatible interface for gsmenu live control of the DVR.
extern "C" {
    void dvr_reenc_set_fps(int fps) {
        if (dvr_reenc_inst) dvr_reenc_inst->stop_recording();
        reenc_params.fps = fps;
        if (dvr_reenc_inst) dvr_reenc_inst->set_video_framerate(fps);
        if (frame_proc) frame_proc->set_fps(fps);
        if (reencoder) reencoder->set_fps(fps);
    }
    void dvr_reenc_set_osd(int enabled) {
        dvr_osd = (bool)enabled;
        if (!enabled && frame_proc)
            frame_proc->set_osd_blend(-1, 0, 0, 0);
    }

    void dvr_reenc_notify_colortrans(int enabled) {
        if (!frame_proc) return;
        if (enabled)
            frame_proc->set_color_correction(live_colortrans_gain, live_colortrans_offset, drm_fd);
        else
            frame_proc->set_color_correction_enabled(false);
    }
    int dvr_reenc_get_fps(void)     { return reenc_params.fps; }
    int dvr_reenc_get_bitrate(void) { return reenc_params.bitrate_kbps; }
    int dvr_reenc_get_osd(void)     { return (int)dvr_osd; }
    int dvr_reenc_get_codec(void)   { return (int)reenc_params.codec - 1; } // 0=h264, 1=h265
    int dvr_reenc_get_resolution(void) { return (int)reenc_params.resolution; } // 0=720p, 1=1080p

    int  dvr_get_mode(void)  { return (int)dvr_mode; }
    // Deprecated — use dvr_get_mode() instead
    int  dvr_reenc_is_reenc(void) { return dvr_mode != DVR_MODE_RAW; }

    void dvr_set_max_size(int mb) {
        dvr_max_file_size = (int64_t)mb * 1000000LL;
        if (dvr_raw) dvr_raw->set_max_file_size(dvr_max_file_size);
        if (dvr_reenc_inst) dvr_reenc_inst->set_max_file_size(dvr_max_file_size);
        spdlog::info("DVR max file size set to {} MB", mb);
    }
    int dvr_get_max_size(void) { return (int)(dvr_max_file_size / 1000000LL); }

    void drm_set_video_scale(float factor) {
        if (output_list) {
            output_list->video_scale_factor = factor;
            modeset_apply_video_scale(drm_fd, output_list);
        }
    }

    void dvr_reenc_set_resolution(int idx) {
        if (dvr_reenc_inst) dvr_reenc_inst->stop_recording();
        reenc_params.resolution = (EncResolution)idx;
        if (frame_proc) frame_proc->set_resolution(reenc_params.resolution);
        if (dvr_reenc_inst) {
            uint32_t rw, rh; reenc_target_dims(rw, rh);
            dvr_reenc_inst->set_video_params(rw, rh, reenc_params.codec);
        }
    }

    void dvr_reenc_set_bitrate(int kbps) {
        reenc_params.bitrate_kbps = kbps;
        if (reencoder) reencoder->set_bitrate(kbps);
    }

    void dvr_reenc_set_codec(int idx) {
        if (dvr_reenc_inst) dvr_reenc_inst->stop_recording();
        VideoCodec vc = (idx == 1) ? VideoCodec::H265 : VideoCodec::H264;
        reenc_params.codec = vc;
        if (reencoder) reencoder->set_codec(vc);
        if (dvr_reenc_inst) {
            uint32_t rw, rh; reenc_target_dims(rw, rh);
            dvr_reenc_inst->set_video_params(rw, rh, vc);
        }
    }

    void dvr_start_all(void) {
        dvr_enabled = 1;
        osd_publish_bool_fact("dvr.recording", NULL, 0, true);
        if (dvr_raw) dvr_raw->start_recording();
        if (dvr_reenc_inst) dvr_reenc_inst->start_recording();
        if (reencoder) reencoder->request_idr();
    }

    void dvr_stop_all(void) {
        if (dvr_raw) dvr_raw->stop_recording();
        if (dvr_reenc_inst) dvr_reenc_inst->stop_recording();
        dvr_enabled = 0;
        osd_publish_bool_fact("dvr.recording", NULL, 0, false);
    }

    // Switch DVR mode at runtime. Stops any active recording.
    // mode: 0=raw, 1=reencode, 2=both
    void dvr_set_mode(int mode) {
        DvrMode new_mode = (DvrMode)mode;
        if (new_mode == dvr_mode) return;

        // Stop any active recording
        dvr_stop_all();

        bool old_has_raw   = (dvr_mode == DVR_MODE_RAW || dvr_mode == DVR_MODE_BOTH);
        bool old_has_reenc = (dvr_mode == DVR_MODE_REENCODE || dvr_mode == DVR_MODE_BOTH);
        bool new_has_raw   = (new_mode == DVR_MODE_RAW || new_mode == DVR_MODE_BOTH);
        bool new_has_reenc = (new_mode == DVR_MODE_REENCODE || new_mode == DVR_MODE_BOTH);

        // Tear down encoder pipeline if no longer needed
        if (old_has_reenc && !new_has_reenc) {
            FrameProcessor *p  = frame_proc;
            MppEncoder     *e  = reencoder;
            Dvr            *d  = dvr_reenc_inst;
            pthread_t       tp = g_tid_fproc;
            pthread_t       te = g_tid_enc;
            pthread_t       td = g_tid_dvr_reenc;
            frame_proc      = nullptr;
            reencoder       = nullptr;
            dvr_reenc_inst  = nullptr;
            g_tid_fproc     = 0;
            g_tid_enc       = 0;
            g_tid_dvr_reenc = 0;
            if (p) p->shutdown();
            if (e) e->shutdown();
            if (d) d->shutdown();
            auto *ctx = new DvrShutdownCtx{d, p, e, td, tp, te};
            pthread_t cleanup_tid;
            pthread_create(&cleanup_tid, NULL, dvr_shutdown_worker, ctx);
            pthread_detach(cleanup_tid);
        }

        // Tear down raw DVR if no longer needed
        if (old_has_raw && !new_has_raw) {
            Dvr *d = dvr_raw;
            pthread_t td = g_tid_dvr_raw;
            dvr_raw = nullptr;
            g_tid_dvr_raw = 0;
            if (d) d->shutdown();
            auto *ctx = new DvrShutdownCtx{d, nullptr, nullptr, td, 0, 0};
            pthread_t cleanup_tid;
            pthread_create(&cleanup_tid, NULL, dvr_shutdown_worker, ctx);
            pthread_detach(cleanup_tid);
        }

        // Create raw DVR if newly needed
        if (new_has_raw && !dvr_raw && dvr_template) {
            dvr_thread_params args;
            bool both = (new_mode == DVR_MODE_BOTH);
            char *tpl = both ? dvr_template_with_suffix(dvr_template, "_raw") : dvr_template;
            args.filename_template = tpl;
            args.mp4_fragmentation_mode = mp4_fragmentation_mode;
            args.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
            args.video_framerate = video_framerate;
            args.max_file_size = dvr_max_file_size;
            args.video_p.video_frm_width = output_list ? output_list->video_frm_width : 0;
            args.video_p.video_frm_height = output_list ? output_list->video_frm_height : 0;
            args.video_p.codec = codec;
            dvr_raw = new Dvr(args);
            pthread_create(&g_tid_dvr_raw, NULL, &Dvr::__THREAD__, dvr_raw);
        }

        // Create encoder pipeline + reenc DVR if newly needed
        if (new_has_reenc && !reencoder && dvr_template) {
            bool both = (new_mode == DVR_MODE_BOTH);
            char *tpl = both ? dvr_template_with_suffix(dvr_template, "_reenc") : dvr_template;
            dvr_thread_params args;
            args.filename_template = tpl;
            args.mp4_fragmentation_mode = mp4_fragmentation_mode;
            args.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
            args.video_framerate = reenc_params.fps;
            args.max_file_size = dvr_max_file_size;
            uint32_t rw, rh; reenc_target_dims(rw, rh);
            args.video_p.video_frm_width = rw;
            args.video_p.video_frm_height = rh;
            args.video_p.codec = reenc_params.codec;
            dvr_reenc_inst = new Dvr(args);
            pthread_create(&g_tid_dvr_reenc, NULL, &Dvr::__THREAD__, dvr_reenc_inst);

            reencoder = new MppEncoder(reenc_params,
                             [](std::shared_ptr<std::vector<uint8_t>> nal) {
                                 if (dvr_enabled && dvr_reenc_inst) dvr_reenc_inst->frame(nal);
                             });
            pthread_create(&g_tid_enc, NULL, &MppEncoder::__THREAD__, reencoder);
            frame_proc = new FrameProcessor(reencoder, reenc_params.fps, reenc_params.resolution, drm_fd);
            if (enable_live_colortrans)
                frame_proc->set_color_correction(live_colortrans_gain,
                                                live_colortrans_offset, drm_fd);
            pthread_create(&g_tid_fproc, NULL, &FrameProcessor::__THREAD__, frame_proc);
            dvr_reenc_inst->on_start_cb = []() { if (reencoder) reencoder->request_idr(); };
        }

        dvr_mode = new_mode;
        spdlog::info("DVR mode set to {}", mode == 0 ? "raw" : mode == 1 ? "reencode" : "both");
    }

    // Deprecated wrapper for backward compatibility
    void dvr_reenc_set_mode(int enabled) {
        dvr_set_mode(enabled ? DVR_MODE_REENCODE : DVR_MODE_RAW);
    }
}

int decoder_stalled_count=0;
bool feed_packet_to_decoder(MppPacket *packet,void* data_p,int data_len){
    mpp_packet_set_data(packet, data_p);
    mpp_packet_set_size(packet, data_len);
    mpp_packet_set_pos(packet, data_p);
    mpp_packet_set_length(packet, data_len);
    mpp_packet_set_pts(packet,(RK_S64) get_time_ms());
    // Feed the data to mpp until either timeout (in which case the decoder might have stalled)
    // or success
    uint64_t data_feed_begin = get_time_ms();
    int ret=0;
    while (!signal_flag && MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
        uint64_t elapsed = get_time_ms() - data_feed_begin;
        osd_publish_uint_fact("video.decoder_feed_time_ms", NULL, 0, elapsed);
        if (elapsed > 100) {
            decoder_stalled_count++;
            spdlog::warn("Cannot feed decoder, stalled {} ?", decoder_stalled_count);
            return false;
        }
        usleep(2 * 1000);
    }
    return true;
}

std::unique_ptr<GstRtpReceiver> receiver;
static MppCodingType current_mpp_type = MPP_VIDEO_CodingHEVC;
static MppCodingType stream_mpp_type  = MPP_VIDEO_CodingHEVC;

static void reinit_mpp_decoder(MppCodingType new_type) {
    if (new_type == current_mpp_type) return;
    spdlog::info("Reinitializing MPP decoder: {} -> {}",
                 current_mpp_type == MPP_VIDEO_CodingHEVC ? "H.265" : "H.264",
                 new_type == MPP_VIDEO_CodingHEVC ? "H.265" : "H.264");
    // Signal frame thread to release the lock, then acquire it
    mpp_reinit_pending.store(true, std::memory_order_release);
    mpi.mpi->reset(mpi.ctx);
    pthread_mutex_lock(&mpp_reinit_mutex);

    mpp_destroy(mpi.ctx);
    mpi.ctx = nullptr;
    mpi.mpi = nullptr;
    int ret = mpp_create(&mpi.ctx, &mpi.mpi);
    assert(!ret);
    set_mpp_decoding_parameters(mpi.mpi, mpi.ctx);
    ret = mpp_init(mpi.ctx, MPP_CTX_DEC, new_type);
    assert(!ret);
    set_mpp_decoding_parameters(mpi.mpi, mpi.ctx);
    int param = MPP_POLL_BLOCK;
    ret = mpi.mpi->control(mpi.ctx, MPP_SET_OUTPUT_BLOCK, &param);
    assert(!ret);
    current_mpp_type = new_type;

    mpp_reinit_pending.store(false, std::memory_order_release);
    pthread_mutex_unlock(&mpp_reinit_mutex);
}

void switch_pipeline_source(const char * source_type, const char * source_path) {
    if (strcmp(source_type, "file") == 0) {
        VideoCodec file_codec = receiver->switch_to_file_playback(source_path);
        MppCodingType new_type = (file_codec == VideoCodec::H265)
            ? MPP_VIDEO_CodingHEVC : MPP_VIDEO_CodingAVC;
        reinit_mpp_decoder(new_type);
    } else if (strcmp(source_type, "stream") == 0) {
        receiver->switch_to_stream();
        reinit_mpp_decoder(stream_mpp_type);
    } else {
        spdlog::error("Unknown source type: {}", source_type);
    }
}

void fast_forward(double rate){ 
        receiver->fast_forward();
}

void fast_rewind(double rate){ 
        receiver->fast_rewind();
}

void skip_duration(int64_t skip_ms){
        receiver->skip_duration(skip_ms);
}

void normal_playback() { 
        receiver->normal_playback();
}

void pause_playback() {
        receiver->pause();
}

void resume_playback() {
        receiver->resume();
}

class CustomMsgManager {
public:
    CustomMsgManager(const char* fifoName, bool enabled) : fifoName(fifoName), fd(-1), enabled(enabled) {}

    int open_fifo() {
        if (!enabled) {
            return 0;
        }
        // Kill FIFO if already exists
        if (access(fifoName, F_OK) == 0) {
            unlink(fifoName);
        }

        if (mkfifo(fifoName, 0622) == -1) {
            spdlog::error("Failed to create FIFO {}: {}", fifoName,  strerror(errno));
            return -1;
        }

        // Change permissions to allow write for everyone
        if (chmod(fifoName, 0622) == -1) {
            spdlog::error("Failed to change permissions {}: {}", fifoName, strerror(errno));
            return -2;
        }

        // Open the FIFO for reading
        fd = open(fifoName, O_RDONLY | O_NONBLOCK);
        if (fd == -1) {
            spdlog::error("Failed to open FIFO {}: {}", fifoName, strerror(errno));
            return -3;
        }

        return 0;
    }

    void check_message() {
        if (!enabled) {
            return;
        } else if (fd == -1) {
            spdlog::error("FIFO is not initialized.");
            return; // Avoid reading if FIFO is not initialized
        }

        // fd is non-blocking
        char chunk[120];
        ssize_t bytes_read = read(fd, chunk, sizeof(chunk));
        if (bytes_read > 0) {
            buffer.append(chunk, bytes_read);
        }

        // Emit one fact per `\n`-terminated message. If the buffer grows past
        // MAX_MSG_LEN without a newline, flush it anyway so a stuck writer
        // cannot make us grow unboundedly.
        while (true) {
            size_t nl = buffer.find('\n');
            if (nl == std::string::npos) {
                if (buffer.size() >= MAX_MSG_LEN) {
                    publish_message(buffer);
                    buffer.clear();
                }
                break;
            }
            std::string msg = buffer.substr(0, nl);
            buffer.erase(0, nl + 1);
            if (!msg.empty()) {
                publish_message(msg);
            }
        }
    }

    // Unescape `\\n` -> literal newline so writers that cannot emit a raw
    // newline (eg shell `echo` without `-e`) can still build multi-line
    // messages.
    static std::string unescape_newlines(const std::string& in) {
        std::string out;
        out.reserve(in.size());
        for (size_t i = 0; i < in.size(); ++i) {
            if (in[i] == '\\' && i + 1 < in.size() && in[i + 1] == 'n') {
                out.push_back('\n');
                ++i;
            } else {
                out.push_back(in[i]);
            }
        }
        return out;
    }

    void publish_message(const std::string& raw) {
        osd_tag tags[1];
        strcpy(tags[0].key, "file");
        strcpy(tags[0].val, fifoName);
        std::string msg = unescape_newlines(raw);
        osd_publish_str_fact("osd.custom_message", tags, 1, msg.c_str());
    }

    ~CustomMsgManager() {
        if (fd != -1) {
            close(fd);
        }
        unlink(fifoName);
    }

private:
    static constexpr size_t MAX_MSG_LEN = 512;

    const char* fifoName;
    int fd; // File descriptor for the FIFO
    bool enabled;
    std::string buffer; // accumulates partial reads until a `\n` is seen
};

void main_loop() {
    CustomMsgManager msg_manager(MSG_FIFO_NAME, osd_custom_message);

    if (msg_manager.open_fifo() != 0) {
        return;
    }

    while (!signal_flag) {
        // TODO: put gsmenu main loop here
        msg_manager.check_message();
		os_sensors.run();
		if (RXMODE == APFPV) {
    		wifi_monitor.run();
		}
        sleep(1);
    }
    return;
}

uint64_t first_frame_ms=0;
void read_gstreamerpipe_stream(MppPacket *packet, int gst_udp_port, const char *sock ,const VideoCodec& codec){
	if (sock) {
		receiver = std::make_unique<GstRtpReceiver>(sock, codec);
	} else {
		receiver = std::make_unique<GstRtpReceiver>(gst_udp_port, codec);
	}
	long long bytes_received = 0; 
	uint64_t period_start=0;
    auto cb=[&packet,/*&decoder_stalled_count,*/ &bytes_received, &period_start](std::shared_ptr<std::vector<uint8_t>> frame){
        // Let the gst pull thread run at quite high priority
        static bool first= false;
        static int stall_count = 0;
        static uint64_t last_stall_idr_ms = 0;
        if(first){
            SchedulingHelper::set_thread_params_max_realtime("DisplayThread",SchedulingHelper::PRIORITY_REALTIME_LOW);
            first= false;
        }
		bytes_received += frame->size();
		uint64_t now = get_time_ms();
		osd_publish_uint_fact("gstreamer.received_bytes", NULL, 0, frame->size());
        const bool fed_ok = feed_packet_to_decoder(packet,frame->data(),frame->size());
        if (!fed_ok) {
            stall_count++;
            if (stall_count >= 3 && (now - last_stall_idr_ms) > 500) {
                last_stall_idr_ms = now;
                stall_count = 0;
                idr_request_decoder_issue("decoder-feed-stall");
            }
        } else {
            stall_count = 0;
        }
        if (dvr_enabled && dvr_raw != NULL) {
			dvr_raw->frame(frame);
        }
    };
    receiver->start_receiving(cb);
    main_loop();
    receiver->stop_receiving();
    spdlog::info("Feeding eos");
    mpp_packet_set_eos(packet);
    //mpp_packet_set_pos(packet, nal_buffer);
    mpp_packet_set_length(packet, 0);
    int ret=0;
    while (MPP_OK != (ret = mpi.mpi->decode_put_packet(mpi.ctx, packet))) {
        usleep(10000);
    }
};


void set_control_verbose(MppApi * mpi,  MppCtx ctx,MpiCmd control,RK_U32 enable){
    RK_U32 res = mpi->control(ctx, control, &enable);
    if(res){
        spdlog::warn("Could not set control {} {}", static_cast<int>(control), enable);
        assert(false);
    }
}

void set_mpp_decoding_parameters(MppApi * mpi,  MppCtx ctx) {
    // config for runtime mode
    MppDecCfg cfg       = NULL;
    mpp_dec_cfg_init(&cfg);
    // get default config from decoder context
    int ret = mpi->control(ctx, MPP_DEC_GET_CFG, cfg);
    if (ret) {
        spdlog::warn("{} failed to get decoder cfg ret {}", ctx, ret);
        assert(false);
    }
    // split_parse is to enable mpp internal frame spliter when the input
    // packet is not aplited into frames.
    RK_U32 need_split   = 1;
    ret = mpp_dec_cfg_set_u32(cfg, "base:split_parse", need_split);
    if (ret) {
        spdlog::warn("{} failed to set split_parse ret {}", ctx, ret);
        assert(false);
    }
    ret = mpi->control(ctx, MPP_DEC_SET_CFG, cfg);
    if (ret) {
        spdlog::warn("{} failed to set cfg {} ret {}", ctx, cfg, ret);
        assert(false);
    }
	int mpp_split_mode =0;
    set_control_verbose(mpi,ctx,MPP_DEC_SET_PARSER_SPLIT_MODE, mpp_split_mode ? 0xffff : 0);
    set_control_verbose(mpi,ctx,MPP_DEC_SET_DISABLE_ERROR, 0xffff);
    set_control_verbose(mpi,ctx,MPP_DEC_SET_IMMEDIATE_OUT, 0xffff);
    set_control_verbose(mpi,ctx,MPP_DEC_SET_ENABLE_FAST_PLAY, 0xffff);
    //set_control_verbose(mpi,ctx,MPP_DEC_SET_ENABLE_DEINTERLACE, 0xffff);
    // Docu fast mode:
    // and improve the
    // parallelism of decoder hardware and software
    // we probably don't want that, since we don't need pipelining to hit our bitrate(s)
    int fast_mode = 0;
    set_control_verbose(mpi,ctx,MPP_DEC_SET_PARSER_FAST_MODE,fast_mode);
}

void printHelp() {
  printf(
    "\n\t\tPixelPilot FPV Decoder for Rockchip (%d.%d)\n"
    "\n"
    "  Usage:\n"
    "    pixelpilot [Arguments]\n"
    "\n"
    "  Arguments:\n"
    "    --config <configfile>  - Load pixelpilot config from file      (Default: /etc/pixelpilot.yaml)\n"
    "\n"
    "    -p <port>              - UDP port for RTP video stream         (Default: 5600)\n"
    "\n"
    "    --socket <socket>      - read data from socket\n"
    "\n"
    "    --mavlink-port <port>  - UDP port for mavlink telemetry        (Default: 14550)\n"
    "\n"
    "    --mavlink-dvr-on-arm   - Start recording when armed\n"
    "\n"
    "    --codec <codec>        - Video codec, should be the same as on VTX  (Default: h265 <h264|h265>)\n"
    "\n"
    "    --log-level <level>         - Log verbosity level, debug|info|warn|error (Default: info)\n"
    "\n"
    "    --log-to-file [path]         - Also log to file, with rotation (Default if no path given: /var/log/pixelpilot/pixelpilot.log)\n"
    "\n"
    "    --log-file-max-size <MB>    - Max log file size in MB before rotation (Default: 10)\n"
    "\n"
    "    --log-file-max-files <N>    - Number of rotated log files to keep (Default: 3)\n"
    "\n"
    "    --osd                  - Enable OSD\n"
    "\n"
    "    --osd-config <file>    - Path to OSD configuration file\n"
    "\n"
    "    --osd-refresh <rate>   - Defines the delay between osd refresh (Default: 1000 ms)\n"
    "\n"
    "    --osd-custom-message   - Enables the display of /run/pixelpilot.msg (beta feature, may be removed)\n"
    "\n"
    "    --dvr-template <path>  - Save the video feed (no osd) to the provided filename template.\n"
    "                             DVR is toggled by SIGUSR1 signal\n"
    "                             Supports placeholders %%Y - year, %%m - month, %%d - day,\n"
    "                             %%H - hour, %%M - minute, %%S - second. Ex: /media/DVR/%%Y-%%m-%%d_%%H-%%M-%%S.mp4\n"
    "\n"
    "    --dvr-sequenced-files  - Prepend a sequence number to the names of the dvr files\n"
    "\n"
    "    --dvr-start            - Start DVR immediately\n"
    "\n"
    "    --dvr-framerate <rate> - Force the dvr framerate for smoother dvr, ex: 60\n"
    "\n"
    "    --dvr-max-size <MB>    - Split DVR files at <MB> megabytes (Default: 4000, for VFAT)\n"
    "\n"
    "    --dvr-fmp4             - Save the video feed as a fragmented mp4\n"
    "\n"
    "    --dvr-mode <mode>      - DVR recording mode: raw, reencode, or both (Default: raw)\n"
    "\n"
    "    --dvr-reenc-codec <c>  - Re-encode codec: h264 or h265  (Default: h264)\n"
    "\n"
    "    --dvr-reenc-bitrate <k>- Re-encode bitrate in kbps       (Default: 8000)\n"
    "\n"
    "    --dvr-reenc-fps <fps>  - Re-encode output FPS            (Default: 30)\n"
    "\n"
    "    --dvr-reenc-resolution <r> - Re-encode resolution: 720p or 1080p (Default: 1080p)\n"
    "\n"
    "    --dvr-osd              - Blend the OSD into the DVR recording\n"
    "\n"
    "    --display-reencode-osd - Burn OSD into display restream / display DVR\n"
    "\n"
    "    --screen-mode <mode>   - Override default screen mode. <width>x<heigth>@<fps> ex: 1920x1080@120\n"
    "\n"
    "    --video-plane-id       - Override default drm plane used for video by plane-id\n"
	"\n"
	"    --video-scale <factor> - Scale video output size (0.5 =< factor <= 1.0) (Default: 1.0)\n"
    "\n"
    "    --osd-plane-id         - Override default drm plane used for osd by plane-id\n"
    "\n"
    "    --disable-vsync        - Disable VSYNC commits\n"
    "\n"
    "    --disable-gregidr      - Disable last-hop probing and IDR requests\n"
    "\n"
    "    --restream-ip <ip>     - Auto-start restream to this IP on launch (stream 0, port 5600)\n"
    "\n"
    "    --restream <spec>      - Restream a stream or display. spec: <index>:<ip>[:<port>]\n"
    "                             index 0,1,...: raw RTP passthrough (default port 5600+index)\n"
    "                             index 'display' or '-1': re-encoded with OSD (default port 5609)\n"
    "                             Repeatable. Example: --restream 0:192.168.1.10 --restream display:192.168.1.10:5609\n"
    "\n"
    "    --live-colortrans      - Apply colortrans LUT to live display via DRM gamma\n"	
    "\n"
    "    --screen-mode-list     - Print the list of supported screen modes and exit.\n"
    "\n"
    "    --wfb-api-port         - Port of wfb-server for cli statistics. (Default: 8003)\n"
    "                             Use \"0\" to disable this stats\n"
    "\n"
    "    --wfb-api-host         - Host or IP of wfb-server for cli statistics. (Default: 127.0.0.1)\n"
    "\n"
    "    --version              - Show program version\n"
    "\n", APP_VERSION_MAJOR, APP_VERSION_MINOR
  );
}

// Multistream switcher hooks, called from input.cpp's GPIO/keyboard
// handlers. No-ops if not running in multistream mode (g_stream_manager
// stays null when no --stream flags were given).
void switch_to_next_stream(void) {
	if (g_stream_manager) g_stream_manager->switch_to_next();
}
void switch_to_prev_stream(void) {
	if (g_stream_manager) g_stream_manager->switch_to_prev();
}

// main
#ifndef TEST

int main(int argc, char **argv)
{
	// Idempotent -- GstRtpReceiver's constructor also calls gst_init_check()
	// for the legacy single-stream path, but the multistream path
	// (stream_pipeline.cpp) calls gst_parse_launch() directly without ever
	// constructing a GstRtpReceiver, so it needs this done explicitly here
	// instead of relying on that side effect.
	gst_init(nullptr, nullptr);

	int ret;
	int i, j;
	int mavlink_thread = 0;
	int print_modelist = 0;
	int dvr_autostart = 0;
	uint16_t wfb_port = 8003;
	const char *wfb_api_host = "127.0.0.1";
	uint16_t mode_width = 0;
	uint16_t mode_height = 0;
	uint32_t mode_vrefresh = 0;
	char * config_file_path = NULL;
	std::string osd_config_path;
	auto log_level = spdlog::level::info;
	const std::string default_log_file = "/var/log/pixelpilot/pixelpilot.log";
	std::string log_file = "";
	int log_max_size_mb = 10;
	int log_max_files = 3;
	
    std::string pidFilePath = "/run/pixelpilot.pid";
    std::ofstream pidFile(pidFilePath);
    pidFile << getpid();
    pidFile.close();
	float video_scale_factor = 1.0;

	// Multistream switcher: one entry per --stream flag. Empty means legacy
	// single-stream mode (-p/--codec), preserving today's exact behavior --
	// see stream_manager.h for the new path, only taken when non-empty.
	struct StreamArg { int port; VideoCodec stream_codec; };
	std::vector<StreamArg> stream_args;
	std::unique_ptr<StreamManager> stream_manager;

	// --restream <index>:<ip>[:<port>]  index=-1 means display restream
	struct RestreamSpec { int index; std::string ip; int port; };
	std::vector<RestreamSpec> restream_specs;

	std::string restream_ip_arg;

	// Load console arguments
	__BeginParseConsoleArguments__(printHelp) 

	__OnArgument("-p") {
		listen_port = atoi(__ArgValue);
		continue;
	}
	
	__OnArgument("--socket") {
		unix_socket = const_cast<char*>(__ArgValue);
		continue;
	}

	__OnArgument("--config") {
		// Already handled above, just skip
		config_file_path = const_cast<char*>(__ArgValue);
		continue;
	}	

	__OnArgument("--codec") {
		char * codec_str = const_cast<char*>(__ArgValue);
		codec = video_codec(codec_str);
		if (codec == VideoCodec::UNKNOWN ) {
			fprintf(stderr, "unsupported video codec");
			return -1;
		}
		continue;
	}

	// --stream <port>[:<codec>], repeatable. Any --stream flag present at
	// all switches the whole process into multistream-switcher mode,
	// ignoring -p entirely (see stream_args branch further below) --
	// codec, if omitted, resolves against --codec's value (whatever it
	// ends up being after the whole argv is parsed, not just up to here).
	__OnArgument("--stream") {
		char buf[64];
		const char *arg = __ArgValue;
		if (strlen(arg) >= sizeof(buf)) {
			fprintf(stderr, "--stream argument too long: %s\n", arg);
			return -1;
		}
		strcpy(buf, arg);
		char *colon = strchr(buf, ':');
		VideoCodec stream_codec = VideoCodec::UNKNOWN; // UNKNOWN = "use --codec's value"
		if (colon) {
			*colon = '\0';
			stream_codec = video_codec(colon + 1);
			if (stream_codec == VideoCodec::UNKNOWN) {
				fprintf(stderr, "unsupported codec in --stream %s\n", arg);
				return -1;
			}
		}
		int port = atoi(buf);
		if (port <= 0) {
			fprintf(stderr, "invalid port in --stream %s\n", arg);
			return -1;
		}
		stream_args.push_back({port, stream_codec});
		continue;
	}

	__OnArgument("--dvr-start") {
		dvr_autostart = 1;
		continue;
	}

	__OnArgument("--dvr-template") {
		dvr_template = const_cast<char*>(__ArgValue);
		continue;
	}

	__OnArgument("--dvr-sequenced-files") {
		dvr_filenames_with_sequence = true;
		continue;
	}

	__OnArgument("--dvr-framerate") {
		video_framerate = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--dvr-max-size") {
		int mb = atoi(__ArgValue);
		if (mb <= 0) {
			fprintf(stderr, "invalid --dvr-max-size value\n");
			return -1;
		}
		dvr_max_file_size = (int64_t)mb * 1000000LL;
		continue;
	}

	__OnArgument("--dvr-fmp4") {
		mp4_fragmentation_mode = 1;
		continue;
	}

	__OnArgument("--dvr-mode") {
		const char *v = __ArgValue;
		if (!strcmp(v, "raw")) dvr_mode = DVR_MODE_RAW;
		else if (!strcmp(v, "reencode")) dvr_mode = DVR_MODE_REENCODE;
		else if (!strcmp(v, "both")) dvr_mode = DVR_MODE_BOTH;
		else {
			fprintf(stderr, "unsupported --dvr-mode (use raw, reencode, or both)\n");
			return -1;
		}
		continue;
	}

	__OnArgument("--dvr-reenc-codec") {
		VideoCodec c = video_codec(const_cast<char*>(__ArgValue));
		if (c == VideoCodec::UNKNOWN) {
			fprintf(stderr, "unsupported codec for --dvr-reenc-codec (use h264 or h265)\n");
			return -1;
		}
		reenc_params.codec = c;
		continue;
	}

	__OnArgument("--dvr-reenc-bitrate") {
		reenc_params.bitrate_kbps = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--dvr-reenc-fps") {
		reenc_params.fps = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--dvr-reenc-resolution") {
		const char *v = __ArgValue;
		if (!strcmp(v, "720p")) reenc_params.resolution = EncResolution::Res720p;
		else if (!strcmp(v, "1080p")) reenc_params.resolution = EncResolution::Res1080p;
		else {
			fprintf(stderr, "unsupported resolution for --dvr-reenc-resolution (use 720p or 1080p)\n");
			return -1;
		}
		continue;
	}

	__OnArgument("--dvr-osd") {
		dvr_osd = true;
		continue;
	}

	__OnArgument("--display-reencode-osd") {
		display_reencode_osd = true;
		continue;
	}

	__OnArgument("--log-level") {
		std::string log_l = std::string(__ArgValue);
		if (log_l == "info") {
			log_level = spdlog::level::info;
		} else if (log_l == "debug"){
			log_level = spdlog::level::debug;
		} else if (log_l == "warn"){
			log_level = spdlog::level::warn;
		} else if (log_l == "error"){
			log_level = spdlog::level::err;
		} else {
			fprintf(stderr, "invalid log level %s\n", log_l.c_str());
			printHelp();
			return -1;
		}
		continue;
	}

	__OnArgument("--log-to-file") {
		// Optional path argument: only consume the next token if it doesn't
		// look like another flag, so "--log-to-file --osd" still works.
		if (ArgID + 1 < argc && argv[ArgID + 1][0] != '-')
			log_file = argv[++ArgID];
		else
			log_file = default_log_file;
		continue;
	}

	__OnArgument("--log-file-max-size") {
		log_max_size_mb = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--log-file-max-files") {
		log_max_files = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--mavlink-port") {
		mavlink_port = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--mavlink-dvr-on-arm") {
		mavlink_dvr_on_arm = true;
		continue;
	}

	__OnArgument("--osd") {
		enable_osd = 1;
		mavlink_thread = 1;
		continue;
	}
	__OnArgument("--osd-config") {
		osd_config_path = std::string(__ArgValue);
		continue;
	}
	__OnArgument("--osd-refresh") {
		refresh_frequency_ms = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--osd-elements") {
		spdlog::warn("--osd-elements parameter is removed.");
		char* elements = const_cast<char*> (__ArgValue);
		continue;
	}

	__OnArgument("--osd-telem-lvl") {
		spdlog::warn("--osd-telem-lvl parameter is removed.");
		continue;
	}

	__OnArgument("--osd-custom-message") {
		osd_custom_message = true;
		continue;
	}

	__OnArgument("--screen-mode") {
		char* mode = const_cast<char*>(__ArgValue);
		mode_width = atoi(strtok(mode, "x"));
		mode_height = atoi(strtok(NULL, "@"));
		mode_vrefresh = atoi(strtok(NULL, "@"));
		continue;
	}

	__OnArgument("--disable-vsync") {
		disable_vsync = true;
		continue;
	}

	__OnArgument("--disable-gregidr") {
		disable_gregidr = true;
		continue;
	}

	__OnArgument("--screen-mode-list") {
		print_modelist = 1;
		continue;
	}

	__OnArgument("--live-colortrans") {
		enable_live_colortrans = true;
		continue;
	}

	__OnArgument("--wfb-api-port") {
		wfb_port = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--wfb-api-host") {
		wfb_api_host = const_cast<char *>(__ArgValue);
		continue;
	}

	__OnArgument("--version") {
		printf("PixelPilot Rockchip %d.%d\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);
		return 0;
	}

	__OnArgument("--video-plane-id") {
		video_plane_id_override = atoi(__ArgValue);
		continue;
	}
	__OnArgument("--osd-plane-id") {
		osd_plane_id_override = atoi(__ArgValue);
		continue;
	}

	__OnArgument("--video-scale") {
    	video_scale_factor = atof(__ArgValue);
    	if (video_scale_factor < 0.5 || video_scale_factor > 1.0) {
        	fprintf(stderr, "Invalid video scale factor, should be (0.5 =< scale <= 1.0)\n");
        	return -1;
    	}
    	continue;
	}

	__OnArgument("--restream-ip") {
		restream_ip_arg = std::string(__ArgValue);
		continue;
	}

	// --restream <index>:<ip>[:<port>]
	// index: 0,1,... for raw RTP passthrough; "display" or "-1" for OSD re-encode
	// port defaults: 5600+index for streams, 5609 for display
	__OnArgument("--restream") {
		const char *arg = __ArgValue;
		char buf[128];
		if (strlen(arg) >= sizeof(buf)) {
			fprintf(stderr, "--restream argument too long: %s\n", arg);
			return -1;
		}
		strcpy(buf, arg);
		// parse index part
		char *colon1 = strchr(buf, ':');
		if (!colon1) {
			fprintf(stderr, "--restream requires <index>:<ip>[:<port>], got: %s\n", arg);
			return -1;
		}
		*colon1 = '\0';
		int ridx;
		if (strcmp(buf, "display") == 0 || strcmp(buf, "-1") == 0) {
			ridx = -1;
		} else {
			ridx = atoi(buf);
			if (ridx < 0) {
				fprintf(stderr, "--restream invalid index: %s\n", arg);
				return -1;
			}
		}
		// parse ip and optional port
		char *ip_start = colon1 + 1;
		char *colon2 = strrchr(ip_start, ':');
		int rport = 0;
		if (colon2 && colon2 != ip_start) {
			*colon2 = '\0';
			rport = atoi(colon2 + 1);
		}
		if (*ip_start == '\0') {
			fprintf(stderr, "--restream missing ip in: %s\n", arg);
			return -1;
		}
		if (rport == 0) rport = (ridx >= 0) ? (5600 + ridx) : 5609;
		restream_specs.push_back({ridx, std::string(ip_start), rport});
		continue;
	}

	__EndParseConsoleArguments__

	// Resolve any --stream entries that omitted :codec against --codec's
	// final value (argv order shouldn't matter for this).
	for (auto &sa : stream_args) {
		if (sa.stream_codec == VideoCodec::UNKNOWN) sa.stream_codec = codec;
	}

	{
		std::vector<spdlog::sink_ptr> sinks;
		auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
		sinks.push_back(console_sink);
		if (!log_file.empty()) {
			try {
				std::filesystem::create_directories(std::filesystem::path(log_file).parent_path());
				auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
					log_file, log_max_size_mb * 1024UL * 1024UL, log_max_files, false);
				sinks.push_back(file_sink);
			} catch (const std::exception& e) {
				fprintf(stderr, "Could not open log file '%s': %s (continuing with console logging only)\n",
					log_file.c_str(), e.what());
			}
		}
		auto logger = std::make_shared<spdlog::logger>("pp", sinks.begin(), sinks.end());
		logger->set_level(log_level);
		if (log_level == spdlog::level::debug)
			logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%s:%#] [%^%l%$] %v");
		spdlog::set_default_logger(logger);
		logger->flush_on(spdlog::level::trace);
	}
	idr_set_enabled(!disable_gregidr);

	if (dvr_template != NULL && (dvr_mode == DVR_MODE_RAW || dvr_mode == DVR_MODE_BOTH) && video_framerate < 0) {
		printf("--dvr-framerate must be provided when raw DVR is enabled.\n"
		       "Use --dvr-mode reencode with --dvr-reenc-fps for hardware re-encoding only.\n");
		return 0;
	}

	printf("PixelPilot Rockchip %d.%d\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);

	// Load yaml config
	try {

		// Set default config path if none specified
		if (config_file_path == NULL) {
			config_file_path = strdup(DEFAULT_CONFIG_PATH);
		}
        config = YAML::LoadFile(config_file_path);

		// GSMENU settings
		if (config["gsmenu"]) {
            if (config["gsmenu"]["enabled"]) {
                gsmenu_enabled = config["gsmenu"]["enabled"].as<bool>();
            }
            if (config["gsmenu"]["transparency"]) {
                // Inverted here, once, so every downstream consumer
                // (style_init/apply_menu_transparency in gsmenu/styles.c)
                // can just call lv_obj_set_style_bg_opa() normally.
                // The OSD DRM plane's "pixel blend mode" is set to
                // Coverage (drm.c, modeset_atomic_prepare_commit) since
                // that's required for opacity to have any visible effect
                // at all on this driver -- but doing so empirically
                // inverts the visible result (0=opaque, 255=transparent)
                // from LVGL's normal bg_opa convention. Confirmed by
                // testing both extremes with and without that DRM
                // property set. Compensating here keeps the YAML key's
                // meaning ("transparency": low=see-through,
                // high=opaque) matching user expectation end-to-end.
                int yaml_value = config["gsmenu"]["transparency"].as<int>();
                gsmenu_transparency = 255 - yaml_value;
            }
            if (config["gsmenu"]["error_timeout_ms"]) {
                gsmenu_error_timeout_ms = config["gsmenu"]["error_timeout_ms"].as<int>();
            }
		if (gsmenu_enabled && config["gsmenu"]["actions"]) {
			if (config["gsmenu"]["actions"]["air"]) {
				const YAML::Node& actionsNode = config["gsmenu"]["actions"]["air"];
				airactions_count = 0;
				
				for (YAML::const_iterator it = actionsNode.begin(); 
					it != actionsNode.end() && airactions_count < MAX_ACTIONS; 
					++it) {
					
					std::string label = (*it)["label"].as<std::string>();
					std::string cmd = (*it)["action"].as<std::string>();
					
					// Access the global array at the current index
					strncpy(airactions[airactions_count].label, label.c_str(), MAX_LABEL_LEN - 1);
					airactions[airactions_count].label[MAX_LABEL_LEN - 1] = '\0';
					
					strncpy(airactions[airactions_count].action, cmd.c_str(), MAX_ACTION_LEN - 1);
					airactions[airactions_count].action[MAX_ACTION_LEN - 1] = '\0';
					
					airactions_count++;
				}
				spdlog::debug("Parsed {} GS Actions", airactions_count);
			}
			if (config["gsmenu"]["actions"]["ground"]) {
				const YAML::Node& actionsNode = config["gsmenu"]["actions"]["ground"];
				gsactions_count = 0;
				
				for (YAML::const_iterator it = actionsNode.begin(); 
					it != actionsNode.end() && gsactions_count < MAX_ACTIONS; 
					++it) {
					
					std::string label = (*it)["label"].as<std::string>();
					std::string cmd = (*it)["action"].as<std::string>();
					
					// Access the global array at the current index
					strncpy(gsactions[gsactions_count].label, label.c_str(), MAX_LABEL_LEN - 1);
					gsactions[gsactions_count].label[MAX_LABEL_LEN - 1] = '\0';
					
					strncpy(gsactions[gsactions_count].action, cmd.c_str(), MAX_ACTION_LEN - 1);
					gsactions[gsactions_count].action[MAX_ACTION_LEN - 1] = '\0';
					
					gsactions_count++;
				}
				spdlog::debug("Parsed {} GS Actions", gsactions_count);
			}
		}
		}

		if (config["restream"] && config["restream"]["manual_ip"]) {
			std::string ip = config["restream"]["manual_ip"].as<std::string>();
			restream_set_pinned_ip(ip.c_str());
		}

		if (!restream_ip_arg.empty()) {
			restream_set_pinned_ip(restream_ip_arg.c_str());
			restream_set_manual_ip(restream_ip_arg.c_str());
			restream_set_enabled(true);
		}

		if (config["os_sensors"] && config["os_sensors"].IsMap()) {
			if (config["os_sensors"]["cpu"]) {
				auto cpu = config["os_sensors"]["cpu"];
				if(cpu.IsScalar() && cpu.as<std::string>() == "auto") {
					os_sensors.discoverCPU();
				} else {
					os_sensors.addCPU();
				}
			}
			if (config["os_sensors"]["power"]) {
				auto power = config["os_sensors"]["power"];
				if(power.IsScalar() && power.as<std::string>() == "auto") {
					os_sensors.discoverPower();
				} else {
					for (const auto& power_sensor : power) {
						std::string type = power_sensor["type"].as<std::string>();
						std::string hwmon_id = power_sensor["hwmon_id"].as<std::string>();
						os_sensors.addPower(type, hwmon_id);
					}
				}
			}
			if (config["os_sensors"]["temperature"]) {
				auto temperature = config["os_sensors"]["temperature"];
				if(temperature.IsScalar() && temperature.as<std::string>() == "auto") {
					os_sensors.discoverTemperature();
				} else {
					for (const auto& temp_sensor : temperature) {
						std::string thermal_zone = temp_sensor["thermal_zone"].as<std::string>();
						os_sensors.addTemperature(thermal_zone);
					}
				}
			}
		} else {
			spdlog::error("Unexpected format of config file 'os_sensors'!");
		}
		os_sensors.addMemory();

	} catch (const YAML::BadFile& e) {
		std::cout << "Configuration file " << config_file_path << " not found." << std::endl;
	} catch (const YAML::ParserException& e) {
		std::cerr << "Error parsing configuration: " << e.what() << std::endl;
	} catch (const YAML::Exception& e) {
		std::cerr << "Configuration error: " << e.what() << std::endl;
	}

	spdlog::info("disable_vsync: {}", disable_vsync);

	if (enable_osd == 0 ) {
		video_zpos = 4;
	}
	
	MppCodingType mpp_type = MPP_VIDEO_CodingHEVC;
	if(codec==VideoCodec::H264) {
		mpp_type = MPP_VIDEO_CodingAVC;
	}
	current_mpp_type = mpp_type;
	stream_mpp_type  = mpp_type;
	ret = mpp_check_support_format(MPP_CTX_DEC, mpp_type);
	assert(!ret);
	
	//////////////////////////////////  DRM SETUP
	ret = modeset_open(&drm_fd, "/dev/dri/card0");
	if (ret < 0) {
		spdlog::warn("modeset_open() =  {}", ret);
	}
	assert(drm_fd >= 0);
	if (print_modelist) {
		modeset_print_modes(drm_fd);
		close(drm_fd);
		return 0;
	}

	output_list = modeset_prepare(drm_fd, mode_width, mode_height, mode_vrefresh, video_plane_id_override, osd_plane_id_override, video_scale_factor);
	if (!output_list) {
		fprintf(stderr,
				"cannot initialize display. Is display connected? Is --screen-mode correct?\n");
		return -2;
	}

	gamma_lut_controller_init(&lut_ctrl, drm_fd, output_list);

	if (enable_live_colortrans) {
		if (gamma_lut_enable(&lut_ctrl, live_colortrans_offset, live_colortrans_gain)) {
			spdlog::info("Gamma LUT enabled with offset={}, gain={}", live_colortrans_offset, live_colortrans_gain);
		}
	}
	
	////////////////////////////////// MPI SETUP
	// Skipped entirely in multistream mode -- each StreamPipeline owns its
	// own MPP context/packet instead of this single global one.
	MppPacket packet = nullptr;
	uint8_t* nal_buffer = nullptr;

	if (stream_args.empty()) {
		nal_buffer = (uint8_t*)malloc(1024 * 1024);
		assert(nal_buffer);
		ret = mpp_packet_init(&packet, nal_buffer, READ_BUF_SIZE);
		assert(!ret);

		ret = mpp_create(&mpi.ctx, &mpi.mpi);
		assert(!ret);
		set_mpp_decoding_parameters(mpi.mpi,mpi.ctx);
		ret = mpp_init(mpi.ctx, MPP_CTX_DEC, mpp_type);
		assert(!ret);
		set_mpp_decoding_parameters(mpi.mpi,mpi.ctx);

		// blocked/wait read of frame in thread
		int param = MPP_POLL_BLOCK;
		ret = mpi.mpi->control(mpi.ctx, MPP_SET_OUTPUT_BLOCK, &param);
		assert(!ret);
	}


	////////////////////////////////// SIGNAL SETUP

	signal(SIGINT, sig_handler);
	signal(SIGPIPE, sig_handler);
	if (dvr_template) {
		signal(SIGUSR1, sigusr1_handler);
	}
	signal(SIGUSR2, sigusr2_handler);
 	//////////////////// THREADS SETUP
	
	ret = pthread_mutex_init(&video_mutex, NULL);
	assert(!ret);
	ret = pthread_cond_init(&video_cond, NULL);
	assert(!ret);

	// Create display restream sink if --restream display:<ip>[:<port>] was given
	for (auto &rs : restream_specs) {
		if (rs.index == -1) {
			display_restream = new UdpRestream();
			if (!display_restream->init(rs.ip, rs.port, reenc_params.codec)) {
				spdlog::error("display restream init failed, disabling");
				delete display_restream;
				display_restream = nullptr;
			}
			break;
		}
	}

	// Display-only restream: use stream-friendly defaults so IDR frames fit in
	// a single UDP datagram.  User can override with --dvr-reenc-bitrate/resolution.
	if (display_restream && !dvr_template) {
		if (reenc_params.bitrate_kbps == 8000)
			reenc_params.bitrate_kbps = 1500;
		if (reenc_params.resolution == EncResolution::Res1080p)
			reenc_params.resolution = EncResolution::Res720p;
	}

	pthread_t tid_frame, tid_display, tid_osd, tid_mavlink, tid_wfbcli;
	if (dvr_template != NULL || display_restream != nullptr) {
		bool has_raw   = dvr_template && (dvr_mode == DVR_MODE_RAW || dvr_mode == DVR_MODE_BOTH);
		bool has_reenc = (dvr_template && (dvr_mode == DVR_MODE_REENCODE || dvr_mode == DVR_MODE_BOTH))
		                 || display_restream != nullptr;
		bool both      = dvr_template && (dvr_mode == DVR_MODE_BOTH);

		if (has_raw) {
			dvr_thread_params args;
			char *tpl = both ? dvr_template_with_suffix(dvr_template, "_raw") : dvr_template;
			args.filename_template = tpl;
			args.mp4_fragmentation_mode = mp4_fragmentation_mode;
			args.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
			args.video_framerate = video_framerate;
			args.max_file_size = dvr_max_file_size;
			args.video_p.video_frm_width = output_list->video_frm_width;
			args.video_p.video_frm_height = output_list->video_frm_height;
			args.video_p.codec = codec;
			dvr_raw = new Dvr(args);
			ret = pthread_create(&g_tid_dvr_raw, NULL, &Dvr::__THREAD__, dvr_raw);
			assert(!ret);
		}

		if (has_reenc) {
			if (dvr_template) {
				dvr_thread_params args;
				char *tpl = both ? dvr_template_with_suffix(dvr_template, "_reenc") : dvr_template;
				args.filename_template = tpl;
				args.mp4_fragmentation_mode = mp4_fragmentation_mode;
				args.dvr_filenames_with_sequence = dvr_filenames_with_sequence;
				args.video_framerate = reenc_params.fps;
				args.max_file_size = dvr_max_file_size;
				uint32_t rw, rh; reenc_target_dims(rw, rh);
				args.video_p.video_frm_width = rw;
				args.video_p.video_frm_height = rh;
				args.video_p.codec = reenc_params.codec;
				dvr_reenc_inst = new Dvr(args);
				ret = pthread_create(&g_tid_dvr_reenc, NULL, &Dvr::__THREAD__, dvr_reenc_inst);
				assert(!ret);
			}

			reencoder = new MppEncoder(reenc_params, [](std::shared_ptr<std::vector<uint8_t>> nal) {
				if (dvr_enabled && dvr_reenc_inst != NULL) {
					dvr_reenc_inst->frame(nal);
				}
				if (display_restream) {
					display_restream->send_nal(nal);
				}
			});
			ret = pthread_create(&g_tid_enc, NULL, &MppEncoder::__THREAD__, reencoder);
			assert(!ret);
			frame_proc = new FrameProcessor(reencoder, reenc_params.fps, reenc_params.resolution, drm_fd);
			if (display_restream) frame_proc->set_always_encode(true);
			if (enable_live_colortrans) {
				frame_proc->set_color_correction(live_colortrans_gain,
				                                live_colortrans_offset, drm_fd);
				spdlog::info("Encoder color correction enabled: gain={} offset={}",
				             live_colortrans_gain, live_colortrans_offset);
			}
			ret = pthread_create(&g_tid_fproc, NULL, &FrameProcessor::__THREAD__, frame_proc);
			assert(!ret);
			if (dvr_reenc_inst) {
				dvr_reenc_inst->on_start_cb = []() {
					if (reencoder) reencoder->request_idr();
				};
				spdlog::info("Re-encoding recorder: codec={} fps={} bitrate={}kbps",
				             reenc_params.codec == VideoCodec::H265 ? "h265" : "h264",
				             reenc_params.fps, reenc_params.bitrate_kbps);
			}
		}

		if (dvr_autostart) {
			dvr_enabled = 1;
			osd_publish_bool_fact("dvr.recording", NULL, 0, true);
			if (dvr_raw) dvr_raw->start_recording();
			if (dvr_reenc_inst) dvr_reenc_inst->start_recording();
			if (reencoder) reencoder->request_idr();
		}
	}
	if (stream_args.empty()) {
		ret = pthread_create(&tid_frame, NULL, __FRAME_THREAD__, NULL);
		assert(!ret);
	}
	ret = pthread_create(&tid_display, NULL, __DISPLAY_THREAD__, NULL);
	assert(!ret);
	if (enable_osd) {
		nlohmann::json osd_config;
		if(osd_config_path != "") {
			std::ifstream f(osd_config_path);
			osd_config = nlohmann::json::parse(f);
		} else {
			osd_config = {};
		}
		if (mavlink_thread) {
			ret = pthread_create(&tid_mavlink, NULL, __MAVLINK_THREAD__, &signal_flag);
			assert(!ret);
		}
		if (wfb_port) {
			wfb_thread_params *wfb_args = (wfb_thread_params *)malloc(sizeof *wfb_args);
			wfb_args->port = wfb_port;
			wfb_args->host = wfb_api_host;
			ret = pthread_create(&tid_wfbcli, NULL, __WFB_CLI_THREAD__, wfb_args);
			assert(!ret);
		}

		// new, not malloc: osd_thread_params::config is an nlohmann::json
		// (non-POD, real constructor/destructor) -- malloc'd memory never
		// runs that constructor, so the assignment below would be writing
		// into not-actually-a-json-object memory (undefined behavior).
		// Latent pre-existing bug, exposed here by the multistream path's
		// different heap allocation pattern shifting what garbage ends up
		// in that malloc'd block; not freed anywhere today (a startup-time,
		// once-per-process allocation) so switching to `new` needs no
		// matching deallocation-site change.
		osd_thread_params *args = new osd_thread_params();
        args->fd = drm_fd;
        args->out = output_list;
		args->config = osd_config;
		ret = pthread_create(&tid_osd, NULL, __OSD_THREAD__, args);
		assert(!ret);
	}

	////////////////////////////////////////////// MAIN LOOP
	if (stream_args.empty()) {
		read_gstreamerpipe_stream((void**)packet, listen_port, unix_socket, codec);
	} else {
		stream_manager = std::make_unique<StreamManager>(drm_fd, output_list, video_zpos);
		g_stream_manager = stream_manager.get();
		for (auto &sa : stream_args) {
			stream_manager->add_stream(sa.port, sa.stream_codec);
			spdlog::info("stream {}: udp:{} codec={}", stream_manager->stream_count() - 1, sa.port,
			             sa.stream_codec == VideoCodec::H264 ? "h264" : "h265");
		}

		// Wire per-stream restream targets from --restream args
		for (auto &rs : restream_specs) {
			if (rs.index >= 0) {
				if (rs.index >= stream_manager->stream_count()) {
					spdlog::warn("--restream index {} out of range (only {} streams)", rs.index, stream_manager->stream_count());
					continue;
				}
				stream_manager->stream(rs.index)->restream_configure(rs.ip, rs.port);
				spdlog::info("stream {}: restream → {}:{}", rs.index, rs.ip, rs.port);
			}
		}

		// --restream-ip backward compat: same as --restream 0:<ip>:5600
		if (!restream_ip_arg.empty() && stream_manager->stream_count() > 0) {
			bool already_set = false;
			for (auto &rs : restream_specs) { if (rs.index == 0) { already_set = true; break; } }
			if (!already_set) {
				stream_manager->stream(0)->restream_configure(restream_ip_arg, 5600);
				spdlog::info("stream 0: restream → {}:5600 (from --restream-ip)", restream_ip_arg);
			}
		}

		// Feed decoded frames from the active stream into frame_proc
		// (needed for display restream and DVR re-encode in multistream mode).
		if (frame_proc) {
			for (int i = 0; i < stream_manager->stream_count(); i++) {
				stream_manager->stream(i)->set_raw_frame_cb(
					[](MppBuffer buf, uint32_t w, uint32_t h,
					   uint32_t hs, uint32_t vs, MppFrameFormat fmt) {
						if (frame_proc) frame_proc->push_latest(buf, w, h, hs, vs, fmt);
					});
			}
		}

		stream_manager->start_all();
		main_loop();
		if (display_restream) { display_restream->stop(); delete display_restream; display_restream = nullptr; }
		stream_manager->stop_all();
		// __FRAME_THREAD__ never ran in this mode, so frm_eos was never set
		// by its own decoder-EOS path (the single-stream case's mechanism
		// for unblocking __DISPLAY_THREAD__'s loop condition below) -- set
		// it explicitly here instead, then force-wake the wait exactly like
		// the existing single-stream cleanup does for tid_frame already.
		frm_eos = 1;
	}

	////////////////////////////////////////////// MPI CLEANUP

	if (stream_args.empty()) {
		ret = pthread_join(tid_frame, NULL);
		assert(!ret);
	}

	ret = pthread_mutex_lock(&video_mutex);
	assert(!ret);	
	ret = pthread_cond_signal(&video_cond);
	assert(!ret);	
	ret = pthread_mutex_unlock(&video_mutex);
	assert(!ret);	

	ret = pthread_join(tid_display, NULL);
	assert(!ret);	
	
	ret = pthread_cond_destroy(&video_cond);
	assert(!ret);
	ret = pthread_mutex_destroy(&video_mutex);
	assert(!ret);

	if (mavlink_thread) {
		ret = pthread_join(tid_mavlink, NULL);
		assert(!ret);
        }
	if (enable_osd) {
		if (wfb_port) {
			ret = pthread_join(tid_wfbcli, NULL);
			assert(!ret);
		}
		ret = pthread_join(tid_osd, NULL);
		assert(!ret);
	}
	if (dvr_template != NULL) {
		if (g_tid_fproc) {
			ret = pthread_join(g_tid_fproc, NULL);
			assert(!ret);
		}
		if (g_tid_enc) {
			ret = pthread_join(g_tid_enc, NULL);
			assert(!ret);
		}
		if (g_tid_dvr_raw) {
			ret = pthread_join(g_tid_dvr_raw, NULL);
			assert(!ret);
		}
		if (g_tid_dvr_reenc) {
			ret = pthread_join(g_tid_dvr_reenc, NULL);
			assert(!ret);
		}
	}

	if (stream_args.empty()) {
		ret = mpi.mpi->reset(mpi.ctx);
		assert(!ret);

		if (mpi.frm_grp) {
			ret = mpp_buffer_group_put(mpi.frm_grp);
			assert(!ret);
			mpi.frm_grp = NULL;
			for (i=0; i<MAX_FRAMES; i++) {
				ret = drmModeRmFB(drm_fd, mpi.frame_to_drm[i].fb_id);
				assert(!ret);
				struct drm_mode_destroy_dumb dmdd;
				memset(&dmdd, 0, sizeof(dmdd));
				dmdd.handle = mpi.frame_to_drm[i].handle;
				do {
					ret = ioctl(drm_fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dmdd);
				} while (ret == -1 && (errno == EINTR || errno == EAGAIN));
				assert(!ret);
			}
		}

		mpp_packet_deinit(&packet);
		mpp_destroy(mpi.ctx);
		free(nal_buffer);
	}
	
	////////////////////////////////////////////// DRM CLEANUP
	restore_planes_zpos(drm_fd, output_list);
	drmModeSetCrtc(drm_fd,
			       output_list->saved_crtc->crtc_id,
			       output_list->saved_crtc->buffer_id,
			       output_list->saved_crtc->x,
			       output_list->saved_crtc->y,
			       &output_list->connector.id,
			       1,
			       &output_list->saved_crtc->mode);
	drmModeFreeCrtc(output_list->saved_crtc);
	drmModeAtomicFree(output_list->video_request);
	drmModeAtomicFree(output_list->osd_request);
	gamma_lut_cleanup(&lut_ctrl);
	modeset_cleanup(drm_fd, output_list);
	close(drm_fd);

    remove(pidFilePath.c_str());

	restore_stdin();
	return return_value;
}

#endif
