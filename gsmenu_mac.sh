#!/bin/bash
# macOS stub backend for PixelPilot GSMenu simulator.
# Mirrors every command in gsmenu.sh with realistic mock values so the full
# menu UI is navigable locally without a drone or ground-station hardware.
#
# State is persisted in $STATE_DIR so set commands are reflected on the next
# get (simulating real hardware behaviour).
#
# Usage:
#   GSMENU_BACKEND=./gsmenu_mac.sh ./build_sim/pixelpilot
#   ./sim.sh   (after setting GSMENU_BACKEND in sim.sh or env)

set -o pipefail

STATE_DIR="${TMPDIR:-/tmp}/gsmenu_mac_state"
mkdir -p "$STATE_DIR"

# ── Helpers ───────────────────────────────────────────────────────────────────

emit_values()     { printf '\x1e'"$1"; }
emit_values_cmd() { printf '\x1e'; "$@"; }

# Persist a key=value pair
state_set() { echo "$2" > "$STATE_DIR/$1"; }

# Read a persisted value, with a default if not yet set
state_get() {
    local key="$1" default="$2"
    if [ -f "$STATE_DIR/$key" ]; then
        cat "$STATE_DIR/$key"
    else
        echo "$default"
    fi
}

# Boolean helper: on/off -> 1/0
bool_set() {
    local key="$1" val="$2"
    [ "$val" = "on" ] && state_set "$key" 1 || state_set "$key" 0
}

# List WiFi channels available on this Mac
list_wifi_channels() {
    # airport is at a fixed path on macOS
    /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -s 2>/dev/null \
        | awk 'NR>1 && $3 != "" {print $3}' | sort -n | uniq \
        | awk '{print $1 " (2.4/5 GHz)"}' \
        | head -20
    # fallback static list if airport returns nothing
    if [ $? -ne 0 ]; then
        printf '36 (5 GHz)\n40 (5 GHz)\n44 (5 GHz)\n48 (5 GHz)\n149 (5 GHz)\n153 (5 GHz)\n157 (5 GHz)\n161 (5 GHz)'
    fi
}

# Return all IPv4 addresses on this Mac
local_ips() {
    ifconfig | grep 'inet ' | awk '{print $2}' | grep -v '^127\.'
}

# ── Main dispatch ─────────────────────────────────────────────────────────────

case "$*" in

# ════════════════════════════════════════════════════════════════════════════
# AIR: WFB-NG  (simulated — no real drone)
# ════════════════════════════════════════════════════════════════════════════

    "get air wfbng power")
        state_get air_wfbng_power 30
        emit_values "1\n20\n25\n30\n35\n40\n45\n50\n55\n58"
        ;;
    "set air wfbng power"*)
        state_set air_wfbng_power "$5"
        ;;

    "get air wfbng air_channel")
        state_get air_wfbng_channel "157 (5 GHz)"
        emit_values_cmd list_wifi_channels
        ;;
    "set air wfbng air_channel"*)
        state_set air_wfbng_channel "$5 $6 $7"
        ;;

    "get air wfbng width")
        state_get air_wfbng_width 20
        emit_values "20\n40"
        ;;
    "set air wfbng width"*)
        state_set air_wfbng_width "$5"
        ;;

    "get air wfbng mcs_index")
        state_get air_wfbng_mcs 1
        emit_values "0 10"
        ;;
    "set air wfbng mcs_index"*)
        state_set air_wfbng_mcs "$5"
        ;;

    "get air wfbng stbc")
        state_get air_wfbng_stbc 1
        ;;
    "set air wfbng stbc"*)
        bool_set air_wfbng_stbc "$5"
        ;;

    "get air wfbng ldpc")
        state_get air_wfbng_ldpc 1
        ;;
    "set air wfbng ldpc"*)
        bool_set air_wfbng_ldpc "$5"
        ;;

    "get air wfbng fec_k")
        state_get air_wfbng_fec_k 8
        emit_values "0 15"
        ;;
    "set air wfbng fec_k"*)
        state_set air_wfbng_fec_k "$5"
        ;;

    "get air wfbng fec_n")
        state_get air_wfbng_fec_n 12
        emit_values "0 15"
        ;;
    "set air wfbng fec_n"*)
        state_set air_wfbng_fec_n "$5"
        ;;

    "get air wfbng mlink")
        state_get air_wfbng_mlink 2000
        emit_values "1500\n2000\n2500\n3000\n3500\n4000"
        ;;
    "set air wfbng mlink"*)
        state_set air_wfbng_mlink "$5"
        ;;

    "get air wfbng adaptivelink")
        state_get air_wfbng_adaptivelink 0
        ;;
    "set air wfbng adaptivelink"*)
        bool_set air_wfbng_adaptivelink "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# AIR: Camera
# ════════════════════════════════════════════════════════════════════════════

    "get air camera mirror")
        state_get air_camera_mirror 0
        ;;
    "set air camera mirror"*)
        bool_set air_camera_mirror "$5"
        ;;

    "get air camera flip")
        state_get air_camera_flip 0
        ;;
    "set air camera flip"*)
        bool_set air_camera_flip "$5"
        ;;

    "get air camera contrast")
        state_get air_camera_contrast 50
        emit_values "0 100"
        ;;
    "set air camera contrast"*)
        state_set air_camera_contrast "$5"
        ;;

    "get air camera hue")
        state_get air_camera_hue 50
        emit_values "0 100"
        ;;
    "set air camera hue"*)
        state_set air_camera_hue "$5"
        ;;

    "get air camera saturation")
        state_get air_camera_saturation 50
        emit_values "0 100"
        ;;
    "set air camera saturation"*)
        state_set air_camera_saturation "$5"
        ;;

    "get air camera luminace")
        state_get air_camera_luminance 50
        emit_values "0 100"
        ;;
    "set air camera luminace"*)
        state_set air_camera_luminance "$5"
        ;;

    "get air camera size")
        state_get air_camera_size "1920x1080"
        emit_values "1280x720\n1456x816\n1920x1080\n1440x1080\n1920x1440\n2560x1440\n3840x2160"
        ;;
    "set air camera size"*)
        state_set air_camera_size "$5"
        ;;

    "get air camera video_mode")
        state_get air_camera_video_mode "16:9 1080p 30"
        emit_values "16:9 720p 30\n16:9 1080p 30\n16:9 1440p 30\n16:9 4k 2160p 30\n16:9 720p 60\n16:9 1080p 60\n4:3 1080p 30\n4:3 1440p 30"
        ;;
    "set air camera video_mode"*)
        state_set air_camera_video_mode "$5 $6 $7"
        ;;

    "get air camera fps")
        state_get air_camera_fps 30
        emit_values "30\n60\n90\n120"
        ;;
    "set air camera fps"*)
        state_set air_camera_fps "$5"
        ;;

    "get air camera bitrate")
        state_get air_camera_bitrate 8192
        emit_values "1024\n2048\n4096\n6144\n8192\n10240\n12288\n16384\n20480"
        ;;
    "set air camera bitrate"*)
        state_set air_camera_bitrate "$5"
        ;;

    "get air camera codec")
        state_get air_camera_codec "h265"
        emit_values "h264\nh265"
        ;;
    "set air camera codec"*)
        state_set air_camera_codec "$5"
        ;;

    "get air camera gopsize")
        state_get air_camera_gopsize 1
        emit_values "0 10"
        ;;
    "set air camera gopsize"*)
        state_set air_camera_gopsize "$5"
        ;;

    "get air camera rc_mode")
        state_get air_camera_rc_mode "cbr"
        emit_values "vbr\navbr\ncbr"
        ;;
    "set air camera rc_mode"*)
        state_set air_camera_rc_mode "$5"
        ;;

    "get air camera rec_enable")
        state_get air_camera_rec_enable 0
        ;;
    "set air camera rec_enable"*)
        bool_set air_camera_rec_enable "$5"
        ;;

    "get air camera rec_split")
        state_get air_camera_rec_split 10
        emit_values "0 60"
        ;;
    "set air camera rec_split"*)
        state_set air_camera_rec_split "$5"
        ;;

    "get air camera rec_maxusage")
        state_get air_camera_rec_maxusage 80
        emit_values "0 100"
        ;;
    "set air camera rec_maxusage"*)
        state_set air_camera_rec_maxusage "$5"
        ;;

    "get air camera exposure")
        state_get air_camera_exposure 20
        emit_values "5 50"
        ;;
    "set air camera exposure"*)
        state_set air_camera_exposure "$5"
        ;;

    "get air camera antiflicker")
        state_get air_camera_antiflicker "disabled"
        emit_values "disabled\n50\n60"
        ;;
    "set air camera antiflicker"*)
        state_set air_camera_antiflicker "$5"
        ;;

    "get air camera sensor_file")
        state_get air_camera_sensor "imx335_fpv"
        emit_values "imx307\nimx335\nimx335_fpv\nimx415_fpv\nimx415_milos10\nimx415_milos15"
        ;;
    "set air camera sensor_file"*)
        state_set air_camera_sensor "$5"
        ;;

    "get air camera fpv_enable")
        state_get air_camera_fpv 1
        ;;
    "set air camera fpv_enable"*)
        bool_set air_camera_fpv "$5"
        ;;

    "get air camera noiselevel")
        state_get air_camera_noiselevel 0
        emit_values "0 1"
        ;;
    "set air camera noiselevel"*)
        state_set air_camera_noiselevel "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# AIR: Telemetry
# ════════════════════════════════════════════════════════════════════════════

    "get air telemetry serial")
        state_get air_telemetry_serial "ttyS2"
        emit_values "ttyS0\nttyS1\nttyS2\nttyS3"
        ;;
    "set air telemetry serial"*)
        state_set air_telemetry_serial "$5"
        ;;

    "get air telemetry router")
        state_get air_telemetry_router "mavfwd"
        emit_values "mavfwd\nmsposd"
        ;;
    "set air telemetry router"*)
        state_set air_telemetry_router "$5"
        ;;

    "get air telemetry osd_fps")
        state_get air_telemetry_osd_fps 20
        emit_values "0 60"
        ;;
    "set air telemetry osd_fps"*)
        state_set air_telemetry_osd_fps "$5"
        ;;

    "get air telemetry gs_rendering")
        state_get air_telemetry_gs_rendering 0
        ;;
    "set air telemetry gs_rendering"*)
        bool_set air_telemetry_gs_rendering "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# AIR: Alink
# ════════════════════════════════════════════════════════════════════════════

    "get air alink power_level_0_to_4")
        state_get air_alink_power_level_0_to_4 2
        emit_values "0\n1\n2\n3\n4"
        ;;
    "get air alink fallback_ms")
        state_get air_alink_fallback_ms 200
        emit_values "1 2000"
        ;;
    "get air alink hold_fallback_mode_s")
        state_get air_alink_hold_fallback_mode_s 2
        emit_values "1 10"
        ;;
    "get air alink min_between_changes_ms")
        state_get air_alink_min_between_changes_ms 1000
        emit_values "1 10000"
        ;;
    "get air alink hold_modes_down_s")
        state_get air_alink_hold_modes_down_s 2
        emit_values "1 10"
        ;;
    "get air alink hysteresis_percent")
        state_get air_alink_hysteresis_percent 10
        emit_values "0 100"
        ;;
    "get air alink hysteresis_percent_down")
        state_get air_alink_hysteresis_percent_down 10
        emit_values "0 100"
        ;;
    "get air alink exp_smoothing_factor")
        state_get air_alink_exp_smoothing_factor 0.5
        emit_values "0 1.6"
        ;;
    "get air alink exp_smoothing_factor_down")
        state_get air_alink_exp_smoothing_factor_down 0.5
        emit_values "0 1.6"
        ;;
    "get air alink check_xtx_period_ms")
        state_get air_alink_check_xtx_period_ms 500
        emit_values "1 5000"
        ;;
    "get air alink request_keyframe_interval_ms")
        state_get air_alink_request_keyframe_interval_ms 500
        emit_values "1 5000"
        ;;
    "get air alink osd_level")
        state_get air_alink_osd_level 2
        emit_values "0\n1\n2\n3\n4\n5\n6"
        ;;
    "get air alink multiply_font_size_by")
        state_get air_alink_multiply_font_size_by 1.0
        emit_values "0 1.5"
        ;;
    "get air alink"*)
        state_get "air_alink_$4" 0
        ;;
    "set air alink"*)
        state_set "air_alink_$4" "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# AIR: Aalink
# ════════════════════════════════════════════════════════════════════════════

    "get air aalink SHOW_SIGNAL_BARS")
        state_get air_aalink_SHOW_SIGNAL_BARS 1
        ;;
    "set air aalink SHOW_SIGNAL_BARS"*)
        bool_set air_aalink_SHOW_SIGNAL_BARS "$5"
        ;;

    "get air aalink channel")
        state_get air_aalink_channel 157
        emit_values "36\n40\n44\n48\n52\n56\n60\n64\n100\n104\n108\n112\n116\n120\n124\n128\n132\n136\n140\n144\n149\n153\n157\n161\n165"
        ;;
    "set air aalink channel"*)
        state_set air_aalink_channel "$5"
        ;;

    "get air aalink SCALE_TX_POWER")
        state_get air_aalink_SCALE_TX_POWER 1.0
        emit_values "0.2 1.2"
        ;;
    "get air aalink THRESH_SHIFT")
        state_get air_aalink_THRESH_SHIFT 0
        emit_values "-50 50"
        ;;
    "get air aalink OSD_SCALE")
        state_get air_aalink_OSD_SCALE 1.0
        emit_values "0.2 2"
        ;;
    "get air aalink OSD_LEVEL")
        state_get air_aalink_OSD_LEVEL 1
        emit_values "0\n1\n2\n3"
        ;;
    "get air aalink THROUGHPUT_PCT")
        state_get air_aalink_THROUGHPUT_PCT 70
        emit_values "0 100"
        ;;
    "get air aalink HIGH_TEMP")
        state_get air_aalink_HIGH_TEMP 80
        emit_values "70 100"
        ;;
    "get air aalink MCS_SOURCE")
        state_get air_aalink_MCS_SOURCE "lowest"
        emit_values "lowest\ndownlink"
        ;;
    "get air aalink"*)
        state_get "air_aalink_$4" 0
        ;;
    "set air aalink"*)
        state_set "air_aalink_$4" "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# GS: WFB-NG
# ════════════════════════════════════════════════════════════════════════════

    "get gs wfbng gs_channel")
        state_get gs_wfbng_channel "157 (5 GHz)"
        emit_values_cmd list_wifi_channels
        ;;
    "set gs wfbng gs_channel"*)
        state_set gs_wfbng_channel "$5 $6 $7"
        ;;

    "get gs wfbng bandwidth")
        state_get gs_wfbng_bandwidth 20
        emit_values "20\n40"
        ;;
    "set gs wfbng bandwidth"*)
        state_set gs_wfbng_bandwidth "$5"
        ;;

    "get gs wfbng txpower")
        state_get gs_wfbng_txpower 50
        emit_values "1\n100"
        ;;
    "set gs wfbng txpower"*)
        state_set gs_wfbng_txpower "$5"
        ;;

    "get gs wfbng adaptivelink")
        state_get gs_wfbng_adaptivelink 0
        ;;
    "set gs wfbng adaptivelink"*)
        bool_set gs_wfbng_adaptivelink "$5"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# GS: System
# ════════════════════════════════════════════════════════════════════════════

    "get gs system rx_codec")
        echo "h265"
        emit_values "h265\nh264"
        ;;
    "set gs system rx_codec"*)
        : # noop
        ;;

    "get gs system rx_mode")
        state_get gs_system_rx_mode "wfb"
        emit_values "wfb\napfpv"
        ;;
    "set gs system rx_mode"*)
        state_set gs_system_rx_mode "$5"
        ;;

    "get gs system gs_rendering")
        state_get gs_system_gs_rendering 0
        ;;
    "set gs system gs_rendering"*)
        bool_set gs_system_gs_rendering "$5"
        ;;

    "get gs system connector")
        echo "HDMI"
        emit_values "HDMI"
        ;;
    "set gs system connector"*)
        : # noop
        ;;

    "get gs system resolution")
        state_get gs_system_resolution "1920x1080@60"
        printf '\x1e'
        printf '1280x720@60\n1920x1080@30\n1920x1080@60\n2560x1440@60\n3840x2160@30'
        ;;
    "set gs system resolution"*)
        state_set gs_system_resolution "$5"
        ;;

    "get gs system video_scale")
        state_get gs_system_video_scale 1.0
        emit_values "0.5 1.0"
        ;;
    "set gs system video_scale"*)
        state_set gs_system_video_scale "$5"
        ;;

    "get gs system rec_fps")
        state_get gs_system_rec_fps 60
        emit_values "30\n60\n90\n120"
        ;;
    "set gs system rec_fps"*)
        state_set gs_system_rec_fps "$5"
        ;;

    "get gs system dvr_mode")
        state_get gs_system_dvr_mode "raw"
        emit_values "raw\nreencode\nboth"
        ;;
    "set gs system dvr_mode"*)
        state_set gs_system_dvr_mode "$5"
        ;;

    "get gs system dvr_max_size")
        state_get gs_system_dvr_max_size 40
        emit_values "1 40"
        ;;
    "set gs system dvr_max_size"*)
        state_set gs_system_dvr_max_size "$5"
        ;;

    "get gs system dvr_reenc_codec")
        state_get gs_system_dvr_reenc_codec "h264"
        emit_values "h264\nh265"
        ;;
    "set gs system dvr_reenc_codec"*)
        state_set gs_system_dvr_reenc_codec "$5"
        ;;

    "get gs system dvr_reenc_resolution")
        state_get gs_system_dvr_reenc_resolution "1080p"
        emit_values "720p\n1080p"
        ;;
    "set gs system dvr_reenc_resolution"*)
        state_set gs_system_dvr_reenc_resolution "$5"
        ;;

    "get gs system dvr_reenc_fps")
        state_get gs_system_dvr_reenc_fps 60
        emit_values "30\n60"
        ;;
    "set gs system dvr_reenc_fps"*)
        state_set gs_system_dvr_reenc_fps "$5"
        ;;

    "get gs system dvr_reenc_bitrate")
        state_get gs_system_dvr_reenc_bitrate 10000
        emit_values "5000\n10000\n15000\n20000\n25000\n30000"
        ;;
    "set gs system dvr_reenc_bitrate"*)
        state_set gs_system_dvr_reenc_bitrate "$5"
        ;;

    "set gs system dvr_osd"* | "set gs system rec_enabled"*)
        : # noop
        ;;

# ════════════════════════════════════════════════════════════════════════════
# GS: APFPV  (nmcli not available on mac — return plausible stubs)
# ════════════════════════════════════════════════════════════════════════════

    "get gs apfpv ssid")
        state_get gs_apfpv_ssid "OpenIPC"
        ;;
    "set gs apfpv ssid"*)
        state_set gs_apfpv_ssid "$5"
        ;;

    "get gs apfpv password")
        state_get gs_apfpv_password "12345678"
        ;;
    "set gs apfpv password"*)
        state_set gs_apfpv_password "$5"
        ;;

    "get gs apfpv wlx"*)
        state_get "gs_apfpv_$4" 0
        ;;
    "set gs apfpv wlx"*)
        bool_set "gs_apfpv_$4" "$5"
        ;;

    "get gs apfpv status wlx"*)
        echo "Disconnected"
        ;;

    "set gs apfpv reset" | "set gs apfpv ssid"* | "set gs apfpv password"*)
        : # noop
        ;;

# ════════════════════════════════════════════════════════════════════════════
# GS: WiFi  (uses real macOS network info where possible)
# ════════════════════════════════════════════════════════════════════════════

    "get gs wifi hotspot")
        # macOS doesn't run nmcli; personal hotspot detection via system_profiler
        system_profiler SPNetworkDataType 2>/dev/null | grep -q "Personal Hotspot" && echo 1 || echo 0
        ;;
    "set gs wifi hotspot"*)
        echo "Hotspot control not available on macOS" >&2
        ;;

    "get gs wifi wlan")
        # 1 if en0 has an IP (WiFi connected)
        ifconfig en0 2>/dev/null | grep -q 'inet ' && echo 1 || echo 0
        ;;
    "set gs wifi wlan"*)
        : # noop — can't control WiFi from shell on macOS without networksetup sudo
        ;;

    "get gs wifi ssid")
        # Current WiFi SSID via airport
        /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I 2>/dev/null \
            | awk '/ SSID/{print $2}' || echo ""
        ;;

    "get gs wifi password")
        # Can retrieve from keychain if approved; return placeholder
        echo ""
        ;;

    "get gs wifi IP")
        local_ips
        ;;

    "get gs wifi networks")
        # Scan nearby WiFi networks via airport
        /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -s 2>/dev/null \
            | awk 'NR>1 {
                signal=$3; security=(NF>=5 ? "WPA" : "--")
                ssid=""
                for(i=1;i<=NF-3;i++) ssid=(ssid=="")?$i:ssid" "$i
                if(ssid!="") print ssid ":" security ":" signal
              }' || true
        ;;

    "get gs wifi savednetworks")
        # macOS stores WiFi credentials in Keychain — not easily scriptable without sudo
        echo ""
        ;;

    "set gs wifi connect"*)
        networksetup -setairportnetwork en0 "$5" "$6" 2>/dev/null || true
        ;;
    "set gs wifi disconnect"*)
        networksetup -setairportpower en0 off 2>/dev/null || true
        networksetup -setairportpower en0 on  2>/dev/null || true
        ;;

# ════════════════════════════════════════════════════════════════════════════
# GS: Main page (info labels) — real macOS values
# ════════════════════════════════════════════════════════════════════════════

    "get gs main Channel")
        # Show the channel of the current WiFi connection
        /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I 2>/dev/null \
            | awk '/channel/{print $2}' || echo "N/A"
        ;;

    "get gs main HDMI-OUT")
        # Report connected display resolutions
        system_profiler SPDisplaysDataType 2>/dev/null \
            | awk '/Resolution/{print $2 "x" $4 "@60"}' | head -1 || echo "N/A"
        ;;

    "get gs main Version")
        echo "macOS $(sw_vers -productVersion) (simulator)"
        ;;

    "get gs main Disk")
        read -r size avail pcent <<< $(df -h / | awk 'NR==2 {print $2, $4, $5}')
        printf "\n   Size: %s\n   Available: %s\n   Pct: %s" "$size" "$avail" "$pcent"
        ;;

    "get gs main WFB_NICS")
        # List wireless interfaces on this Mac
        networksetup -listallhardwareports 2>/dev/null \
            | awk '/Wi-Fi/{getline; print $2}' || echo "en0"
        ;;

# ════════════════════════════════════════════════════════════════════════════
# Buttons / Actions
# ════════════════════════════════════════════════════════════════════════════

    "button air actions Reboot")
        echo "[mac] Would reboot air unit (SSH not available in simulator)" >&2
        ;;
    "button gs actions Reboot")
        echo "[mac] Would reboot ground station" >&2
        ;;
    "search channel")
        echo "Not implemented"
        echo "Not implemented" >&2
        exit 1
        ;;

# ════════════════════════════════════════════════════════════════════════════
# Unknown
# ════════════════════════════════════════════════════════════════════════════

    *)
        echo "Unknown: $*" >&2
        exit 1
        ;;
esac

case $? in
    0) ;;
    1) exit 0 ;;
    *) exit $? ;;
esac
