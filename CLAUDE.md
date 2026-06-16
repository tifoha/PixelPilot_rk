# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

PixelPilot_rk is a WFB-ng (WiFiBroadcast-ng) video decoder client for Rockchip platforms (RK3566, RK3588s). It receives FPV drone video, decodes it via Rockchip MPP hardware, displays it via DRM/KMS with an OSD overlay using LVGL, and can re-encode to DVR. The codebase is C/C++17 with a GStreamer pipeline for video input.

## Build Commands

**Native build (on Rockchip device):**
```bash
git submodule update --init
cmake -B build
cmake --build build -j$(nproc) --target install
build/pixelpilot
```

**Debug build with sanitizers:**
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
```

**Build tests:**
```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
./build/pixelpilot_tests
```

**GSMenu simulator (local macOS/Linux dev):**
```bash
./sim.sh   # keys: w/a/s/d/Enter to navigate, t to toggle drone detection
```

**Cross-build for arm64 via QEMU:**
```bash
make qemu_build DEBIAN_CODENAME=bookworm           # binary
make qemu_build_deb DEBIAN_CODENAME=bookworm       # .deb package
make qemu_test DEBIAN_CODENAME=bookworm            # run tests
```

## Architecture

The application has two main execution paths controlled by CMake options:

**Production path** (`USE_SIMULATOR=OFF`, default):
- `src/main.cpp` — entry point; parses CLI args, sets up GStreamer pipeline, DRM display, OSD, DVR, input handling
- `src/gstrtpreceiver.cpp` — GStreamer pipeline receiving RTP/UDP video from wfb-ng
- `src/drm.c` — DRM/KMS display init and frame presentation
- `src/frame_processor.cpp` — Rockchip MPP hardware decode pipeline; feeds decoded frames to display and DVR encoder
- `src/frame_colorcorrect.cpp` — EGL/GLES2 shader pass for GPU color correction and OSD blending into frames
- `src/mpp_encoder.cpp` — Rockchip MPP hardware re-encoder for DVR recording with OSD overlay
- `src/dvr.cpp` — DVR state machine (start/stop/bitrate/FPS changes); uses `minimp4.h` for MP4 muxing
- `src/osd.cpp` / `src/osd_gl.cpp` — OSD rendering: parses `config_osd.json`, renders telemetry widgets; `osd_gl` handles EGL texture output
- `src/lvosd.c` / `src/lvosd.h` — LVGL-based OSD layer for on-screen display
- `src/wfbcli.cpp` — wfb-ng link stats client (UDP msgpack)
- `src/mavlink.c` — MAVLink telemetry parser
- `src/WiFiRSSIMonitor.cpp` — WiFi RSSI polling
- `src/os_mon.cpp` — CPU/memory/temperature OS monitor
- `src/input.cpp` — GPIO/keyboard input handling
- `src/menu.c` — OSD menu controller
- `src/gsmenu/` — Ground Station Menu subsystem (C files); controls air unit settings, DVR player, WFB-ng params, scripts, etc. via SSH/shell commands

**Simulator path** (`USE_SIMULATOR=ON`):
- `src/simulator.c` replaces the hardware-dependent code; SDL2-based rendering for local development of the GSMenu UI

## Key Configuration

- `lv_conf.h` — LVGL configuration (display driver, color depth, features)
- `config_osd.json` — OSD widget layout and data source bindings
- `pixelpilot.yaml` — GPIO pin assignments and runtime options
- `pixelpilot_config.h.in` — CMake-generated version header

## Testing

Tests use Catch2 and live in `tests/test_osd.cpp`. They test the OSD expression parser (`ExpressionTree` in `src/osd.hpp`). Enable with `-DBUILD_TESTS=ON`; Debug build flags (ASAN/UBSAN) are applied automatically. The `TEST` preprocessor macro guards test-only code paths.

## Target Platform

This software only runs fully on Rockchip Linux (RK3566/RK3588s) with `librockchip-mpp`, `librga`, EGL/GLES2, and DRM available. The GSMenu simulator (`sim.sh`) is the only component runnable on a developer workstation.
