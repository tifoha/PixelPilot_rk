# PixelPilot_rk
> [!IMPORTANT]
> Warning, this is an experimental project.
>
> Use this software at your own risk.

## Introduction

WFB-ng client (Video Decoder) for Rockchip platform powered by the [Rockchip MPP library](https://github.com/rockchip-linux/mpp).
It also displays a simple cairo based OSD that shows the bandwidth, decoding latency, and framerate of the decoded video, and wfb-ng link statistics.

Additional features:
- **GPU color correction** — live linear color transform applied to video and OSD via EGL/GLES2 (configurable gain/offset, e.g. for FPV color grading)
- **DVR re-encoding with OSD overlay** — records video via Rockchip MPP hardware encoder with the OSD blended in; supports live bitrate, FPS and codec changes
- **Frame pacer** — feeds the re-encoder at a steady target FPS, dropping excess frames or repeating the last frame as needed
- **Image flip** — upside-down display support via DRM
- **GSMenu** — on-screen ground station control menu for live air-unit and link settings

This project is based on a unique frozen development [FPVue_rk](https://github.com/gehee/FPVue_rk) by [Gee He](https://github.com/gehee).

Tested on RK3566 (Radxa Zero 3W) and RK3588s (Orange Pi 5).

## Installation

It can be installed as a binary DEB package or built from source.

### DEB package

Currently we have packages for Debian Bullseye (11) and Bookworm (12).
First you need to add a repository that contains `wfb-ng` dependency

```
curl -s https://apt.wfb-ng.org/public.asc | sudo gpg --dearmor --yes -o /usr/share/keyrings/wfb-ng.gpg
echo "deb [signed-by=/usr/share/keyrings/wfb-ng.gpg] https://apt.wfb-ng.org/ $(lsb_release -cs) master" | sudo tee /etc/apt/sources.list.d/wfb-ng.list
sudo apt update
```

Also make sure Radxa's repositories are enabled:

```
keyring="keyring.deb"
version="$(curl -L https://github.com/radxa-pkg/radxa-archive-keyring/releases/latest/download/VERSION)"
curl -L --output "$keyring" "https://github.com/radxa-pkg/radxa-archive-keyring/releases/download/${version}/radxa-archive-keyring_${version}_all.deb"
dpkg -i $keyring
```

Bookworm:
* https://radxa-repo.github.io/bookworm/
* https://radxa-repo.github.io/rk3566-bookworm

```
tee /etc/apt/sources.list.d/70-radxa.list <<< "deb [signed-by=/usr/share/keyrings/radxa-archive-keyring.gpg] https://radxa-repo.github.io/bookworm/ bookworm main"
tee /etc/apt/sources.list.d/80-radxa-rk3566.list <<< "deb [signed-by=/usr/share/keyrings/radxa-archive-keyring.gpg] https://radxa-repo.github.io/rk3566-bookworm rk3566-bookworm main"
```

Bullseye:
* https://radxa-repo.github.io/bullseye/ (bullseye and rockchip-bullseye)

```
tee /etc/apt/sources.list.d/70-radxa.list <<< "deb [signed-by=/usr/share/keyrings/radxa-archive-keyring.gpg] https://radxa-repo.github.io/bullseye/ bullseye main"
tee /etc/apt/sources.list.d/80-rockchip.list <<< "deb [signed-by=/usr/share/keyrings/radxa-archive-keyring.gpg] https://radxa-repo.github.io/bullseye rockchip-bullseye main"
```

Then download the latest `.deb` package from "releases" section on Github and install it:

```
sudo apt install ./bookworm_pixelpilot-rk_*_arm64.deb
```

Configuration files are:

* `/etc/pixelpilot/pixelpilot.yaml` - GPIO settings etc
* `/etc/pixelpilot/osd_config.json` - OSD configuration
* `/etc/default/pixelpilot` - systemd overrides / command line parameters

### Build from source

Build on the Rockchip linux system directly.

#### Install dependencies

- drm, cairo, mpp, logging, json, msgpack, gpiod, yaml-cpp

```
sudo apt install libdrm-dev libcairo-dev librockchip-mpp-dev libspdlog-dev nlohmann-json3-dev libmsgpack-dev libgpiod-dev libyaml-cpp-dev
```

- EGL/GLES2 and RGA (required for GPU color correction and DVR re-encoding)

```
sudo apt install libegl-dev libgles2-mesa-dev libgbm-dev librga-dev
```

- gstreamer

```
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev
```

#### Build Instructions

Build and run application in production environment:

```
git clone https://github.com/OpenIPC/PixelPilot_rk
cd PixelPilot_rk
git submodule update --init
cd ..
```

```
cmake -B build
sudo cmake --build build --target install
build/pixelpilot
```

Build and run application for debugging purposes:

```
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
build/pixelpilot --osd
```

Build and run gsmenu SDL simulator locally
```
./sim.sh
```
Simulator has w,a,s,d,Enter input.
Press t to toggle drone detection.

### Build from source for arm64

To build it on a non-ARM host machine, it is possible to build with QEMU emulator.

```
sudo apt-get install qemu-user-static
```
and then either build the binary (use `BUILD_TYPE=debug` to build debug version):

```
make qemu_build DEBIAN_CODENAME=bullseye
ls -l pixelpilot
```

or build .deb package:

```
make qemu_build_deb DEBIAN_CODENAME=bookworm
ls -l pixelpilot-rk_*.deb
```

## Usage

Show command line options:
```
pixelpilot --help
```

### OSD config

The OSD is configured declaratively in a JSON file (default `/etc/pixelpilot/config_osd.json`,
overridable with `--osd-config`). Widgets subscribe to *facts* (typed telemetry values from
MAVLink, WFB-ng stats, DVR state, OS metrics, etc.) and are redrawn at a fixed interval.

See **[docs/OSD.md](docs/OSD.md)** for the full reference: coordinate system, all available facts,
all widget types with examples, and a complete sample config.

## Color Correction

PixelPilot supports a real-time GPU-accelerated color transform applied to the live video feed.
The transform formula is:

```
output = clamp((input + offset) * gain, 0, 1)
```

Default values are `offset = -0.15` and `gain = 2.5`, which produce a high-contrast FPV look.
The transform is applied via an EGL/GLES2 shader directly on the DRM buffer (NV12), so there is no CPU copy overhead.

When color correction is active, the OSD overlay is also GPU-processed with the **inverse** transform so that OSD elements remain visually correct.

If DVR re-encoding is enabled alongside color correction, the recorded video uses the same color-corrected frames.

Parameters can be adjusted live through the GSMenu.

## GSMenu

The gsmenu provides a ui to modify air and ground settings.
Navigation is controlled via a GPIO buttons.
GPIO mapping can be configured in pixelpilot.yaml, see example.
PixelPilot_rk will take ownership of the needed gpios.
Settings on air and ground are get/set useing the gsmenu.sh script.

Install gsmenu.sh dependencies:
```
git clone https://github.com/OpenIPC/yaml-cli.git
cd yaml-cli
cmake -B build
sudo cmake --build build --target install
curl -L -o /usr/local/bin/yq https://github.com/mikefarah/yq/releases/download/v4.45.4/yq_linux_arm64
chmod +x /usr/local/bin/yq
sudo apt install drm-info jq netcat
```

### Navigation
Navigation mode:
  - Up/Down: Navigate
  - Right: Select page / Start edit mode
  - Left: Back/Close

Edit mode:
  - Up/down: Change value
  - Right: Confirm selection
  - Left: Back/Cancel

Keyboard mode:

  - Up/Down/Left/Right: Select key
  - Enter: Press selected key

When not in menu Up/Down will open the menu.

## Known issues

1. Video is cropped when the fpv feed resolution is bigger than the screen mode.
1. Crashes when video feed resolution is higher than the screen resolution.

## The way it works

It uses `gstreamer` to read the RTP media stream from video UDP.
It uses `mpp` library to decode MPEG frames using Rockchip hardware decoder.
It uses [Direct Rendering Manager (DRM)](https://en.wikipedia.org/wiki/Direct_Rendering_Manager) to
display video on the screen, see `drm.c`.
It uses `mavlink` decoder to read Mavlink telemetry from telemetry UDP (if enabled), see `mavlink.c`
It uses `cairo` library to draw OSD elements (if enabled), see `osd.c`.
It uses `lvgl` to draw the gsmenu.
It writes non-decoded MPEG stream to file as DVR (if enabled) using `minimp4.h` library.
It uses EGL/GLES2 (via `frame_colorcorrect.cpp`) to apply a GPU color transform to decoded frames before display and recording.
It uses Rockchip MPP hardware encoder (`mpp_encoder.cpp`) to optionally re-encode the color-corrected video with OSD blended in for DVR recording.
The `EncoderPacer` (`encoder_pacer.cpp`) decouples the decoder from the re-encoder, ensuring the encoder receives frames at a steady target FPS.

Pixelpilot starts several threads:

* main thread:
  controls gstreamer which reads RTP, extracts MPEG frames and
  - feeds them to MPP hardware decoder
  - sends them to DVR thread via mutex-protected `std::queue` (if enabled)
* DVR_THREAD (if enabled):
  reads video frames and start/stop/shutdown commands from main thread via `std::queue` and writes
  frames them to disk using `minimp4` library.
  It yields on a condition variable for DVR queue
* FRAME_THREAD:
  reads decoded video frames from MPP hardware decoder and forwards them to `DISPLAY_THREAD`
  through DRM `output_list` protected by `video_mutex`.
  Seems that thread vields on `mpi->decode_get_frame()` call waiting for HW decoder to return a new frame
* DISPLAY_THREAD:
  reads decoded frames and OSD from `video_mutex`-protected `output_list` and calls `drm*` functions to
  render them on the screen.
  The loop yields on `video_mutex` and `video_cond` waiting for a new frame to
  display from FRAME_THREAD
* MAVLINK_THREAD (if OSD and mavlink configured):
  reads mavlink packets from UDP, decodes and updates `osd_vars` (without any mutex).
  The loop yields on UDP read.
* WFBCLI_THREAD (if OSD is enabled):
  connects to the local WFB instance stats API, reads JSON stats messages and publishes OSD facts.
  The loop yields on TCP read.
* OSD_THREAD (if OSD is enabled):
  takes `drm_fd`, `output_list` and JSON config as thread parameters,
  receives Facts through mutex-with-timeout-protected `std::queue`, feeds Facts to widgets and
  periodically draws widgets on a buffer inside `output_list` using Cairo library.
  There exists legacy OSD, is based on `osd_vars`, draws using Cairo library, to be removed.
  The loop yields on queue's mutex with timeout (timeout in order to re-draw OSD at fixed intervals).
* ENCODER_PACER_THREAD (if DVR re-encoding is enabled):
  wakes at the target FPS interval and submits the most recent decoded frame to the MPP re-encoder.
  Drops frames when the source is faster than the target FPS; repeats the last frame when slower.
* MPP_ENCODER_THREAD (if DVR re-encoding is enabled):
  receives frames via an RPC queue from the pacer and encodes them with the Rockchip MPP hardware
  encoder. Handles live bitrate, FPS, and codec changes without full re-initialisation where possible.

## Release

* update project version in `CMakeList.txt`: `project(pixelpilot, VERSION <X.Y.Z>)`, in
  `Makefile`: `DEB_VERSION`, and update `debian/changelog` (with `dch -v <X.Y.Z>`). Commit
* push that commit to master (either directly or with PR)
* tag the tip of the master branch with the same `<X.Y.Z>` version
* run `git push --tags`; it will trigger a build job that would publish a new GitHub release
