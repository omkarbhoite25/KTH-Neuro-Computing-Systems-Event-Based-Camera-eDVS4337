<div align="center">

# :zap: eDVS4337 Event-Based Camera

### Secure ROS Driver & Rust Processing Pipeline

[![CMake Build & Test](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/cmake.yml/badge.svg)](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/cmake.yml)
[![CodeQL](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/codeql.yml/badge.svg)](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/codeql.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg)](https://www.rust-lang.org/)
[![SLSA 3](https://slsa.dev/images/gh-badge-level3.svg)](https://slsa.dev)

A modular, security-hardened ROS driver for the **eDVS4337 neuromorphic camera**, paired with a high-performance **Rust signal-processing library** for real-time event denoising and filtering.

<img src="https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/blob/main/images/eDVS.png" width="720">

*eDVS4337 sensor mounted on the KTH robotic head platform*

</div>

---

**Project by:** Omkar Vilas Bhoite
**Advised by:** Prof. Jorg Conradt & Juan Pablo Romero Bermudez, KTH Royal Institute of Technology

---

## :book: Table of Contents

- [:eye: What Is an Event Camera?](#eye-what-is-an-event-camera)
- [:building_construction: Architecture](#building_construction-architecture)
- [:package: Prerequisites](#package-prerequisites)
- [:hammer_and_wrench: Building](#hammer_and_wrench-building)
- [:rocket: Running](#rocket-running)
- [:mag: Viewing Events](#mag-viewing-events)
- [:gear: Configuration](#gear-configuration)
- [:crab: Rust Processing Library](#crab-rust-processing-library)
- [:computer: Standalone CLI Tool](#computer-standalone-cli-tool)
- [:satellite: ROS Interface](#satellite-ros-interface)
- [:shield: Security](#shield-security)
- [:factory: CI/CD Pipeline](#factory-cicd-pipeline)
- [:wrench: Troubleshooting](#wrench-troubleshooting)
- [:page_facing_up: License](#page_facing_up-license)

---

## :eye: What Is an Event Camera?

Unlike a conventional frame-based camera that captures entire images at a fixed rate (e.g., 30 fps), the eDVS4337 is a **neuromorphic vision sensor**. Each pixel operates independently and asynchronously, reporting brightness changes the instant they occur.

| Property | Frame Camera | Event Camera (eDVS4337) |
|:---------|:-------------|:------------------------|
| **Output** | Full image at fixed intervals | Sparse stream of change events |
| **Temporal resolution** | ~33 ms (30 fps) | ~1 us (microsecond) |
| **Redundancy** | High (unchanged pixels re-transmitted) | None (only changes reported) |
| **Dynamic range** | ~60 dB | >120 dB |
| **Motion blur** | Present at high speeds | Eliminated |
| **Data rate** | Constant (resolution x framerate) | Proportional to scene activity |

Each event is a tuple of four values:

```
(x, y, timestamp, polarity)
```

- **x, y** : pixel coordinate on the 128 x 128 sensor array
- **timestamp** : time of the brightness change in microseconds
- **polarity** : direction of the change (+1 for brighter, -1 for darker)

This makes event cameras ideal for **high-speed robotics**, **low-latency tracking**, **autonomous navigation**, and **neuromorphic computing** research.

---

## :building_construction: Architecture

The project is a **C++ and Rust hybrid** with two tightly integrated components:

```
                    +-----------------+
                    |   ROS Master    |
                    +--------+--------+
                             |
              +--------------+--------------+
              |                             |
    +---------v----------+       +----------v---------+
    | /edvs/control      |       | /edvs/events       |
    | (Control.msg)      |       | (EventArray.msg)   |
    | Subscriber         |       | Publisher           |
    +--------------------+       +--------------------+
              |                             ^
              v                             |
    +---------------------------------------------------+
    |              EdvsDriver (C++ ROS Node)             |
    |                                                   |
    |  +-------------+    +---------------------------+ |
    |  | libcaer     |    | Rust FFI Filters          | |
    |  | Serial I/O  |--->| TemporalFilter            | |
    |  | /dev/ttyUSB0|    | HotPixelFilter            | |
    |  +-------------+    +---------------------------+ |
    +---------------------------------------------------+
              |
    +---------v----------+
    |   eDVS4337 Camera  |
    |   (128 x 128 DVS)  |
    +--------------------+
```

### :file_folder: Project Structure

```
camera/src/event_based_camera/
  |
  +-- include/event_based_camera/
  |     edvs_driver.hpp ......... Driver class (RAII, threaded readout)
  |     event_types.hpp ......... Shared event struct (C/Rust FFI layout)
  |     security.hpp ............ Input validation and rate limiting
  |
  +-- src/
  |     edvs_driver_node.cpp .... Thin ROS entry point (main)
  |     edvs_driver.cpp ......... Driver implementation
  |     security.cpp ............ Security utilities
  |
  +-- msg/
  |     Control.msg ............. Start/stop command (int32 sos)
  |     Event.msg ............... Single polarity event
  |     EventArray.msg .......... Batched events with std_msgs/Header
  |
  +-- config/
  |     edvs_params.yaml ........ Default runtime parameters
  |
  +-- launch/
  |     edvs_camera.launch ...... Single-command launch file
  |
  +-- rust/edvs_processing/ .... Rust crate (event filters + CLI)
        Cargo.toml
        src/
          lib.rs ................ Crate root
          event.rs .............. repr(C) Event type
          denoise.rs ............ Temporal nearest-neighbor filter
          hot_pixel.rs .......... Hot pixel detector
          accumulator.rs ........ Event-to-frame accumulator
          ffi.rs ................ C-compatible FFI exports
          bin/edvs_process.rs ... Standalone offline CLI tool
```

### :link: How the Pieces Connect

| Layer | Language | Role |
|:------|:---------|:-----|
| **Driver node** | C++ | Opens the eDVS via libcaer, reads raw events over serial, publishes to ROS |
| **Security module** | C++ | Validates device paths, rate-limits commands, checks permissions |
| **Processing filters** | Rust | Denoises events, detects hot pixels, accumulates frames (linked via FFI) |
| **CLI tool** | Rust | Offline filtering of recorded event files (zero ROS dependency) |
| **ROS messages** | .msg | `Control`, `Event`, `EventArray` definitions for topic communication |

---

## :package: Prerequisites

> :bulb: **Tested on:** Ubuntu 20.04 with ROS Noetic and ROS Melodic.

### 1. ROS (Noetic or Melodic)

Follow the official installation guide:
:arrow_right: https://wiki.ros.org/ROS/Installation

### 2. Rust Toolchain

```bash
curl https://sh.rustup.rs -sSf | sh
source $HOME/.cargo/env
```

Verify with:

```bash
rustc --version
cargo --version
```

### 3. System Dependencies

```bash
sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev libserialport-dev
```

### 4. libcaer (eDVS Camera Library)

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install libcaer-dev
```

### 5. Serial Port Access

The eDVS communicates over a serial port (typically `/dev/ttyUSB0` or `/dev/ttyACM0`). Your user account must have permission to access it.

<details>
<summary><b>:key: Serial Port Setup Instructions</b> (click to expand :point_down:)</summary>

**Step 1.** Check which port the camera is connected to:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

**Step 2.** Verify that your user is in the `dialout` group:

```bash
id $USER
```

If `dialout` does not appear in the output, add yourself:

```bash
sudo usermod -a -G dialout $USER
```

Log out and back in for the change to take effect.

**Step 3.** If you still cannot open the port, grant read/write access manually:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Replace `ttyUSB0` with your actual device name.

</details>

---

## :hammer_and_wrench: Building

### Step 1: Build the Rust Processing Library

```bash
cd camera/src/event_based_camera/rust/edvs_processing
cargo build --release
```

### Step 2: Build the ROS Package

```bash
cd ../../../../../camera
catkin_make
source devel/setup.bash
```

> :bulb: **Tip:** To avoid re-sourcing every time you open a new terminal, add this line to your `~/.bashrc`:
>
> ```bash
> source ~/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/camera/devel/setup.bash
> ```

### Quick Verification

```bash
# Should print the path to the edvs_camera binary
which edvs_camera

# Should list the package
rospack find event_based_camera
```

---

## :rocket: Running

### Option A: Launch File (Recommended)

The simplest way to start the camera node. This loads all parameters from the config file automatically.

```bash
# Terminal 1 : Start the ROS master
roscore

# Terminal 2 : Launch the camera node
roslaunch event_based_camera edvs_camera.launch
```

Then send a start command from a third terminal:

```bash
# Terminal 3 : Begin capturing events
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 1"
```

### Option B: Manual Node Launch

If you prefer to override parameters directly on the command line:

```bash
# Terminal 1
roscore

# Terminal 2 : Run with a custom serial port
rosrun event_based_camera edvs_camera _serial_port:=/dev/ttyACM0

# Terminal 3 : Start the camera
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 1"
```

### :stop_sign: Stopping the Camera

**Option 1:** Send a stop command:

```bash
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 0"
```

**Option 2:** Press `Ctrl+C` in the node terminal. The driver shuts down gracefully: it stops the data stream, joins the readout thread, and releases the serial port via RAII.

---

## :mag: Viewing Events

Once the camera is running, events are published on the `/edvs/events` topic as `EventArray` messages.

### Echo Live Events

```bash
rostopic echo /edvs/events
```

### Check the Publishing Rate

```bash
rostopic hz /edvs/events
```

### Inspect a Single Message

```bash
rostopic echo -n 1 /edvs/events
```

<details>
<summary><b>:page_facing_up: Example EventArray Output</b> (click to expand :point_down:)</summary>

```yaml
header:
  seq: 42
  stamp:
    secs: 1709312456
    nsecs: 789012345
  frame_id: "edvs"
width: 128
height: 128
events:
  -
    x: 64
    y: 32
    timestamp: 1709312456789012
    polarity: 1
  -
    x: 65
    y: 32
    timestamp: 1709312456789045
    polarity: -1
  ...
```

Each event has fields `x`, `y`, `timestamp` (microseconds), and `polarity` (+1 for brightness increase, -1 for decrease).

</details>

---

## :gear: Configuration

All parameters are defined in `config/edvs_params.yaml` and can be overridden at launch time.

| Parameter | Type | Default | Description |
|:----------|:-----|:--------|:------------|
| `serial_port` | `string` | `/dev/ttyUSB0` | Path to the eDVS serial device |
| `enable_denoising` | `bool` | `true` | Enable temporal nearest-neighbor denoising |
| `denoise_threshold_us` | `int` | `5000` | Time window (us) for spatial neighbor correlation |
| `enable_hot_pixel_filter` | `bool` | `true` | Enable hot pixel detection and rejection |
| `hot_pixel_window_us` | `int` | `1000000` | Observation window (us) for rate tracking |
| `hot_pixel_max_rate` | `int` | `1000` | Max events per pixel per window before flagging |

### Overriding at Launch

```bash
roslaunch event_based_camera edvs_camera.launch serial_port:=/dev/ttyACM0
```

### Overriding with rosrun

```bash
rosrun event_based_camera edvs_camera \
  _serial_port:=/dev/ttyACM0 \
  _enable_denoising:=false \
  _hot_pixel_max_rate:=500
```

---

## :crab: Rust Processing Library

The `edvs_processing` crate provides three memory-safe event processing filters, compiled as a static library and linked into the C++ driver via FFI. Each filter is independently unit-tested.

### :one: Temporal Denoising (`denoise.rs`)

Rejects isolated noise events using a **nearest-neighbor spatio-temporal correlation** test.

- For each incoming event, the filter checks whether any of its **8 spatial neighbors** (3x3 window) fired within `threshold_us` microseconds.
- Events with no recent neighbor are classified as **uncorrelated noise** and discarded.
- Events with at least one recent neighbor are passed through as **valid signal**.

This preserves real edges and motion while removing random background activity.

### :two: Hot Pixel Filter (`hot_pixel.rs`)

Detects and suppresses **stuck or noisy pixels** that fire at abnormally high rates.

- Tracks per-pixel event counts in a **sliding time window** of configurable length.
- At the end of each window, any pixel whose count exceeds `max_rate` is flagged as **hot**.
- Events from flagged pixels are rejected in the **following window**.
- Flags are re-evaluated every window, so a pixel can recover if its rate drops.

### :three: Event Accumulator (`accumulator.rs`)

Converts a stream of polarity events into a **grayscale image frame**.

- Each pixel starts at a neutral value of **128**.
- ON events (+1) increment the pixel value; OFF events (-1) decrement it.
- Values are clamped to the range **0 to 255**.
- The frame can be reset to neutral at any time.

This is useful for visualization, integration with traditional CV pipelines, and debugging.

### :test_tube: Running the Tests

```bash
cd camera/src/event_based_camera/rust/edvs_processing
cargo test
```

```
running 17 tests
test accumulator::tests::test_clamp_lower ... ok
test accumulator::tests::test_clamp_upper ... ok
test accumulator::tests::test_initial_frame_neutral ... ok
test accumulator::tests::test_off_event_decrements ... ok
test accumulator::tests::test_on_event_increments ... ok
test accumulator::tests::test_out_of_bounds_ignored ... ok
test accumulator::tests::test_reset ... ok
test denoise::tests::test_corner_event ... ok
test denoise::tests::test_distant_neighbor_rejected ... ok
test denoise::tests::test_isolated_event_rejected ... ok
test denoise::tests::test_neighbor_event_accepted ... ok
test denoise::tests::test_out_of_bounds_rejected ... ok
test event::tests::test_event_display ... ok
test event::tests::test_event_size ... ok
test hot_pixel::tests::test_hot_pixel_count ... ok
test hot_pixel::tests::test_hot_pixel_detected ... ok
test hot_pixel::tests::test_normal_pixel_passes ... ok

test result: ok. 17 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

Tests cover boundary pixels, temporal threshold edge cases, saturation clamping, out-of-bounds rejection, and hot pixel recovery.

---

## :computer: Standalone CLI Tool

The `edvs-process` binary reads tab-separated events from standard input, applies the configurable filter pipeline, and writes passing events to standard output. It has **zero ROS dependency** and can run anywhere Rust compiles.

### Input Format

One event per line, tab-separated:

```
x	y	timestamp	polarity
```

Lines starting with `#` are treated as comments and skipped.

### Examples

**Filter a recorded event file:**

```bash
cat recorded_events.tsv | edvs-process > filtered_events.tsv
```

**Disable denoising, keep only hot pixel filtering:**

```bash
cat events.tsv | edvs-process --denoise-us 0 > filtered.tsv
```

**Use custom sensor dimensions and a tighter denoise window:**

```bash
cat events.tsv | edvs-process --width 64 --height 64 --denoise-us 3000 > out.tsv
```

**Pipe directly from the old stdout-based driver:**

```bash
rosrun event_based_camera edvs_camera | edvs-process > live_filtered.tsv
```

### Full Usage Reference

```
edvs-process [OPTIONS]

Options:
  --width N            Sensor width in pixels (default: 128)
  --height N           Sensor height in pixels (default: 128)
  --denoise-us N       Temporal denoise threshold in microseconds
                       (default: 5000, set to 0 to disable)
  --hot-pixel-rate N   Maximum events per pixel per window
                       (default: 1000, set to 0 to disable)
  --hot-pixel-window N Hot pixel observation window in microseconds
                       (default: 1000000)
  --help               Show this help message
```

The tool prints a summary to standard error when it finishes:

```
edvs-process: 84210/100000 events passed filters (84.2%)
```

---

## :satellite: ROS Interface

### Topics

| Topic | Message Type | Direction | Description |
|:------|:-------------|:----------|:------------|
| `/edvs/control` | `event_based_camera/Control` | :inbox_tray: Subscriber | Send `sos: 1` to start, `sos: 0` to stop |
| `/edvs/events` | `event_based_camera/EventArray` | :outbox_tray: Publisher | Batched filtered events with header |

### Message Definitions

<details>
<summary><b>:page_facing_up: Control.msg</b> (click to expand :point_down:)</summary>

```
int32 sos    # 1 = start capture, 0 = stop capture
```

</details>

<details>
<summary><b>:page_facing_up: Event.msg</b> (click to expand :point_down:)</summary>

```
uint16 x          # Pixel column (0 to width-1)
uint16 y          # Pixel row (0 to height-1)
int64  timestamp  # Event time in microseconds
int8   polarity   # +1 (ON / brighter) or -1 (OFF / darker)
```

</details>

<details>
<summary><b>:page_facing_up: EventArray.msg</b> (click to expand :point_down:)</summary>

```
std_msgs/Header              header
uint32                       width      # Sensor width (pixels)
uint32                       height     # Sensor height (pixels)
event_based_camera/Event[]   events     # Batch of filtered events
```

</details>

---

## :shield: Security

The driver implements multiple layers of hardening to address vulnerabilities found in the original prototype.

| Category | Measure | Details |
|:---------|:--------|:--------|
| :mag: **Input validation** | Device path validation | Path must exist under `/dev/`, must be a character device (`S_ISCHR`), must not contain `..` traversal sequences |
| :mag: **Input validation** | Control message validation | Only `sos: 0` and `sos: 1` are accepted; all other values are rejected with a warning |
| :lock: **Access control** | Permission check | Read/write access to the serial device is verified before opening; clear error messages guide the user to fix permissions |
| :hourglass_flowing_sand: **Rate limiting** | Command throttling | Control commands are throttled to one per second, preventing accidental or malicious flooding |
| :broom: **Resource safety** | RAII lifecycle | Camera device wrapped in `std::unique_ptr`; destructor calls `dataStop()` and closes the serial port automatically |
| :thread: **Concurrency** | Threaded readout | Event reading runs in a dedicated thread; the ROS callback queue is never blocked |
| :no_entry_sign: **Memory safety** | Rust processing | All event filtering logic is written in Rust, eliminating buffer overflows, use-after-free, and data races by construction |
| :robot: **Static analysis** | CodeQL CI | Automated C++ vulnerability scanning on every push and on a weekly schedule |
| :package: **Supply chain** | SLSA Level 3 | Release builds include cryptographic provenance attestations |

### Vulnerabilities Fixed from Original Codebase

| ID | Issue | Severity | Resolution |
|:---|:------|:---------|:-----------|
| S1 | Hardcoded device path `/dev/ttyUSB0` | HIGH | Parameterized via ROS param with validation |
| S5/S6 | Memory leaks and double-free in `tensor.hpp` | CRITICAL | File deleted; Rust handles data processing |
| S7 | Data races in `buf.hpp` query methods | HIGH | File deleted; no circular buffers needed |
| S9 | `int64_t` timestamp truncated to `int` | MEDIUM | `int64_t` used throughout the pipeline |
| S10 | Infinite loop in ROS callback (DoS) | HIGH | Threaded readout with `std::atomic<bool>` |
| S13 | Serial port left open on shutdown | MEDIUM | RAII destructor handles cleanup |

---

## :factory: CI/CD Pipeline

Three GitHub Actions workflows run automatically on every push and pull request.

| Workflow | Trigger | Purpose |
|:---------|:--------|:--------|
| **CMake Build & Test** | Push, PR | Builds the Rust crate (`cargo build`, `cargo test`), then builds the full ROS package (`catkin_make`) on Ubuntu 20.04 |
| **CodeQL Security Analysis** | Push, PR, weekly | Static analysis of all C++ code for buffer overflows, format string bugs, null dereferences, and injection vulnerabilities |
| **SLSA Provenance** | Release created | Generates SLSA Level 3 supply-chain attestations with SHA256 hashes of build artifacts |

All CI jobs run on `ubuntu-20.04` with ROS Noetic, matching the production deployment target.

---

## :wrench: Troubleshooting

<details>
<summary><b>:x: "Serial port validation failed"</b> (click to expand :point_down:)</summary>

The device path is incorrect or the device is not connected.

1. Check which port the camera is on: `ls /dev/ttyUSB* /dev/ttyACM*`
2. Update the `serial_port` parameter to match:
   ```bash
   roslaunch event_based_camera edvs_camera.launch serial_port:=/dev/ttyACM0
   ```

</details>

<details>
<summary><b>:x: "Serial port permission check failed"</b> (click to expand :point_down:)</summary>

Your user account does not have read/write access to the serial device.

1. Add yourself to the `dialout` group: `sudo usermod -a -G dialout $USER`
2. Log out and back in.
3. Or grant access manually: `sudo chmod a+rw /dev/ttyUSB0`

</details>

<details>
<summary><b>:x: "Command not found" when running rosrun or roslaunch</b> (click to expand :point_down:)</summary>

The ROS workspace has not been sourced in this terminal.

```bash
source ~/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/camera/devel/setup.bash
```

Add it to `~/.bashrc` to make it permanent.

</details>

<details>
<summary><b>:warning: Events are published but most are filtered out</b> (click to expand :point_down:)</summary>

The denoising threshold may be too strict for your scene. Try one of the following:

- Increase the threshold: `_denoise_threshold_us:=10000`
- Disable denoising entirely: `_enable_denoising:=false`
- Disable the hot pixel filter: `_enable_hot_pixel_filter:=false`

</details>

<details>
<summary><b>:warning: Camera does not respond after sending sos: 1</b> (click to expand :point_down:)</summary>

The driver rate-limits control commands to one per second. Wait at least one second between commands.

If the camera still does not respond, check that the device is plugged in and that the serial port path is correct.

</details>

<details>
<summary><b>:warning: catkin_make fails with "Could not find library: caer"</b> (click to expand :point_down:)</summary>

libcaer is not installed. Install it from the iniVation PPA:

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install libcaer-dev
```

</details>

---

## :page_facing_up: License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

<div align="center">

**Built with** :heart: **at KTH Royal Institute of Technology**

*Neuromorphic Computing Systems Group*

</div>
