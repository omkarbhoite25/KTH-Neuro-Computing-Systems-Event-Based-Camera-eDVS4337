<div align="center">

# :zap: eDVS4337 Event-Based Camera

### Production-Grade ROS Driver & Rust Processing Pipeline

[![CMake Build & Test](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/cmake.yml/badge.svg)](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/cmake.yml)
[![CodeQL](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/codeql.yml/badge.svg)](https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/actions/workflows/codeql.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Rust](https://img.shields.io/badge/Rust-2021_Edition-orange.svg)](https://www.rust-lang.org/)
[![SLSA 3](https://slsa.dev/images/gh-badge-level3.svg)](https://slsa.dev)
[![Tests](https://img.shields.io/badge/Tests-30%20passing-brightgreen.svg)](#test_tube-running-the-tests)

A modular, security-hardened ROS driver for the **eDVS4337 neuromorphic camera**, paired with a high-performance **Rust signal-processing library** for real-time event denoising and filtering.

<img src="https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/blob/main/images/eDVS.png" width="720">

*eDVS4337 sensor mounted on the KTH robotic head platform*

</div>

---

**Project by:** Omkar Vilas Bhoite
**Advised by:** Prof. Jorg Conradt & Juan Pablo Romero Bermudez, KTH Royal Institute of Technology

---

## :sparkles: Key Features

| | Feature | Description |
|:--|:--------|:------------|
| :crab: | **Rust-Powered Filters** | Temporal denoising, hot pixel rejection, and frame accumulation — memory-safe by construction |
| :shield: | **Security-Hardened** | 11 vulnerabilities fixed, mutex-guarded concurrency, monotonic rate limiting, `panic::catch_unwind()` at FFI boundaries |
| :gear: | **Modular Architecture** | Abstract `EvtCamera` interface, composable `FilterPipeline`, cleanly separated concerns |
| :test_tube: | **30 Tests** | 17 unit tests + 13 integration tests covering FFI null safety, pipeline correctness, and edge cases |
| :robot: | **CI/CD** | Automated build, CodeQL security analysis, and SLSA Level 3 provenance on every push |
| :computer: | **Standalone CLI** | Offline event filtering with zero ROS dependency — runs anywhere Rust compiles |

---

## :fast_forward: Quick Start

```bash
# 1. Build Rust processing library
cd camera/src/event_based_camera/rust/edvs_processing
cargo build --release

# 2. Build ROS package
cd ../../../../../camera
catkin_make && source devel/setup.bash

# 3. Launch (3 terminals)
roscore                                                              # Terminal 1
roslaunch event_based_camera edvs_camera.launch                      # Terminal 2
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 1"    # Terminal 3

# 4. View live events
rostopic echo /edvs/events
```

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
| **Temporal resolution** | ~33 ms (30 fps) | 15 us per event [(Lichtsteiner et al. 2008)](https://ieeexplore.ieee.org/document/4444573/) |
| **Redundancy** | High (unchanged pixels re-transmitted) | None (only changes reported) |
| **Dynamic range** | ~60 dB (typical CMOS) | 120 dB [(DVS128 datasheet)](https://docs.inivation.com/_static/hardware_guides/dvs128.pdf) |
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

The project is a **C++ and Rust hybrid** with a layered, modular architecture. Each layer has a single responsibility and communicates through well-defined interfaces.

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
    |  +---------------------+   +--------------------+ |
    |  | EvtCamera interface |   | FilterPipeline     | |
    |  |   (abstraction)     |   |  (RAII C++ class)  | |
    |  |         |           |   |         |          | |
    |  |  +------v--------+  |   |  +------v-------+  | |
    |  |  | LibcaerEdvs   |  |   |  | Rust FFI     |  | |
    |  |  | (serial I/O)  |  |-->|  | Temporal     |  | |
    |  |  +---------------+  |   |  | HotPixel     |  | |
    |  +---------------------+   |  +--------------+  | |
    |                            +--------------------+ |
    |  mutex-guarded device access | atomic stop flag   |
    +---------------------------------------------------+
              |
    +---------v----------+
    |   eDVS4337 Camera  |
    |   (128 x 128 DVS)  |
    +--------------------+
```

### :jigsaw: Design Principles

- **Abstraction**: The `EvtCamera` interface decouples the driver from any specific camera SDK — swap libcaer for Metavision or a mock without touching driver logic
- **Composition**: `FilterPipeline` owns Rust filter lifecycles via RAII `RustFilterPtr<T>` wrappers — no manual `destroy()` calls
- **Thread safety**: `device_mutex_` guards all shared state; `std::atomic<bool>` enables lock-free stop signaling
- **FFI safety**: `panic::catch_unwind()` at every Rust FFI boundary; null checks on all pointer parameters

### :file_folder: Project Structure

```
camera/src/event_based_camera/
  |
  +-- include/event_based_camera/
  |     edvs_driver.hpp ......... Driver class (RAII, threaded readout)
  |     evt_camera.hpp .......... Abstract camera interface (EvtCamera)
  |     libcaer_edvs.hpp ........ libcaer eDVS implementation
  |     filter_pipeline.hpp ..... FilterPipeline class (Rust FFI RAII wrappers)
  |     event_types.hpp ......... Shared event struct (C/Rust FFI layout)
  |     security.hpp ............ Input validation and rate limiting
  |
  +-- src/
  |     edvs_driver_node.cpp .... Thin ROS entry point (main)
  |     edvs_driver.cpp ......... Driver implementation
  |     filter_pipeline.cpp ..... FilterPipeline init/process/reset
  |     libcaer_edvs.cpp ........ libcaer device open/start/stop/read
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
          lib.rs ................ Crate root (MAX_SENSOR_DIM constant)
          event.rs .............. repr(C) Event type
          denoise.rs ............ Temporal nearest-neighbor filter
          hot_pixel.rs .......... Hot pixel detector
          accumulator.rs ........ Event-to-frame accumulator
          ffi.rs ................ C-compatible FFI exports (panic-safe)
          bin/edvs_process.rs ... Standalone offline CLI tool
        tests/
          integration_test.rs ... FFI safety and pipeline integration tests
```

### :link: Component Responsibilities

| Component | Language | Responsibility |
|:----------|:---------|:---------------|
| **EdvsDriver** | C++ | Orchestrates camera lifecycle, ROS pub/sub, threaded readout loop |
| **EvtCamera** | C++ | Pure virtual interface — enables SDK-agnostic device access and testing |
| **LibcaerEdvs** | C++ | libcaer-based eDVS implementation (serial open, config, packet parsing) |
| **FilterPipeline** | C++ | RAII wrapper owning Rust FFI filter pointers; composable `process()` method |
| **Security** | C++ | Device path validation, permission checks, monotonic rate limiting (`WallTime`) |
| **edvs_processing** | Rust | Temporal denoise, hot pixel detection, frame accumulation (linked via FFI) |
| **edvs-process** | Rust | Standalone CLI for offline event filtering (zero ROS dependency) |
| **ROS Messages** | .msg | `Control`, `Event`, `EventArray` definitions for topic communication |

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

**Option 2:** Press `Ctrl+C` in the node terminal. The driver shuts down gracefully: it atomically sets the stop flag, joins the readout thread, calls `stopStream()` (noexcept), and releases all resources via RAII.

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

The `edvs_processing` crate provides three memory-safe event processing filters, compiled as a static library and linked into the C++ driver via FFI through the `FilterPipeline` RAII wrapper. All FFI boundary functions use `panic::catch_unwind()` to prevent Rust panics from unwinding into C++. Each filter is independently unit-tested, and the full pipeline has integration tests covering FFI null safety and end-to-end event processing.

### :one: Temporal Denoising (`denoise.rs`)

Rejects isolated noise events using a **nearest-neighbor spatio-temporal correlation** test.

- For each incoming event, the filter checks whether any of its **8 spatial neighbors** (3x3 window) fired within `threshold_us` microseconds.
- Events with no recent neighbor are classified as **uncorrelated noise** and discarded.
- Events with at least one recent neighbor are passed through as **valid signal**.

This preserves real edges and motion while removing random background activity.

<details>
<summary><b>:bar_chart: Temporal Denoising Diagram & Example</b> (click to expand :point_down:)</summary>

**How it works:** When an event arrives at pixel (x, y), the filter looks at the 8 surrounding neighbors. If any neighbor had an event within the last `threshold_us` microseconds, the event is valid. Otherwise it is noise.

```
  Sensor Grid (zoomed to 5x5 region)         3x3 Neighbor Check
  ================================           ====================

  Each cell shows the last event              For new event at (5,5)
  timestamp (us) at that pixel.               at t = 102,000 us:

       3     4     5     6     7                  +---+---+---+
    +-----+-----+-----+-----+-----+              | 4 | 4 | 4 |
  3 |  0  |  0  |  0  |  0  |  0  |              | 4 | 5 | 6 |  <-- neighbor
    +-----+-----+-----+-----+-----+              +---+---+---+     columns
  4 |  0  |100.0|100.5|  0  |  0  |              | 4 | 5 | 6 |
    +-----+-----+-----+-----+-----+              +---+---+---+
  5 |  0  |101.0| NEW |  0  |  0  |                3   4   5
    +-----+-----+-----+-----+-----+                   ^
  6 |  0  |  0  |  0  |  0  |  0  |              neighbor rows
    +-----+-----+-----+-----+-----+
  7 |  0  |  0  |  0  |  0  |  0  |
    +-----+-----+-----+-----+-----+

  Timestamps shown in milliseconds for readability (actual units: us)
```

**Example walkthrough** (`threshold_us = 5,000`, this project's default):

> **Note on threshold value:** This project uses a default of 5,000 us (5 ms). The reference
> [jAER](https://github.com/SensorsINI/jaer) software from ETH Zurich uses values around
> 17,000 us in its BackgroundActivityFilter configurations. The optimal value depends on
> scene dynamics and sensor bias settings. All pass/reject decisions below are computed
> exactly as the `denoise.rs` code executes them.

```
  Step 1: Event arrives at (4,4) t=100,000 us
          No neighbors have fired yet (all zeros).
          Result: REJECTED (but timestamp recorded)

          Grid at (4,4) updated: last_ts = 100,000

  Step 2: Event arrives at (5,4) t=100,500 us
          Neighbor (4,4) last fired at t=100,000
          Delta: |100,500 - 100,000| = 500 us < 5,000 us threshold
          Result: ACCEPTED (correlated with neighbor)

  Step 3: Event arrives at (4,5) t=101,000 us
          Neighbor (4,4) delta = 1,000 us < 5,000 us
          Neighbor (5,4) delta =   500 us < 5,000 us
          Result: ACCEPTED (two supporting neighbors)

  Step 4: Event arrives at (10,10) t=200,000 us
          All 8 neighbors have last_ts = 0 (never fired)
          Result: REJECTED (isolated noise event)

  Step 5: Event arrives at (5,4) t=300,000 us
          Neighbor (4,4) delta = |300,000 - 100,000| = 200,000 us > 5,000 us
          Neighbor (4,5) delta = |300,000 - 101,000| = 199,000 us > 5,000 us
          Result: REJECTED (neighbors too old, temporal gap too large)
```

> **Note on timestamps:** The event timestamps above (100,000 us, 102,000 us, etc.)
> are illustrative values chosen to clearly demonstrate filter behavior. They are not
> from a real recording, but the inter-event intervals (500 us to 200,000 us) are
> within the range produced by the DVS128 sensor, which has 15 us pixel latency
> ([Lichtsteiner et al. 2008](https://ieeexplore.ieee.org/document/4444573/)).

**Filter pipeline visualization:**

```
  Raw events from camera
  =======================

  (4,4)  t=100000 pol=+1   ──> Filter ──> REJECT  (no neighbors)
  (5,4)  t=100500 pol=-1   ──> Filter ──> PASS    (neighbor at 4,4 within 5ms)
  (4,5)  t=101000 pol=+1   ──> Filter ──> PASS    (neighbors at 4,4 and 5,4)
  (10,10) t=200000 pol=+1  ──> Filter ──> REJECT  (isolated, no neighbors)
  (5,4)  t=300000 pol=+1   ──> Filter ──> REJECT  (neighbors too old)
                                   |
                                   v
                        Filtered output: 2/5 events passed (40%)
```

</details>

### :two: Hot Pixel Filter (`hot_pixel.rs`)

Detects and suppresses **stuck or noisy pixels** that fire at abnormally high rates.

- Tracks per-pixel event counts in a **sliding time window** of configurable length.
- At the end of each window, any pixel whose count exceeds `max_rate` is flagged as **hot**.
- Events from flagged pixels are rejected in the **following window**.
- Flags are re-evaluated every window, so a pixel can recover if its rate drops.

<details>
<summary><b>:bar_chart: Hot Pixel Filter Diagram & Example</b> (click to expand :point_down:)</summary>

**How it works:** The filter divides time into fixed-length windows. It counts events per pixel in each window. If a pixel exceeds `max_rate` events, it is flagged as "hot" and all its events are rejected in the next window.

```
  Time ────────────────────────────────────────────────────────────>

  Window 1                          Window 2                     Window 3
  [0 us ─────── 1,000,000 us]      [1,000,000 ─── 2,000,000]   [2,000,000 ── ...
  |                            |    |                         |   |
  |  Count events per pixel    |    |  Evaluate + reset       |   |
  |  Pixel (3,7): 347 events   |    |  (3,7) > 100? YES: HOT |   |  Re-evaluate
  |  Pixel (0,0):   5 events   |    |  (0,0) > 100? NO : OK  |   |  hot flags
  |  Pixel (8,2): 102 events   |    |  (8,2) > 100? YES: HOT |   |
  |                            |    |                         |   |

  max_rate = 100 events/window
```

**Example walkthrough** (`window_us = 1,000,000`, `max_rate = 5`):

```
  WINDOW 1: t = 0 to 1,000,000 us
  ================================
  All pixels start unflagged. Events are counted.

  Pixel (10,10) receives these events:
    t=100,000  t=200,000  t=300,000  t=400,000
    t=500,000  t=600,000  t=700,000  t=800,000
    t=900,000  t=950,000
    Total: 10 events

  Pixel (20,20) receives these events:
    t=150,000  t=750,000
    Total: 2 events

  All events PASS (no pixels are flagged yet in Window 1)


  WINDOW BOUNDARY: t = 1,000,000 us
  ===================================
  Evaluate counts from Window 1:

    Pixel (10,10): 10 events > max_rate(5) --> FLAGGED HOT
    Pixel (20,20):  2 events < max_rate(5) --> OK
    All other pixels: 0 events             --> OK

  Reset all counters to zero.

    +---+---+---+---+---+          +---+---+---+---+---+
    |   |   |   |   |   |          |   |   |   |   |   |
    +---+---+---+---+---+          +---+---+---+---+---+
    |   | 10|   |   |   |   --->   |   |HOT|   |   |   |
    +---+---+---+---+---+   eval   +---+---+---+---+---+
    |   |   | 2 |   |   |          |   |   | ok|   |   |
    +---+---+---+---+---+          +---+---+---+---+---+
       counts (Window 1)              flags (Window 2)


  WINDOW 2: t = 1,000,000 to 2,000,000 us
  =========================================
  Events from flagged pixels are REJECTED.

  Event at (10,10) t=1,300,000  --> REJECTED (pixel is hot)
  Event at (10,10) t=1,500,000  --> REJECTED (pixel is hot)
  Event at (20,20) t=1,400,000  --> PASS     (pixel is not hot)
  Event at (50,50) t=1,600,000  --> PASS     (pixel is not hot)


  WINDOW BOUNDARY: t = 2,000,000 us
  ===================================
  Re-evaluate: If (10,10) had fewer events this window,
  it gets unflagged and can pass events again.
  HOT PIXELS CAN RECOVER.
```

**Measured noise rates from jAER reference software** ([SensorsINI/jaer, NoiseTesterFilter.java](https://github.com/SensorsINI/jaer/blob/master/src/net/sf/jaer/eventprocessing/filter/NoiseTesterFilter.java)):

```
  DVS pixel noise characterization (from jAER source code):

  Shot noise rate:  5 Hz per pixel    (default in NoiseTesterFilter)
  Leak noise rate:  0.3 Hz per pixel  (default in NoiseTesterFilter)
  Realistic leak:   0.1-0.2 Hz        (code comment: "realistic for DAVIS cameras")
  Rate limit:       25 Hz per pixel   (maximum before filter flags as excessive)
```

> **Note:** The hot pixel example above uses `max_rate = 5` (not the project default
> of 1,000) to keep the walkthrough compact. The event counts, window boundaries, and
> pass/reject decisions are computed exactly as the `hot_pixel.rs` code executes them.
> The `window_us = 1,000,000` (1 second) matches this project's default from
> `edvs_params.yaml`.

</details>

### :three: Event Accumulator (`accumulator.rs`)

Converts a stream of polarity events into a **grayscale image frame**.

- Each pixel starts at a neutral value of **128**.
- ON events (+1) increment the pixel value; OFF events (-1) decrement it.
- Values are clamped to the range **0 to 255**.
- The frame can be reset to neutral at any time.

This is useful for visualization, integration with traditional CV pipelines, and debugging.

<details>
<summary><b>:bar_chart: Event Accumulator Diagram & Example</b> (click to expand :point_down:)</summary>

**How it works:** The accumulator maintains a grayscale frame where 128 is neutral gray. Each ON event (+1 polarity) makes that pixel slightly brighter. Each OFF event (-1 polarity) makes it slightly darker. The result is a reconstructed intensity image from the event stream.

```
  Polarity Event Stream                    Accumulated Frame
  ========================                 ==================

  ON event  (+1) --> pixel gets BRIGHTER   255 = white (max brightness)
  OFF event (-1) --> pixel gets DARKER       0 = black (min brightness)
  No events      --> pixel stays at 128    128 = neutral gray (starting value)

  Brightness scale:

  0         64        128        192       255
  |---------|---------|---------|---------|
  BLACK   DARK     NEUTRAL   BRIGHT    WHITE
  (many   GRAY      GRAY     GRAY     (many
  OFF)                                  ON)
```

**Example: 4x4 sensor accumulating 8 events**

```
  Initial frame (all pixels = 128, shown as "."):

      0   1   2   3
    +---+---+---+---+
  0 | . | . | . | . |     . = 128 (neutral)
    +---+---+---+---+
  1 | . | . | . | . |
    +---+---+---+---+
  2 | . | . | . | . |
    +---+---+---+---+
  3 | . | . | . | . |
    +---+---+---+---+


  Event stream (in order):

    #   x  y  polarity   pixel before --> after
    1   1  0    +1       128 --> 129
    2   2  0    +1       128 --> 129
    3   1  1    +1       128 --> 129
    4   2  1    -1       128 --> 127
    5   1  0    +1       129 --> 130         (second ON event at same pixel)
    6   0  2    -1       128 --> 127
    7   0  2    -1       127 --> 126         (second OFF event at same pixel)
    8   3  3    +1       128 --> 129


  Resulting frame:

      0    1    2    3
    +----+----+----+----+
  0 |128 |130 |129 |128 |     130 = two ON events (brighter)
    +----+----+----+----+
  1 |128 |129 |127 |128 |     127 = one OFF event (darker)
    +----+----+----+----+
  2 |126 |128 |128 |128 |     126 = two OFF events (even darker)
    +----+----+----+----+
  3 |128 |128 |128 |129 |
    +----+----+----+----+

  Visualized as brightness (. = neutral, + = bright, - = dark):

      0   1   2   3
    +---+---+---+---+
  0 | . | + | + | . |
    +---+---+---+---+
  1 | . | + | - | . |
    +---+---+---+---+
  2 | - | . | . | . |
    +---+---+---+---+
  3 | . | . | . | + |
    +---+---+---+---+
```

**Clamping behavior at the extremes:**

```
  Pixel starts at 128 (neutral)

  After 127 consecutive ON events:   128 + 127 = 255 (WHITE, maximum)
  After 1 more ON event:             255 + 1   = 255 (clamped, cannot exceed 255)
  After 1 more ON event:             255 + 1   = 255 (still clamped)

  Pixel starts at 128 (neutral)

  After 128 consecutive OFF events:  128 - 128 = 0   (BLACK, minimum)
  After 1 more OFF event:            0   - 1   = 0   (clamped, cannot go below 0)
```

**Real-world example: edge moving across sensor**

```
  A bright edge moves left-to-right across columns 2,3,4,5:

  Time t1: edge at col 2          Time t2: edge at col 3
  (ON events at col 2)            (ON at col 3, OFF at col 2)

      0   1   2   3   4               0   1   2   3   4
    +---+---+---+---+---+           +---+---+---+---+---+
    | . | . | + | . | . |           | . | . | - | + | . |
    +---+---+---+---+---+           +---+---+---+---+---+
    | . | . | + | . | . |           | . | . | - | + | . |
    +---+---+---+---+---+           +---+---+---+---+---+

  Time t3: edge at col 4          Time t4: edge at col 5
  (ON at col 4, OFF at col 3)     (ON at col 5, OFF at col 4)

      0   1   2   3   4               0   1   2   3   4   5
    +---+---+---+---+---+           +---+---+---+---+---+---+
    | . | . | - | = | + |           | . | . | - | = | = | + |
    +---+---+---+---+---+           +---+---+---+---+---+---+
    | . | . | - | = | + |           | . | . | - | = | = | + |
    +---+---+---+---+---+           +---+---+---+---+---+---+

  . = 128 (neutral)   + = brighter   - = darker   = = returned toward neutral
```

> **Note on accuracy:** All accumulator arithmetic (128 +/- 1, clamping to 0..255) is
> computed exactly as the `accumulator.rs` code executes. The moving-edge example above
> is a conceptual illustration of how event cameras encode motion; the specific pixel
> coordinates and timing are illustrative, not from a real recording.

</details>

### :arrows_counterclockwise: Complete Filter Pipeline

Events flow through the `FilterPipeline` in sequence. Each filter can independently pass or reject an event. The pipeline is managed by RAII — filters are created on `init()` and destroyed automatically when the pipeline goes out of scope.

```
  Raw Event ──> Temporal Denoise ──> Hot Pixel Filter ──> Accumulator ──> Frame
  from camera   (reject noise)       (reject stuck px)   (build image)   output
                     |                      |
                     v                      v
                 REJECTED               REJECTED
              (isolated noise)       (defective pixel)
```

<details>
<summary><b>:bar_chart: Full Pipeline Example</b> (click to expand :point_down:)</summary>

```
  10 raw events from camera sensor (128x128, threshold=5000us, max_rate=5):

  #  x    y    timestamp    pol   Denoise         Hot Pixel       Accumulator
  -- ---  ---  -----------  ---   --------------- --------------- ----------------
  1  10   10   100,000      +1    REJECT (alone)  --              --
  2  11   10   102,000      -1    PASS (near #1)  PASS            (11,10) = 127
  3  10   11   103,000      +1    PASS (near #1)  PASS            (10,11) = 129
  4  64   64   200,000      +1    REJECT (alone)  --              --
  5  11   11   104,000      +1    PASS (near #3)  PASS            (11,11) = 129
  6  50   50   500,000      +1    REJECT (alone)  --              --
  7  10   10   105,000      -1    PASS (near #3)  PASS            (10,10) = 127
  8  10   10   106,000      +1    PASS (near #7)  PASS            (10,10) = 128
  9  10   10   107,000      -1    PASS (near #8)  PASS            (10,10) = 127
  10 10   10   108,000      +1    PASS (near #9)  PASS            (10,10) = 128

  Summary:
    Input:     10 events
    After denoise: 7 events (3 isolated noise events removed)
    After hot pixel: 7 events (no hot pixels in this window)
    Accumulator frame: mostly neutral with slight activity near (10,10)-(11,11)
```

> **Note on accuracy:** All pass/reject decisions and accumulator values in the table above
> are computed exactly as the Rust filter code executes them (verified against `denoise.rs`,
> `hot_pixel.rs`, and `accumulator.rs`). The input event data (timestamps, coordinates) are
> illustrative examples, not from a real eDVS4337 recording. The `threshold_us = 5,000` and
> `max_rate = 5` are chosen for demonstration; see `config/edvs_params.yaml` for production
> defaults.

</details>

### :test_tube: Running the Tests

```bash
cd camera/src/event_based_camera/rust/edvs_processing
cargo test
```

```
running 17 tests  (unit tests)
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

running 13 tests  (integration tests)
test test_ffi_accumulator_null_acc ... ok
test test_ffi_accumulator_round_trip ... ok
test test_ffi_create_invalid_params_returns_null ... ok
test test_ffi_hot_pixel_destroy_null ... ok
test test_ffi_hot_pixel_null_filter ... ok
test test_ffi_temporal_destroy_null ... ok
test test_ffi_temporal_null_event ... ok
test test_ffi_temporal_null_filter ... ok
test test_full_pipeline_normal_events ... ok
test test_pipeline_accumulator_clamp_after_many_events ... ok
test test_pipeline_accumulator_frame_after_mixed_events ... ok
test test_pipeline_hot_pixel_rejected_after_window ... ok
test test_pipeline_noise_rejected ... ok

test result: ok. 13 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

**30 total tests** covering: boundary pixels, temporal threshold edge cases, saturation clamping, out-of-bounds rejection, hot pixel recovery, FFI null safety, invalid parameter handling, and full pipeline integration.

---

## :computer: Standalone CLI Tool

The `edvs-process` binary reads tab-separated events from standard input, applies the configurable filter pipeline, and writes passing events to standard output. It has **zero ROS dependency** and can run anywhere Rust compiles.

### Input Format

One event per line, tab-separated:

```
x	y	timestamp	polarity
```

Lines starting with `#` are treated as comments and skipped. Only valid polarities (-1 and +1) are accepted.

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

The driver implements multiple layers of defense-in-depth hardening, addressing both the original prototype's vulnerabilities and new threats identified during security review.

### Security Measures

| Category | Measure | Details |
|:---------|:--------|:--------|
| :mag: **Input validation** | Device path validation | Path must exist under `/dev/`, must be a character device (`S_ISCHR`), must not contain `..` traversal sequences |
| :mag: **Input validation** | Control message validation | Only `sos: 0` and `sos: 1` are accepted; all other values are rejected with a warning |
| :mag: **Input validation** | Parameter validation | Filter parameters validated before `int` to `uint32_t`/`int64_t` cast; negative values rejected at startup |
| :mag: **Input validation** | Event bounds check | Out-of-bounds events (x >= width or y >= height) are logged and skipped; only valid polarities (-1, +1) accepted |
| :lock: **Access control** | Permission check | Read/write access to the serial device is verified before opening; clear error messages guide the user to fix permissions |
| :hourglass_flowing_sand: **Rate limiting** | Monotonic throttling | Uses `ros::WallTime` (immune to simulated time manipulation) to throttle control commands to one per second |
| :broom: **Resource safety** | RAII lifecycle | Camera device wrapped in `unique_ptr<EvtCamera>`; Rust FFI pointers wrapped in `RustFilterPtr<T>` RAII template; `stopStream()` is `noexcept` |
| :thread: **Concurrency** | Mutex-guarded device | `device_mutex_` protects all device access in `readoutLoop()`; `std::atomic<bool>` flag for clean stop; width/height cached locally under lock |
| :no_entry_sign: **Memory safety** | Rust processing | All event filtering in Rust; `panic::catch_unwind()` at FFI boundaries returns null on panic; `checked_mul` prevents dimension overflow; `MAX_SENSOR_DIM` bounds |
| :robot: **Static analysis** | CodeQL CI | Automated C++ vulnerability scanning on every push and on a weekly schedule |
| :package: **Supply chain** | SLSA Level 3 | Release builds include cryptographic provenance attestations |

### Vulnerability Remediation Log

| ID | Issue | Severity | Resolution |
|:---|:------|:---------|:-----------|
| S1 | Hardcoded device path `/dev/ttyUSB0` | HIGH | Parameterized via ROS param with validation |
| S5/S6 | Memory leaks and double-free in `tensor.hpp` | CRITICAL | File deleted; Rust handles data processing |
| S7 | Data races in `buf.hpp` query methods | HIGH | File deleted; no circular buffers needed |
| S9 | `int64_t` timestamp truncated to `int` | MEDIUM | `int64_t` used throughout the pipeline |
| S10 | Infinite loop in ROS callback (DoS) | HIGH | Threaded readout with `std::atomic<bool>` |
| S13 | Serial port left open on shutdown | MEDIUM | RAII destructor handles cleanup |
| S14 | Use-after-free race in `readoutLoop` device access | CRITICAL | Mutex-guarded device read; cached width/height locally |
| S15 | Integer overflow in `width * height` allocation | HIGH | `checked_mul` + `MAX_SENSOR_DIM` bounds in Rust constructors |
| S16 | Negative int cast to uint32_t for filter params | HIGH | C++ validates parameters > 0 before casting |
| S17 | Clock skew bypass via simulated time rate-limit | MEDIUM | Switched to monotonic `ros::WallTime` |
| S18 | `stopStream()` could throw during cleanup | MEDIUM | Declared `noexcept`; body wrapped in try-catch |

---

## :factory: CI/CD Pipeline

Three GitHub Actions workflows run automatically on every push and pull request.

| Workflow | Trigger | Purpose |
|:---------|:--------|:--------|
| **CMake Build & Test** | Push, PR | Builds the Rust crate (`cargo build`, `cargo test` with 30 tests), then builds the full ROS package (`catkin_make`) on Ubuntu 20.04 |
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
