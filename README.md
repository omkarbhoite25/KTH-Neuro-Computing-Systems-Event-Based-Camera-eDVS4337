# eDVS4337 Event-Based Camera: ROS Driver and Rust Processing Pipeline

A secure, modular ROS driver for the eDVS4337 neuromorphic (event-based) camera, paired with a Rust signal-processing library for real-time denoising and filtering.

**Project by:** Omkar Vilas Bhoite
**Advised by:** Prof. Jorg Conradt and Juan Pablo Romero Bermudez, KTH Royal Institute of Technology

<img src="https://github.com/omkarbhoite25/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/blob/main/images/eDVS.png" width="800">

## What Is an Event Camera?

Unlike a conventional camera that captures full frames at a fixed rate, the eDVS4337 produces **asynchronous polarity events**. Each pixel independently reports brightness changes as they happen, yielding a stream of `(x, y, timestamp, polarity)` tuples with microsecond resolution. This makes event cameras ideal for high-speed robotics, low-latency tracking, and neuromorphic computing.

## Architecture

The project has two main components:

```
camera/src/event_based_camera/
  include/event_based_camera/
    edvs_driver.hpp          C++ driver class (RAII, threaded readout)
    event_types.hpp          Shared event struct (C/Rust FFI compatible)
    security.hpp             Input validation and rate limiting
  src/
    edvs_driver_node.cpp     Thin ROS entry point
    edvs_driver.cpp          Driver implementation
    security.cpp             Security utilities
  msg/
    Control.msg              Start/stop command
    Event.msg                Single polarity event
    EventArray.msg           Batched events with header
  config/
    edvs_params.yaml         Default parameters
  launch/
    edvs_camera.launch       Single-command launch file
  rust/edvs_processing/      Rust crate (event filters, CLI tool)
    src/
      event.rs               FFI-compatible Event type
      denoise.rs             Temporal nearest-neighbor filter
      hot_pixel.rs           Hot pixel detector
      accumulator.rs         Event-to-frame accumulator
      ffi.rs                 C-compatible function exports
      bin/edvs_process.rs    Standalone CLI for offline filtering
```

**C++ ROS node** handles device communication through libcaer and publishes events to ROS topics. **Rust library** provides memory-safe event processing (denoising, hot pixel filtering, frame accumulation), linked into the C++ node via FFI.

## Prerequisites

### ROS

Install ROS Noetic or Melodic by following the official guide at https://wiki.ros.org/ROS/Installation.

### Rust

Install the Rust toolchain using rustup:

```bash
curl https://sh.rustup.rs -sSf | sh
source $HOME/.cargo/env
```

### System Dependencies

```bash
sudo apt-get install build-essential cmake pkg-config libusb-1.0-0-dev libserialport-dev
```

### libcaer (eDVS Camera Library)

```bash
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install libcaer-dev
```

### Serial Port Access

The eDVS communicates over a serial port (typically `/dev/ttyUSB0` or `/dev/ttyACM0`). Your user account must have permission to access it.

1. Check which port the camera is connected to:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

2. Verify that your user is in the `dialout` group:

```bash
id $USER
```

If `dialout` does not appear in the output, add yourself to the group:

```bash
sudo usermod -a -G dialout $USER
```

Log out and back in for the group change to take effect.

3. If you still cannot open the port, grant read and write access manually:

```bash
sudo chmod a+rw /dev/ttyUSB0
```

Replace `ttyUSB0` with your actual device name.

## Building

Build both the Rust processing library and the ROS package:

```bash
# Build the Rust crate first
cd camera/src/event_based_camera/rust/edvs_processing
cargo build --release

# Build the ROS package
cd ../../../../..
cd camera
catkin_make
source devel/setup.bash
```

To avoid re-sourcing every time you open a terminal, add this line to your `~/.bashrc`:

```bash
source ~/KTH-Neuro-Computing-Systems-Event-Based-Camera-eDVS4337/camera/devel/setup.bash
```

## Running

### Option A: Using the Launch File (Recommended)

This is the simplest way to start the camera node. It loads all parameters from the config file automatically.

```bash
# Terminal 1: start the ROS master
roscore

# Terminal 2: launch the camera node
roslaunch event_based_camera edvs_camera.launch
```

Then send a start command from a third terminal:

```bash
# Terminal 3: start capturing events
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 1"
```

### Option B: Running the Node Directly

If you prefer to set parameters on the command line:

```bash
# Terminal 1
roscore

# Terminal 2: run with a custom serial port
rosrun event_based_camera edvs_camera _serial_port:=/dev/ttyACM0

# Terminal 3: start the camera
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 1"
```

### Stopping the Camera

Send a stop command:

```bash
rostopic pub -1 /edvs/control event_based_camera/Control "sos: 0"
```

Or press `Ctrl+C` in the node terminal. The driver shuts down gracefully, stopping the data stream and releasing the serial port.

## Viewing Events

Once the camera is running, events are published on the `/edvs/events` topic as `EventArray` messages.

### Echo Events in the Terminal

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

This prints one `EventArray` message containing a batch of events. Each event has fields `x`, `y`, `timestamp` (microseconds), and `polarity` (+1 for brightness increase, -1 for decrease).

## Configuration

All parameters are set in `config/edvs_params.yaml` and can be overridden at launch time.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Path to the eDVS serial device |
| `enable_denoising` | `true` | Enable temporal nearest-neighbor denoising |
| `denoise_threshold_us` | `5000` | Time window in microseconds for neighbor correlation |
| `enable_hot_pixel_filter` | `true` | Enable hot pixel detection and rejection |
| `hot_pixel_window_us` | `1000000` | Observation window in microseconds (1 second) |
| `hot_pixel_max_rate` | `1000` | Maximum events per pixel per window before marking as hot |

### Overriding Parameters at Launch

```bash
roslaunch event_based_camera edvs_camera.launch serial_port:=/dev/ttyACM0
```

Or when running the node directly:

```bash
rosrun event_based_camera edvs_camera _serial_port:=/dev/ttyACM0 _enable_denoising:=false
```

## Rust Processing Library

The `edvs_processing` crate provides three event filters, all callable from C++ via FFI and independently testable.

### Temporal Denoising

Rejects isolated noise events. An event passes only if at least one of its eight spatial neighbors fired within the configured time threshold. This removes uncorrelated background noise while preserving real edges and motion.

### Hot Pixel Filter

Detects pixels that fire at abnormally high rates. The filter tracks per-pixel event counts in a sliding time window and flags any pixel exceeding the maximum rate. Events from flagged pixels are suppressed in the following window.

### Accumulator

Converts an event stream into a grayscale frame. Each pixel starts at a neutral value of 128. ON events (+1 polarity) increment the pixel, and OFF events (-1 polarity) decrement it, clamped to the range 0 to 255. The frame can be reset at any time.

### Running the Tests

```bash
cd camera/src/event_based_camera/rust/edvs_processing
cargo test
```

All 17 unit tests cover edge cases including boundary pixels, temporal thresholds, saturation clamping, and out-of-bounds rejection.

## Standalone CLI Tool

The `edvs-process` binary reads tab-separated events from standard input, applies filters, and writes passing events to standard output. It has no ROS dependency.

### Example: Filter a Recorded Event File

```bash
cat recorded_events.tsv | edvs-process > filtered_events.tsv
```

The input format is one event per line, tab-separated: `x  y  timestamp  polarity`.

### Example: Disable Denoising, Keep Only Hot Pixel Filtering

```bash
cat events.tsv | edvs-process --denoise-us 0 > filtered.tsv
```

### Example: Use Custom Sensor Dimensions

```bash
cat events.tsv | edvs-process --width 64 --height 64 --denoise-us 3000 > out.tsv
```

### Full Usage

```
edvs-process [OPTIONS]

Options:
  --width N            Sensor width in pixels (default: 128)
  --height N           Sensor height in pixels (default: 128)
  --denoise-us N       Temporal denoise threshold in microseconds (default: 5000, 0 to disable)
  --hot-pixel-rate N   Maximum events per pixel per window (default: 1000, 0 to disable)
  --hot-pixel-window N Hot pixel observation window in microseconds (default: 1000000)
  --help               Show help
```

The tool prints a summary to standard error when it finishes:

```
edvs-process: 84210/100000 events passed filters (84.2%)
```

## ROS Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/edvs/control` | `event_based_camera/Control` | Subscriber | Send `sos: 1` to start, `sos: 0` to stop |
| `/edvs/events` | `event_based_camera/EventArray` | Publisher | Batched filtered events with header |

## Security

The driver includes several hardening measures:

- **Device path validation:** The serial port path must exist under `/dev/`, must be a character device, and must not contain path traversal sequences. Permissions are checked before the device is opened.
- **Input validation:** The `sos` field in control messages accepts only 0 or 1. All other values are rejected with a warning.
- **Rate limiting:** Control commands are throttled to one per second, preventing accidental flooding.
- **RAII resource management:** The camera device is wrapped in a smart pointer. The destructor stops the data stream and closes the serial port automatically, even if the node is killed.
- **Threaded readout:** Event reading runs in a separate thread, so the ROS callback queue is never blocked.
- **CI/CD security scanning:** CodeQL runs on every push and weekly, checking for buffer overflows, format string issues, and other C++ vulnerabilities.

## CI/CD

Three GitHub Actions workflows run automatically:

- **cmake.yml** builds the Rust crate and the ROS package on every push and pull request to `main`.
- **codeql.yml** performs CodeQL static analysis on C++ code, running on push, pull request, and a weekly schedule.
- **slsa-provenance.yml** generates SLSA Level 3 supply-chain provenance attestations for release builds.

## Troubleshooting

**The node exits with "Serial port validation failed"**
Check that the device path is correct. Run `ls /dev/ttyUSB* /dev/ttyACM*` to find the camera. Update the `serial_port` parameter accordingly.

**The node exits with "Serial port permission check failed"**
Your user does not have read/write access to the device. Add yourself to the `dialout` group and log out, or run `sudo chmod a+rw /dev/ttyUSB0`.

**"Command not found" when running rosrun or roslaunch**
You need to source the workspace. Run `source camera/devel/setup.bash` in your terminal.

**Events are being published but many are filtered out**
If the denoising threshold is too strict, real events may be rejected along with noise. Try increasing `denoise_threshold_us` (e.g., to 10000) or disabling denoising entirely by setting `enable_denoising` to `false`.

**The camera does not respond after sending sos: 1**
Make sure the previous command was sent at least one second ago. The driver rate-limits control commands to prevent accidental spam.

## License

MIT
