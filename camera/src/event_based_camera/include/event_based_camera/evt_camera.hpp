#ifndef EVT_CAMERA_HPP
#define EVT_CAMERA_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "event_based_camera/event_types.hpp"

/// Camera metadata returned after opening a device.
struct CameraInfo {
    uint16_t width;
    uint16_t height;
    int device_id;
    bool is_master;
};

/// Abstract interface for an event-based camera device.
///
/// Decouples the driver from any specific SDK (libcaer, Metavision, etc.)
/// and enables testing with mock implementations.
class EvtCamera {
public:
    virtual ~EvtCamera() = default;

    /// Open the device and return its metadata.
    virtual CameraInfo open(const std::string& port, uint32_t baud_rate) = 0;

    /// Send default configuration and start the data stream.
    virtual void startStream() = 0;

    /// Stop the data stream.  Must not throw — called during cleanup/destruction.
    virtual void stopStream() noexcept = 0;

    /// Read the next batch of polarity events (blocking).
    /// Returns an empty vector if no events are available.
    virtual std::vector<edvs::Event> readEvents() = 0;
};

#endif  // EVT_CAMERA_HPP
