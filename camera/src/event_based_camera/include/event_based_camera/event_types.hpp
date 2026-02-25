#ifndef EDVS_EVENT_TYPES_HPP
#define EDVS_EVENT_TYPES_HPP

#include <cstdint>

namespace edvs {

/// Single polarity event from the eDVS camera.
/// Layout is C-compatible for FFI with Rust.
struct __attribute__((packed)) Event {
    uint16_t x;
    uint16_t y;
    int64_t timestamp;  // microseconds
    int8_t polarity;    // -1 or +1
};

/// Default eDVS4337 resolution constants
static constexpr uint16_t EDVS_DEFAULT_WIDTH = 128;
static constexpr uint16_t EDVS_DEFAULT_HEIGHT = 128;

}  // namespace edvs

#endif  // EDVS_EVENT_TYPES_HPP
