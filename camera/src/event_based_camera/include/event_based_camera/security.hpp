#ifndef EDVS_SECURITY_HPP
#define EDVS_SECURITY_HPP

#include <cstdint>
#include <string>
#include <ros/ros.h>

namespace edvs_security {

/// Result of a security check.  Caller decides how to log.
struct ValidationResult {
    bool ok;
    std::string message;  // empty on success; descriptive error on failure
};

/// Validate that a serial port path is safe to open.
/// Checks: path exists, is a character device, resides under /dev/.
ValidationResult validateSerialPort(const std::string& path);

/// Check that the current user has read/write access to the device.
ValidationResult checkPermissions(const std::string& path);

/// Validate the control message sos field (only 0 or 1 accepted).
ValidationResult validateControlMessage(int32_t sos);

/// Rate-limit check: returns ok=true if enough time has elapsed since last_time.
/// Updates last_time to now if the check passes.
/// Uses WallTime (monotonic) to prevent clock-skew bypass via simulated time.
ValidationResult rateLimitCheck(ros::WallTime& last_time, double min_interval_sec);

}  // namespace edvs_security

#endif  // EDVS_SECURITY_HPP
