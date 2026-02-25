#ifndef EDVS_SECURITY_HPP
#define EDVS_SECURITY_HPP

#include <string>
#include <ros/ros.h>

namespace edvs_security {

/// Validate that a serial port path is safe to open.
/// Checks: path exists, is a character device, resides under /dev/.
bool validateSerialPort(const std::string& path);

/// Check that the current user has read/write access to the device.
bool checkPermissions(const std::string& path);

/// Validate the control message sos field (only 0 or 1 accepted).
bool validateControlMessage(int32_t sos);

/// Rate-limit check: returns true if enough time has elapsed since last_time.
/// Updates last_time to now if the check passes.
bool rateLimitCheck(ros::Time& last_time, double min_interval_sec);

}  // namespace edvs_security

#endif  // EDVS_SECURITY_HPP
