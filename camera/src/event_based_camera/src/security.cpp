#include "event_based_camera/security.hpp"

#include <sys/stat.h>
#include <unistd.h>
#include <cstring>

namespace edvs_security {

bool validateSerialPort(const std::string& path) {
    // Must start with /dev/
    if (path.find("/dev/") != 0) {
        ROS_ERROR("Serial port path must be under /dev/: %s", path.c_str());
        return false;
    }

    // Reject path traversal attempts
    if (path.find("..") != std::string::npos) {
        ROS_ERROR("Serial port path contains '..': %s", path.c_str());
        return false;
    }

    // Check the path exists and is a character device
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        ROS_ERROR("Serial port does not exist: %s (%s)", path.c_str(), strerror(errno));
        return false;
    }

    if (!S_ISCHR(st.st_mode)) {
        ROS_ERROR("Path is not a character device: %s", path.c_str());
        return false;
    }

    return true;
}

bool checkPermissions(const std::string& path) {
    if (access(path.c_str(), R_OK | W_OK) != 0) {
        ROS_ERROR("Insufficient permissions for %s. "
                  "Ensure user is in 'dialout' group and device has rw access. (%s)",
                  path.c_str(), strerror(errno));
        return false;
    }
    return true;
}

bool validateControlMessage(int32_t sos) {
    if (sos != 0 && sos != 1) {
        ROS_WARN("Invalid control message sos=%d, expected 0 or 1. Ignoring.", sos);
        return false;
    }
    return true;
}

bool rateLimitCheck(ros::Time& last_time, double min_interval_sec) {
    ros::Time now = ros::Time::now();

    // Allow the first command (last_time is zero/uninitialized)
    if (last_time.isZero()) {
        last_time = now;
        return true;
    }

    double elapsed = (now - last_time).toSec();
    if (elapsed < min_interval_sec) {
        ROS_WARN("Control command rate-limited: %.1fs since last command (min %.1fs)",
                 elapsed, min_interval_sec);
        return false;
    }

    last_time = now;
    return true;
}

}  // namespace edvs_security
