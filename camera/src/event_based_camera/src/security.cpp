#include "event_based_camera/security.hpp"

#include <sys/stat.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>

namespace edvs_security {

namespace {
/// Thread-safe wrapper around strerror_r (GNU variant).
std::string safe_strerror(int errnum) {
    char buf[256];
    // GNU strerror_r returns char* (may or may not use buf)
    const char* msg = strerror_r(errnum, buf, sizeof(buf));
    return std::string(msg);
}

ValidationResult ok() {
    return {true, {}};
}

ValidationResult fail(const std::string& msg) {
    return {false, msg};
}
}  // namespace

ValidationResult validateSerialPort(const std::string& path) {
    // Must start with /dev/
    if (path.find("/dev/") != 0) {
        return fail("Serial port path must be under /dev/: " + path);
    }

    // Reject path traversal attempts
    if (path.find("..") != std::string::npos) {
        return fail("Serial port path contains '..': " + path);
    }

    // Check the path exists and is a character device
    struct stat st;
    if (stat(path.c_str(), &st) != 0) {
        return fail("Serial port does not exist: " + path + " (" + safe_strerror(errno) + ")");
    }

    if (!S_ISCHR(st.st_mode)) {
        return fail("Path is not a character device: " + path);
    }

    return ok();
}

ValidationResult checkPermissions(const std::string& path) {
    if (access(path.c_str(), R_OK | W_OK) != 0) {
        return fail("Insufficient permissions for " + path +
                    ". Ensure user is in 'dialout' group and device has rw access. ("
                    + safe_strerror(errno) + ")");
    }
    return ok();
}

ValidationResult validateControlMessage(int32_t sos) {
    if (sos != 0 && sos != 1) {
        return fail("Invalid control message sos=" + std::to_string(sos)
                    + ", expected 0 or 1. Ignoring.");
    }
    return ok();
}

ValidationResult rateLimitCheck(ros::WallTime& last_time, double min_interval_sec) {
    ros::WallTime now = ros::WallTime::now();

    // Allow the first command (last_time is zero/uninitialized)
    if (last_time.isZero()) {
        last_time = now;
        return ok();
    }

    double elapsed = (now - last_time).toSec();
    if (elapsed < min_interval_sec) {
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "Control command rate-limited: %.1fs since last command (min %.1fs)",
                 elapsed, min_interval_sec);
        return fail(std::string(buf));
    }

    last_time = now;
    return ok();
}

}  // namespace edvs_security
