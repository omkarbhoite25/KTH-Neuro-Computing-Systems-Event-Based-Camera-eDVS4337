#include "event_based_camera/edvs_driver.hpp"
#include "event_based_camera/libcaer_edvs.hpp"
#include "event_based_camera/security.hpp"
#include "event_based_camera/Event.h"
#include "event_based_camera/EventArray.h"

#include <vector>

EdvsDriver::EdvsDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , dvs_size_x_(edvs::EDVS_DEFAULT_WIDTH)
    , dvs_size_y_(edvs::EDVS_DEFAULT_HEIGHT)
{
    // Load parameters
    pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");

    pnh_.param<bool>("enable_denoising", filter_config_.enable_denoising, true);
    pnh_.param<bool>("enable_hot_pixel_filter", filter_config_.enable_hot_pixel, true);

    int denoise_us = 5000;
    pnh_.param<int>("denoise_threshold_us", denoise_us, 5000);
    if (filter_config_.enable_denoising && denoise_us <= 0) {
        throw std::runtime_error("denoise_threshold_us must be positive, got: "
                                 + std::to_string(denoise_us));
    }
    filter_config_.denoise_threshold_us = static_cast<int64_t>(denoise_us);

    int hp_window = 1000000;
    pnh_.param<int>("hot_pixel_window_us", hp_window, 1000000);
    if (filter_config_.enable_hot_pixel && hp_window <= 0) {
        throw std::runtime_error("hot_pixel_window_us must be positive, got: "
                                 + std::to_string(hp_window));
    }
    filter_config_.hot_pixel_window_us = static_cast<int64_t>(hp_window);

    int hp_max_rate = 1000;
    pnh_.param<int>("hot_pixel_max_rate", hp_max_rate, 1000);
    if (filter_config_.enable_hot_pixel && hp_max_rate <= 0) {
        throw std::runtime_error("hot_pixel_max_rate must be positive, got: "
                                 + std::to_string(hp_max_rate));
    }
    filter_config_.hot_pixel_max_rate = static_cast<uint32_t>(hp_max_rate);

    // Validate serial port before anything else
    auto port_result = edvs_security::validateSerialPort(serial_port_);
    if (!port_result.ok) {
        ROS_ERROR("%s", port_result.message.c_str());
        throw std::runtime_error("Serial port validation failed: " + port_result.message);
    }

    auto perm_result = edvs_security::checkPermissions(serial_port_);
    if (!perm_result.ok) {
        ROS_ERROR("%s", perm_result.message.c_str());
        throw std::runtime_error("Serial port permission check failed: " + perm_result.message);
    }

    // Setup ROS pub/sub
    event_pub_ = nh_.advertise<event_based_camera::EventArray>("/edvs/events", 100);
    control_sub_ = nh_.subscribe("/edvs/control", 10, &EdvsDriver::controlCallback, this);

    ROS_INFO("eDVS driver initialized. Serial port: %s", serial_port_.c_str());
    ROS_INFO("Waiting for start command on /edvs/control (sos: 1)...");
}

EdvsDriver::~EdvsDriver() {
    stop();
    // RAII wrappers handle filter destruction automatically
    ROS_INFO("eDVS driver shut down cleanly.");
}

void EdvsDriver::controlCallback(const event_based_camera::Control::ConstPtr& msg) {
    auto ctrl_result = edvs_security::validateControlMessage(msg->sos);
    if (!ctrl_result.ok) {
        ROS_WARN("%s", ctrl_result.message.c_str());
        return;
    }

    auto rate_result = edvs_security::rateLimitCheck(last_control_time_, MIN_CONTROL_INTERVAL_SEC);
    if (!rate_result.ok) {
        ROS_WARN("%s", rate_result.message.c_str());
        return;
    }

    if (msg->sos == 1) {
        start();
    } else {
        stop();
    }
}

void EdvsDriver::start() {
    std::lock_guard<std::mutex> lock(device_mutex_);

    if (running_.load()) {
        ROS_WARN("Camera is already running.");
        return;
    }

    try {
        // Open the eDVS device via abstraction layer
        device_ = std::make_unique<LibcaerEdvs>();
        CameraInfo info = device_->open(serial_port_, CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M);

        dvs_size_x_ = info.width;
        dvs_size_y_ = info.height;

        ROS_INFO("eDVS opened — ID: %d, Master: %d, DVS: %dx%d",
                 info.device_id, info.is_master,
                 dvs_size_x_, dvs_size_y_);

        device_->startStream();

        // Initialize filter pipeline
        pipeline_.init(dvs_size_x_, dvs_size_y_, filter_config_);

        if (pipeline_.hasDenoising()) {
            ROS_INFO("Temporal denoising enabled (threshold: %ld us)",
                     filter_config_.denoise_threshold_us);
        }
        if (pipeline_.hasHotPixel()) {
            ROS_INFO("Hot pixel filter enabled (max rate: %u events/window)",
                     filter_config_.hot_pixel_max_rate);
        }

        running_.store(true);
        readout_thread_ = std::thread(&EdvsDriver::readoutLoop, this);

        ROS_INFO("Camera started — publishing events on /edvs/events");

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to start eDVS camera: %s", e.what());
        pipeline_.reset();
        device_.reset();
    }
}

void EdvsDriver::stop() {
    // Set running_ to false first (atomic, no lock needed).
    // The readout loop checks this and will exit after its current iteration.
    if (!running_.exchange(false)) {
        return;  // Was already stopped
    }

    // Join the readout thread (it will release device_mutex_ and exit).
    if (readout_thread_.joinable()) {
        readout_thread_.join();
    }

    // Now safe to clean up — readout thread has exited.
    std::lock_guard<std::mutex> lock(device_mutex_);
    if (device_) {
        device_->stopStream();  // noexcept — safe during cleanup
        device_.reset();
    }

    ROS_INFO("Camera stopped.");
}

void EdvsDriver::readoutLoop() {
    while (running_.load() && ros::ok()) {
        try {
            // Read events under the device mutex to prevent use-after-free
            // if stop() is called concurrently.
            std::vector<edvs::Event> events;
            uint16_t width, height;
            {
                std::lock_guard<std::mutex> lock(device_mutex_);
                if (!device_) {
                    break;
                }
                events = device_->readEvents();
                width = dvs_size_x_;
                height = dvs_size_y_;
            }

            if (events.empty()) {
                continue;
            }

            // Build the EventArray message
            event_based_camera::EventArray event_array_msg;
            event_array_msg.header.stamp = ros::Time::now();
            event_array_msg.header.frame_id = "edvs";
            event_array_msg.width = width;
            event_array_msg.height = height;
            event_array_msg.events.reserve(events.size());

            for (auto& ev : events) {
                if (ev.x >= width || ev.y >= height) {
                    ROS_WARN_THROTTLE(10.0, "Out-of-bounds event: (%u,%u) vs sensor (%u,%u)",
                                      ev.x, ev.y, width, height);
                    continue;
                }

                if (!pipeline_.process(&ev)) {
                    continue;
                }

                // Pack into ROS message
                event_based_camera::Event event_msg;
                event_msg.x = ev.x;
                event_msg.y = ev.y;
                event_msg.timestamp = ev.timestamp;
                event_msg.polarity = ev.polarity;
                event_array_msg.events.push_back(event_msg);
            }

            if (!event_array_msg.events.empty()) {
                event_pub_.publish(event_array_msg);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(5.0, "Error reading events: %s", e.what());
        }
    }
}
