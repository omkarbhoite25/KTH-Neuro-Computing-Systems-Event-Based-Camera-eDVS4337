#include "event_based_camera/edvs_driver.hpp"
#include "event_based_camera/security.hpp"
#include "event_based_camera/Event.h"
#include "event_based_camera/EventArray.h"

#include <vector>

EdvsDriver::EdvsDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , temporal_filter_(nullptr)
    , hot_pixel_filter_(nullptr)
    , dvs_size_x_(edvs::EDVS_DEFAULT_WIDTH)
    , dvs_size_y_(edvs::EDVS_DEFAULT_HEIGHT)
{
    // Load parameters
    pnh_.param<std::string>("serial_port", serial_port_, "/dev/ttyUSB0");
    pnh_.param<bool>("enable_denoising", enable_denoising_, true);
    pnh_.param<bool>("enable_hot_pixel_filter", enable_hot_pixel_filter_, true);

    int denoise_us = 5000;
    pnh_.param<int>("denoise_threshold_us", denoise_us, 5000);
    denoise_threshold_us_ = static_cast<int64_t>(denoise_us);

    int hp_window = 1000000;
    pnh_.param<int>("hot_pixel_window_us", hp_window, 1000000);
    hot_pixel_window_us_ = static_cast<int64_t>(hp_window);

    int hp_max_rate = 1000;
    pnh_.param<int>("hot_pixel_max_rate", hp_max_rate, 1000);
    hot_pixel_max_rate_ = static_cast<uint32_t>(hp_max_rate);

    // Validate serial port before anything else
    if (!edvs_security::validateSerialPort(serial_port_)) {
        throw std::runtime_error("Serial port validation failed: " + serial_port_);
    }
    if (!edvs_security::checkPermissions(serial_port_)) {
        throw std::runtime_error("Serial port permission check failed: " + serial_port_);
    }

    // Setup ROS pub/sub
    event_pub_ = nh_.advertise<event_based_camera::EventArray>("/edvs/events", 100);
    control_sub_ = nh_.subscribe("/edvs/control", 10, &EdvsDriver::controlCallback, this);

    ROS_INFO("eDVS driver initialized. Serial port: %s", serial_port_.c_str());
    ROS_INFO("Waiting for start command on /edvs/control (sos: 1)...");
}

EdvsDriver::~EdvsDriver() {
    stop();

    if (temporal_filter_) {
        edvs_temporal_filter_destroy(temporal_filter_);
        temporal_filter_ = nullptr;
    }
    if (hot_pixel_filter_) {
        edvs_hot_pixel_filter_destroy(hot_pixel_filter_);
        hot_pixel_filter_ = nullptr;
    }

    ROS_INFO("eDVS driver shut down cleanly.");
}

void EdvsDriver::controlCallback(const event_based_camera::Control::ConstPtr& msg) {
    if (!edvs_security::validateControlMessage(msg->sos)) {
        return;
    }

    if (!edvs_security::rateLimitCheck(last_control_time_, MIN_CONTROL_INTERVAL_SEC)) {
        return;
    }

    if (msg->sos == 1) {
        start();
    } else {
        stop();
    }
}

void EdvsDriver::start() {
    if (running_.load()) {
        ROS_WARN("Camera is already running.");
        return;
    }

    try {
        // Open the eDVS device
        device_ = std::make_unique<libcaer::devices::edvs>(
            1, serial_port_.c_str(), CAER_HOST_CONFIG_SERIAL_BAUD_RATE_12M);

        struct caer_edvs_info edvs_info = device_->infoGet();
        dvs_size_x_ = static_cast<uint16_t>(edvs_info.dvsSizeX);
        dvs_size_y_ = static_cast<uint16_t>(edvs_info.dvsSizeY);

        ROS_INFO("eDVS opened — ID: %d, Master: %d, DVS: %dx%d",
                 edvs_info.deviceID, edvs_info.deviceIsMaster,
                 dvs_size_x_, dvs_size_y_);

        device_->sendDefaultConfig();
        device_->dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);
        device_->configSet(CAER_HOST_CONFIG_DATAEXCHANGE,
                           CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

        // Initialize Rust filters
        if (enable_denoising_ && !temporal_filter_) {
            temporal_filter_ = edvs_temporal_filter_create(
                dvs_size_x_, dvs_size_y_, denoise_threshold_us_);
            ROS_INFO("Temporal denoising enabled (threshold: %ld us)", denoise_threshold_us_);
        }
        if (enable_hot_pixel_filter_ && !hot_pixel_filter_) {
            hot_pixel_filter_ = edvs_hot_pixel_filter_create(
                dvs_size_x_, dvs_size_y_, hot_pixel_window_us_, hot_pixel_max_rate_);
            ROS_INFO("Hot pixel filter enabled (max rate: %u events/window)", hot_pixel_max_rate_);
        }

        running_.store(true);
        readout_thread_ = std::thread(&EdvsDriver::readoutLoop, this);

        ROS_INFO("Camera started — publishing events on /edvs/events");

    } catch (const std::exception& e) {
        ROS_ERROR("Failed to start eDVS camera: %s", e.what());
        device_.reset();
    }
}

void EdvsDriver::stop() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    if (readout_thread_.joinable()) {
        readout_thread_.join();
    }

    if (device_) {
        try {
            device_->dataStop();
        } catch (const std::exception& e) {
            ROS_WARN("Error stopping data stream: %s", e.what());
        }
        device_.reset();
    }

    ROS_INFO("Camera stopped.");
}

void EdvsDriver::readoutLoop() {
    while (running_.load() && ros::ok()) {
        try {
            std::unique_ptr<libcaer::events::EventPacketContainer> packet_container
                = device_->dataGet();

            if (!packet_container) {
                continue;
            }

            for (auto& packet : *packet_container) {
                if (!packet || packet->getEventType() != POLARITY_EVENT) {
                    continue;
                }

                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                    = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                int32_t event_count = packet->getEventNumber();
                if (event_count <= 0) {
                    continue;
                }

                // Build the EventArray message
                event_based_camera::EventArray event_array_msg;
                event_array_msg.header.stamp = ros::Time::now();
                event_array_msg.header.frame_id = "edvs";
                event_array_msg.width = dvs_size_x_;
                event_array_msg.height = dvs_size_y_;
                event_array_msg.events.reserve(static_cast<size_t>(event_count));

                for (int32_t i = 0; i < event_count; ++i) {
                    const libcaer::events::PolarityEvent& ev = (*polarity)[i];

                    edvs::Event raw_event;
                    raw_event.x = ev.getX();
                    raw_event.y = ev.getY();
                    raw_event.timestamp = ev.getTimestamp64(*polarity);
                    raw_event.polarity = static_cast<int8_t>(2 * static_cast<int>(ev.getPolarity()) - 1);

                    // Apply Rust filters
                    if (temporal_filter_) {
                        if (!edvs_temporal_filter_process(temporal_filter_, &raw_event)) {
                            continue;  // filtered out as noise
                        }
                    }
                    if (hot_pixel_filter_) {
                        if (!edvs_hot_pixel_filter_process(hot_pixel_filter_, &raw_event)) {
                            continue;  // filtered out as hot pixel
                        }
                    }

                    // Pack into ROS message
                    event_based_camera::Event event_msg;
                    event_msg.x = raw_event.x;
                    event_msg.y = raw_event.y;
                    event_msg.timestamp = raw_event.timestamp;
                    event_msg.polarity = raw_event.polarity;
                    event_array_msg.events.push_back(event_msg);
                }

                if (!event_array_msg.events.empty()) {
                    event_pub_.publish(event_array_msg);
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(5.0, "Error reading events: %s", e.what());
        }
    }
}
