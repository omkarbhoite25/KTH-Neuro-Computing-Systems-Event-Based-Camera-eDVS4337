#ifndef EDVS_DRIVER_HPP
#define EDVS_DRIVER_HPP

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <libcaercpp/devices/edvs.hpp>

#include "event_based_camera/Control.h"
#include "event_based_camera/event_types.hpp"

// Forward-declare Rust FFI types (opaque pointers)
extern "C" {
    struct TemporalFilter;
    struct HotPixelFilter;

    TemporalFilter* edvs_temporal_filter_create(uint32_t width, uint32_t height, int64_t threshold_us);
    bool edvs_temporal_filter_process(TemporalFilter* filter, const edvs::Event* event);
    void edvs_temporal_filter_destroy(TemporalFilter* filter);

    HotPixelFilter* edvs_hot_pixel_filter_create(uint32_t width, uint32_t height,
                                                  int64_t window_us, uint32_t max_rate);
    bool edvs_hot_pixel_filter_process(HotPixelFilter* filter, const edvs::Event* event);
    void edvs_hot_pixel_filter_destroy(HotPixelFilter* filter);
}

class EdvsDriver {
public:
    EdvsDriver(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~EdvsDriver();

    // Non-copyable, non-movable
    EdvsDriver(const EdvsDriver&) = delete;
    EdvsDriver& operator=(const EdvsDriver&) = delete;

private:
    void controlCallback(const event_based_camera::Control::ConstPtr& msg);
    void readoutLoop();
    void start();
    void stop();

    // Device
    std::unique_ptr<libcaer::devices::edvs> device_;
    std::atomic<bool> running_{false};
    std::thread readout_thread_;

    // ROS
    ros::Subscriber control_sub_;
    ros::Publisher event_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Parameters
    std::string serial_port_;
    bool enable_denoising_;
    int64_t denoise_threshold_us_;
    bool enable_hot_pixel_filter_;
    int64_t hot_pixel_window_us_;
    uint32_t hot_pixel_max_rate_;

    // Security
    ros::Time last_control_time_;
    static constexpr double MIN_CONTROL_INTERVAL_SEC = 1.0;

    // Rust FFI filter handles (managed via RAII helpers)
    TemporalFilter* temporal_filter_;
    HotPixelFilter* hot_pixel_filter_;

    // Camera info
    uint16_t dvs_size_x_;
    uint16_t dvs_size_y_;
};

#endif  // EDVS_DRIVER_HPP
