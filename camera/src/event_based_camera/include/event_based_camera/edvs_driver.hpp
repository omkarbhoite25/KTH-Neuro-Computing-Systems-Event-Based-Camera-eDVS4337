#ifndef EDVS_DRIVER_HPP
#define EDVS_DRIVER_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ros/ros.h>

#include "event_based_camera/Control.h"
#include "event_based_camera/event_types.hpp"
#include "event_based_camera/evt_camera.hpp"
#include "event_based_camera/filter_pipeline.hpp"

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

    // Device (abstracted behind EvtCamera interface)
    std::mutex device_mutex_;  // Guards start/stop state transitions
    std::unique_ptr<EvtCamera> device_;
    std::atomic<bool> running_{false};
    std::thread readout_thread_;

    // ROS
    ros::Subscriber control_sub_;
    ros::Publisher event_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Parameters
    std::string serial_port_;
    FilterPipeline::Config filter_config_;

    // Security
    ros::WallTime last_control_time_;
    static constexpr double MIN_CONTROL_INTERVAL_SEC = 1.0;

    // Rust FFI filter pipeline (RAII-managed)
    FilterPipeline pipeline_;

    // Camera info
    uint16_t dvs_size_x_;
    uint16_t dvs_size_y_;
};

#endif  // EDVS_DRIVER_HPP
