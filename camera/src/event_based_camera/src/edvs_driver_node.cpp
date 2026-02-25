#include <ros/ros.h>
#include "event_based_camera/edvs_driver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "edvs_camera");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        EdvsDriver driver(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL("eDVS driver failed: %s", e.what());
        return 1;
    }

    return 0;
}
