
#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelControllerSensorData.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "husky_highlevel_controller_sensor_data");
    ros::NodeHandle nodeHandle("~");

    ROS_INFO("Before creating HuskyHighlevelControllerSensorData object");
    husky_highlevel_controller::HuskyHighlevelControllerSensorData huskyHighlevelControllerSensorData(nodeHandle);
    ROS_INFO("After creating HuskyHighlevelControllerSensorData object");

    ros::spin();
    return 0;
}