#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void scanCallback(const sensor_msgs::LaserScan& msg) {
    ROS_INFO("Minimum laser scan range: [%f]", msg.range_min);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nodeHandle("~");

    std::string topic;
    int queueSize;
    if (!nodeHandle.getParam("topic", topic) || !nodeHandle.getParam("queueSize", queueSize)) {
        ROS_ERROR("Could not get parameter");
    }
    ros::Subscriber subscriber = nodeHandle.subscribe(topic, queueSize, scanCallback);

    ros::spin();

    return 0;
}