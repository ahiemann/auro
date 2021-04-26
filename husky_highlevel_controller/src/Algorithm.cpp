#include "husky_highlevel_controller/Algorithm.hpp"
#include <string.h>
#include <ros/ros.h>

namespace husky_highlevel_controller {
    Algorithm::Algorithm() {

    }

    Algorithm::~Algorithm() = default;

    double Algorithm::getMinimalDistance(const sensor_msgs::LaserScan& msg) {
        double min = msg.range_max;
        for (int i = 0; i < msg.ranges.size(); ++i) {
            if (msg.ranges[i] < min)
                min = msg.ranges[i];
        }
        ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
        return min;
    }
}