#include "husky_highlevel_controller/Algorithm.hpp"
#include <string.h>
#include <ros/ros.h>
#include <tuple>
#include <vector>

namespace husky_highlevel_controller {
    Algorithm::Algorithm() {

    }

    Algorithm::~Algorithm() = default;

    std::tuple<double, int> Algorithm::getMinimalDistance(const sensor_msgs::LaserScan& msg) {
        double min = msg.range_max;
        int minIndex = -1;
        for (int i = 0; i < msg.ranges.size(); ++i) {
            if (msg.ranges[i] < min) {
                min = msg.ranges[i];
                minIndex = i;   
            }
        }
        /*
        if (min < 1) {
            ROS_WARN("Minimal distance below 1: %lf", min);
        }
        */

        //ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
        return std::tuple<double, int>(min, minIndex);
    }

    sensor_msgs::LaserScan Algorithm::createNewLaserScan(const sensor_msgs::LaserScan last_scan, int minIndex, double minValue) {
        sensor_msgs::LaserScan new_scan;

        int startIndex, stopIndex;
        std::vector<float> new_ranges;
        for (int i = 1; i <= 2; i++) {
            int index = minIndex - i;
            if (index < 0) {
                new_ranges.push_back(999.0);
            }
            else {
                new_ranges.push_back(last_scan.ranges[index]);
            }
            startIndex = index;
        }
        new_ranges.push_back(minValue);
        for (int i = 1; i <= 2; i++) {
            int index = minIndex + i;
            if (index > last_scan.ranges.size()) {
                new_ranges.push_back(999.0);
            }
            else {
                new_ranges.push_back(last_scan.ranges[index]);
            }
            stopIndex = index;
        }

        new_scan.ranges = new_ranges;
        new_scan.angle_increment = last_scan.angle_increment;

        float new_angle_min = last_scan.angle_min;
        for (int i = 0; i < startIndex; i++) {
            new_angle_min += new_scan.angle_increment;
        }
        float new_angle_max = last_scan.angle_max;
        for (int i = last_scan.ranges.size(); i < stopIndex; i--) {
            new_angle_max -= new_scan.angle_increment;
        }

        new_scan.angle_min = new_angle_min;
        new_scan.angle_max = new_angle_max;

        return new_scan;
    }

    float Algorithm::calculateTargetPosition(const sensor_msgs::LaserScan laserScan, const int minIndex){
        return laserScan.angle_min + (laserScan.angle_increment * minIndex);
    }

}