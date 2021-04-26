#pragma once
#include <sensor_msgs/LaserScan.h>
#include <tuple>

namespace husky_highlevel_controller {
    class Algorithm {
        public:
            Algorithm();
            virtual ~Algorithm();
            std::tuple<double, int> getMinimalDistance(const sensor_msgs::LaserScan& msg);
            sensor_msgs::LaserScan createNewLaserScan(const sensor_msgs::LaserScan last_scan, int minIndex, double minValue);
        private:
    };
} // end namespace