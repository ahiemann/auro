#pragma once
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {
    class Algorithm {
        public:
            Algorithm();
            virtual ~Algorithm();
            float getMinimalDistance(const sensor_msgs::LaserScan& msg);
        private:
    };
} // end namespace