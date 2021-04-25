#include "husky_highlevel_controller/Algorithm.hpp"

namespace husky_highlevel_controller {
    Algorithm::Algorithm() {

    }

    Algorithm::~Algorithm() = default;

    float Algorithm::getMinimalDistance(const sensor_msgs::LaserScan& msg) {
        return msg.range_min;
    }
}