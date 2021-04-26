#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {
    HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
        if(! readParameters()) {
            ROS_ERROR("Could not read parameters");
            ros::requestShutdown();
        }
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighlevelController::laserScanCallback, this);

        ROS_INFO("Successfully launched node for Husky Highlevel Controller");
    }

    HuskyHighlevelController::~HuskyHighlevelController() {
        
    }

    bool HuskyHighlevelController::readParameters() {
        if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
        if (!nodeHandle_.getParam("queueSize", queueSize_)) return false;
        return true;
    }

    void HuskyHighlevelController::laserScanCallback(const sensor_msgs::LaserScan& msg) {
        ROS_INFO("Minimum laser scan range: [%lf]" , algorithm_.getMinimalDistance(msg));
    }
} // end namespace