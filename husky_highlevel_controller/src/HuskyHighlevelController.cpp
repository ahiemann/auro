#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

namespace husky_highlevel_controller {
    HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
        if(! readParameters()) {
            ROS_ERROR("Could not read parameters");
            ros::requestShutdown();
        }
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighlevelController::laserScanCallback, this);
        publisher_ = nodeHandle_.advertise<sensor_msgs::LaserScan>(scan_publisher_, queueSize_);

        ROS_INFO("Successfully launched node for Husky Highlevel Controller");
    }

    HuskyHighlevelController::~HuskyHighlevelController() {
        
    }

    bool HuskyHighlevelController::readParameters() {
        if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
        if (!nodeHandle_.getParam("queueSize", queueSize_)) return false;
        if (!nodeHandle_.getParam("scan_publisher", scan_publisher_)) return false;
        return true;
    }

    void HuskyHighlevelController::laserScanCallback(const sensor_msgs::LaserScan& msg) {
        double minValue;
        int minIndex;
        std::tie(minValue, minIndex) = algorithm_.getMinimalDistance(msg);
        this->publishLaserScan(msg, minIndex, minValue);
        ROS_INFO("Minimum laser scan range: [%lf]" , minValue);
    }

    void HuskyHighlevelController::publishLaserScan(const sensor_msgs::LaserScan& last_scan, int minIndex, double minValue) {
        sensor_msgs::LaserScan new_scan;

        if (minIndex == -1) {
            return;
        }
        new_scan = algorithm_.createNewLaserScan(last_scan, minIndex, minValue);
        

        // Publish new LaserScan object
        publisher_.publish(new_scan);
    }
} // end namespace