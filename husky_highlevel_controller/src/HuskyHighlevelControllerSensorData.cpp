#include "husky_highlevel_controller/HuskyHighlevelControllerSensorData.hpp"
#include "husky_highlevel_controller_msgs/TargetPose.h"

namespace husky_highlevel_controller {
    HuskyHighlevelControllerSensorData::HuskyHighlevelControllerSensorData(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle) {
        ROS_INFO("HuskyHighlevelControllerSensorData object was created!");
        if(! readParameters()) {
            ROS_ERROR("Could not read parameters");
            ros::requestShutdown(); 
        }
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighlevelControllerSensorData::laserScanCallback, this);
        publisher_sensor_data = nodeHandle_.advertise<husky_highlevel_controller_msgs::TargetPose>(sensor_data_publisher_, queueSize_);

        ROS_INFO("Successfully launched node for Husky Highlevel Controller Sensor Data");
    }

    HuskyHighlevelControllerSensorData::~HuskyHighlevelControllerSensorData() {  }

    bool HuskyHighlevelControllerSensorData::readParameters() {
        if (!nodeHandle_.getParam("laserscan_topic", subscriberTopic_)) return false;
        if (!nodeHandle_.getParam("queueSize", queueSize_)) return false;
        if (!nodeHandle_.getParam("sensor_data_topic", sensor_data_publisher_)) return false;
        return true;
    }

    void HuskyHighlevelControllerSensorData::laserScanCallback(const sensor_msgs::LaserScan& msg) {
        double minValue;
        int minIndex;
        std::tie(minValue, minIndex) = algorithm_.getMinimalDistance(msg);
        
        //ROS_INFO("Minimial distance from algorithm: %lf", minValue);

        double targetAngle = algorithm_.calculateTargetPosition(msg, minIndex);
        this->publishMovementData(minValue, targetAngle);
    }

    void HuskyHighlevelControllerSensorData::publishMovementData(const double distance, const double targetAngle) {
        husky_highlevel_controller_msgs::TargetPose targetPose;
        targetPose.distance = distance;
        targetPose.angle = targetAngle;

       publisher_sensor_data.publish(targetPose); 
    }
} // end namespace