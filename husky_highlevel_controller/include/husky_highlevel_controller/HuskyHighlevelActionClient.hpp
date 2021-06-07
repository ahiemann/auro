#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"
#include <ros/ros.h>

#include "husky_highlevel_controller/MoveUntilDistanceAction.h"

/*
namespace husky_highlevel_controller
{
    class HuskyHighlevelControllerActionClient {
        public:
            HuskyHighlevelControllerActionClient(ros::NodeHandle& nodeHandle);
            virtual ~HuskyHighlevelControllerActionClient();

        private:
            bool readParameters();

            void laserScanCallback(const sensor_msgs::LaserScan& msg);
            void publishMovementData(const double distance, const float targetAngle);

            //! ROS node handle
            ros::NodeHandle& nodeHandle_;

            //! ROS topic subscriber
            ros::Subscriber subscriber_;

            // sensor data publisher
            ros::Publisher publisher_sensor_data;

            // sensor data publisher name
            std::string sensor_data_publisher_;

            //! ROS topic name to subscribe to
            std::string subscriberTopic_;

            // size for message queue
            int queueSize_;

            //! Algorithm computation object
            Algorithm algorithm_;
    };
} // namespace name
*/