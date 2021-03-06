#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"
#include <ros/ros.h>

namespace husky_highlevel_controller
{
    class HuskyHighlevelControllerSensorData {
        public:
            HuskyHighlevelControllerSensorData(ros::NodeHandle& nodeHandle);
            virtual ~HuskyHighlevelControllerSensorData();

        private:
            bool readParameters();

            void laserScanCallback(const sensor_msgs::LaserScan& msg);
            void publishMovementData(const double distance, const double targetAngle);

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
