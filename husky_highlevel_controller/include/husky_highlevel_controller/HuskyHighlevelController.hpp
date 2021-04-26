#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace husky_highlevel_controller {
    class HuskyHighlevelController {
        public:
            HuskyHighlevelController(ros::NodeHandle& nodeHandle);
            virtual ~HuskyHighlevelController();

        private:
            bool readParameters();

            void laserScanCallback(const sensor_msgs::LaserScan& msg);
            void publishLaserScan(const sensor_msgs::LaserScan& msg, int minIndex, double minValue);

            //! ROS node handle
            ros::NodeHandle& nodeHandle_;

            //! ROS topic subscriber
            ros::Subscriber subscriber_;

            // scan publisher
            ros::Publisher publisher_;

            // scan publisher name
            std::string scan_publisher_;

            //! ROS topic name to subscribe to
            std::string subscriberTopic_;

            // size for message queue
            int queueSize_;

            //! ROS service server
            ros::ServiceServer serviceServer_;

            //! Algorithm computation object
            Algorithm algorithm_;
    }; // end class
} // end namespace