#pragma once

#include "husky_highlevel_controller/Algorithm.hpp"
#include "husky_highlevel_controller_msgs/TargetPose.h"
#include "husky_highlevel_controller/MoveUntilDistanceAction.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>


typedef actionlib::SimpleActionServer<husky_highlevel_controller::MoveUntilDistanceAction> ActionServer;

namespace husky_highlevel_controller {
    class HuskyHighlevelController {
        public:
            HuskyHighlevelController(ros::NodeHandle& nodeHandle);
            virtual ~HuskyHighlevelController();

        private:
            bool readParameters();

            void sensorDataCallback(const husky_highlevel_controller_msgs::TargetPose& msg);
            void driveCallback(const husky_highlevel_controller::MoveUntilDistanceGoalConstPtr &goal);
            void publishLaserScan(const sensor_msgs::LaserScan& msg, int minIndex, double minValue);

            // Action Server
            ActionServer actionServer_;

            //! ROS node handle
            ros::NodeHandle& nodeHandle_;

            //! ROS topic subscriber
            ros::Subscriber subscriber_;

            // targetMarker publisher
            ros::Publisher publisher_targetMarker;

            // ROS transform buffer
            tf2_ros::Buffer tfBuffer_;

            // ROS transform listener
            tf2_ros::TransformListener tfListener_;

            // targetMarker publisher name
            std::string target_marker_publisher_;
            
            // cmd_vel publisher
            ros::Publisher publisher_cmd_vel;

            // cmd_vel publisher name
            std::string cmd_vel_publisher_;

            // proportional factor for p-controller
            float proportionalFactor_;

            // scan publisher
            ros::Publisher publisher_scan;

            // scan publisher name
            std::string scan_publisher_;

            //! ROS topic name to subscribe to
            std::string sensorDataTopic_;

            // size for message queue
            int queueSize_;

            //! ROS service server
            ros::ServiceServer serviceServer_;

            //! Algorithm computation object
            Algorithm algorithm_;
    }; // end class
} // end namespac