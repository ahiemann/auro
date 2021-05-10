#include "husky_highlevel_controller/HuskyHighlevelController.hpp"


namespace husky_highlevel_controller {
    HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) : nodeHandle_(nodeHandle), tfListener_(tfBuffer_) {
        if(! readParameters()) {
            ROS_ERROR("Could not read parameters");
            ros::requestShutdown();
        }
        subscriber_ = nodeHandle_.subscribe(subscriberTopic_, queueSize_, &HuskyHighlevelController::laserScanCallback, this);
        publisher_scan = nodeHandle_.advertise<sensor_msgs::LaserScan>(scan_publisher_, queueSize_);
        publisher_cmd_vel = nodeHandle_.advertise<geometry_msgs::Twist>(cmd_vel_publisher_, queueSize_);
        publisher_targetMarker = nodeHandle_.advertise<visualization_msgs::Marker>(target_marker_publisher_, 1);

        ROS_INFO("Successfully launched node for Husky Highlevel Controller");
    }

    HuskyHighlevelController::~HuskyHighlevelController() {  }

    bool HuskyHighlevelController::readParameters() {
        if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
        if (!nodeHandle_.getParam("queueSize", queueSize_)) return false;
        if (!nodeHandle_.getParam("scan_publisher", scan_publisher_)) return false;
        if (!nodeHandle_.getParam("cmd_vel_publisher", cmd_vel_publisher_)) return false;
        if (!nodeHandle_.getParam("target_marker_publisher", target_marker_publisher_)) return false;
        if (!nodeHandle_.getParam("proportional_factor", proportionalFactor_)) return false;
        return true;
    }

    void HuskyHighlevelController::laserScanCallback(const sensor_msgs::LaserScan& msg) {
        double minValue;
        int minIndex;
        std::tie(minValue, minIndex) = algorithm_.getMinimalDistance(msg);
        this->publishLaserScan(msg, minIndex, minValue);
        ROS_INFO("Minimum laser scan range: [%lf]" , minValue);

        float targetAngle = algorithm_.calculateTargetPosition(msg, minIndex);
        
        geometry_msgs::Twist cmd_vel;


        // control robot to the targetAngle
        if (minValue > 1) {
            cmd_vel.linear.x = 1;
            cmd_vel.angular.z = proportionalFactor_ * (0 - targetAngle);
            ROS_INFO("Angle :[%lf]", cmd_vel.angular.z );
        } 
        else {
            cmd_vel.linear.x = 0; 
            cmd_vel.angular.z = 0; 
        }

        publisher_cmd_vel.publish(cmd_vel);

        // TargetAngle Marker - Rviz
        visualization_msgs::Marker targetMarker;
        targetMarker.header.frame_id = "base_link";
        targetMarker.header.stamp = ros::Time();

        targetMarker.ns = "target";
        targetMarker.id = 0;

        targetMarker.type = visualization_msgs::Marker::CYLINDER;
        targetMarker.action = visualization_msgs::Marker::ADD;

        targetMarker.pose.position.x = (minValue + 1) * cos(targetAngle);
        targetMarker.pose.position.y = -(minValue) * sin(targetAngle);
        targetMarker.pose.position.z = 0;

        targetMarker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

        targetMarker.scale.x = 1;
        targetMarker.scale.y = 1;
        targetMarker.scale.z = 1;
        
        targetMarker.color.a = 1.0;
        targetMarker.color.r = 0.0;
        targetMarker.color.g = 0.0;
        targetMarker.color.b = 1.0;

        //Aufgabe 7a)
        //publisher_targetMarker.publish(targetMarker);

        geometry_msgs::TransformStamped transformStamped;

        try{
          transformStamped = tfBuffer_.lookupTransform("odom", "base_laser", ros::Time(0));
          targetMarker.header.frame_id = "odom";
          tf2::doTransform(targetMarker.pose, targetMarker.pose, transformStamped);
    
          publisher_targetMarker.publish(targetMarker);
        }
        catch (tf2::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          return;
        }
    }

    void HuskyHighlevelController::publishLaserScan(const sensor_msgs::LaserScan& last_scan, int minIndex, double minValue) {
        sensor_msgs::LaserScan new_scan;

        if (minIndex == -1) {
            return;
        }
        new_scan = algorithm_.createNewLaserScan(last_scan, minIndex, minValue);

        // Publish new LaserScan object
        publisher_scan.publish(new_scan);
    }
} // end namespace