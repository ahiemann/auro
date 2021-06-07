#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "husky_highlevel_controller/MoveUntilDistanceAction.h"


int main (int argc, char **argv)
{
  ros::init(argc, argv, "husky_highlevel_controller_action_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<husky_highlevel_controller::MoveUntilDistanceAction> actionClient_("move_until_distance_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  actionClient_.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  husky_highlevel_controller::MoveUntilDistanceGoal goal;
  goal.distance = 1;
  actionClient_.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = actionClient_.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = actionClient_.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}