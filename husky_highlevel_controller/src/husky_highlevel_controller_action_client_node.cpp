#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include "husky_highlevel_controller/MoveUntilDistanceAction.h"


actionlib::SimpleActionClient<husky_highlevel_controller::MoveUntilDistanceAction>* actionClient_;


void activeCallback() {
  ROS_INFO("Goal was activated");
}

void doneCallback(const actionlib::SimpleClientGoalState& state, const husky_highlevel_controller::MoveUntilDistanceResultConstPtr result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Final distance: %lf", result->resultDistance);
}

void feedbackCB(const husky_highlevel_controller::MoveUntilDistanceFeedbackConstPtr& feedback) {
  double remainingDistance = feedback->feedbackDistance;
  ROS_INFO("Feedback received, remaining distance is %lf", remainingDistance);

  // Just for demonstration of action cancellation
  if ((int)remainingDistance < 3) {
    ROS_WARN("Remaining distance below 3.0, will cancel action now");
    actionClient_->cancelAllGoals();
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "husky_highlevel_controller_action_client");

  actionClient_ = new actionlib::SimpleActionClient<husky_highlevel_controller::MoveUntilDistanceAction>("move_until_distance_action", true);

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<husky_highlevel_controller::MoveUntilDistanceAction> actionClient_("move_until_distance_action", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  actionClient_.waitForServer(); // will wait for infinite time
  ROS_INFO("Action server started, sending goal.");

  // send a goal to the action
  husky_highlevel_controller::MoveUntilDistanceGoal goal;
  goal.distance = 1;
  actionClient_.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCB);

  ros::spin();

  //exit  
  return 0;
}