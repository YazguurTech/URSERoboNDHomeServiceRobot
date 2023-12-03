#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 4.75;
  goal.target_pose.pose.position.y = -6.25;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up goal");
  ac.sendGoal(goal);
  
  ROS_INFO("The robot is moving to the pick-up zone.");
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot has reached the pick-up zone.");
    ROS_INFO("The robot is picking up a package.");
    ros::Duration(5.0).sleep();
    ROS_INFO("The packge is picked up.");
  } else {
    ROS_INFO("The base failed to move to the pick-up zone for some reason");
    ros::Duration(5.0).sleep();
    return 0;
  }

  goal.target_pose.pose.position.x = -3.5;
  goal.target_pose.pose.position.y = 5.8;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending drop-zone goal");
  ac.sendGoal(goal);

  ROS_INFO("The robot is going to the drop-off zone to delivery the package.");

  // Wait an infinite time for the results
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot has reached the drop-off zone.");
    ROS_INFO("The robot is dropping off the package.");
    ros::Duration(5.0).sleep();
    ROS_INFO("The package is delivered.");
  } else {
    ROS_INFO("The base failed to move to the drop-off zone for some reason");
  }
  
  ros::Duration(5.0).sleep();
  
  return 0;
}
