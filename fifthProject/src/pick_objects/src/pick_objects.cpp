#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// external Sources/references : https://edu.gaitech.hk/turtlebot/map-navigation.html and udacity knowledge forum.
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	// Initialize the simple_navigation_goals node
	ros::init(argc, argv, "pick_objects");

	// Parameter server is very usefull is this project because it can be used by all nodes to have a check variable to know when robot has reached 
	// a particular pose. (in comparison to other methods)
	// got this advice from udacity knowledge forum 
	ros::NodeHandle nh;
	nh.setParam("is_at_1st_pose", false);
	nh.setParam("is_at_2nd_pose", false);

	// tell the action client "move_base" that we want to spin a thread by default
	MoveBaseClient ac("move_base", true);

	// Wait 5 sec for move_base action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	// set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// GO TO PICK-UP LOCATION
	goal.target_pose.pose.position.x = 4.0;
	goal.target_pose.pose.position.y = -0.5; 
	goal.target_pose.pose.orientation.w = 1.0;
	goal.target_pose.pose.orientation.z = 0.0;

	// Send the goal position and orientation for the robot to reach
	ROS_INFO("pick-up pose sent");
	ac.sendGoal(goal);

	// Wait an infinite time for the results
	ac.waitForResult();

	// Check if the robot reached its goal
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Reached pickup zone");
		nh.setParam("is_at_1st_pose", true);
	}
	else
	{
		ROS_WARN_ONCE("Failed to reach pick up zone!");
	}

	ros::Duration(5).sleep(); // sleep for 5 seconds

	// GO TO DROP-OFF LOCATION
	goal.target_pose.pose.position.x = -1.0;
	goal.target_pose.pose.position.y = 3.0; 
	goal.target_pose.pose.orientation.w = 0.0;
	goal.target_pose.pose.orientation.z = 1.0;

	// Send the goal position and orientation for the robot to reach
	ROS_INFO("drop of pose sent");
	ac.sendGoal(goal);

	// Wait an infinite time for the results (blocking)
	ac.waitForResult();

	// Check if the robot reached its goal
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Reached drop-off zone");
		nh.setParam("is_at_2nd_pose", true);
	}
	else
	{
		ROS_INFO("Failed to reach drop off zone!");
	}

	while (1)
	{
		if (!ros::ok())
           		return 0;
  	}

}
