#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res)
{
	
        geometry_msgs::Twist robo_vel;
        robo_vel.linear.x = req.linear_x;
        robo_vel.angular.z = req.angular_z;
        motor_command_publisher.publish(robo_vel);
	res.msg_feedback="Velocity given to chassis: Linear ="+ std::to_string(robo_vel.linear.x) + " Angular ="+ std::to_string(robo_vel.angular.z);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_bot");

    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the /cmd_vel topic
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    ros::spin();

    return 0;
}
