#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    if(client.call(srv))
    {
        ROS_INFO("Service /ball_chaser/command_robot Called L:%f A:%f",lin_x,ang_z); //---Check why srv.response.msg_feedback not working
//        ROS_INFO(srv.response.msg_feedback);
    }
    else ROS_ERROR("Failed to call /ball_chaser/command_robot service");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int desired_pixel = 255;
    int spotted_counter = 0;
    //Looping through each pixel in the image
    for(int i=0;i<img.height;i++)
    {
	for(int j=0;j<img.step;j++)
        {
            if(img.data[(i*img.step)+j]==desired_pixel) //to check if any of rbg value is 255
            {
                if(img.data[(i*img.step)+j+1] == desired_pixel && img.data[(i*img.step)+j+2] == desired_pixel) //to check if value of it's successive ones are 255 too.
                {
                    spotted_counter++;
//                  ROS_INFO("White Spotted");
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
                    if(j<=(img.step/3))
                    {
                        drive_robot(0.0,1.0); //left turn
                        goto TIME_TO_UPATE;
                    }
                    if(j>(img.step/3) && j<(2*img.step/3))
                    {
                        drive_robot(5.0,0.0); //forward move
                        goto TIME_TO_UPATE;
                    }
                    if(j>=(2*img.step/3))
                    {
                        drive_robot(0.0,-1.0); //right turn
                        goto TIME_TO_UPATE;
                    }   
                }            
            }
        }
    }
    if(spotted_counter == 0)
    {
        drive_robot(0.0,0.0); //to stop if no white found
    }
    TIME_TO_UPATE: ; //updates image by breaking out of loop if white is found.
//    ROS_INFO("============IMAGE UPDATED===========");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service to request from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic n
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    ros::spin();

    return 0;
}
