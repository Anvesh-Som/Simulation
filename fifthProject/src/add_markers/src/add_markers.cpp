//External sources:- https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes and udacity knowledge forum

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	bool is_at_1st_pose = false, is_at_2nd_pose = false;

	// Set shape type to be a cube
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map"; //our fixed frame
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "add_markers";
	marker.id = 0;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 4;
	marker.pose.position.y = -0.5;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.3;
	marker.scale.y = 0.3;
	marker.scale.z = 0.01;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	while (ros::ok())
	{
		
		// Set the frame ID and timestamp
		marker.header.stamp = ros::Time::now();

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;
		ROS_INFO("Marker Added");
		marker.lifetime = ros::Duration();

		// Publishishing marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(0.5);
		}
		marker_pub.publish(marker);
		
		r.sleep();

		//ros::Duration(5).sleep(); // sleep for 5 seconds
		n.getParam("is_at_1st_pose", is_at_1st_pose); //to know when robot reached first given pose.
		if (is_at_1st_pose) //breaks when robot reached 1st pose
			break;
	}




	// Now to delete virtual object
	ROS_INFO("Marker Deleted");
	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker); //published new status

	while (true)   // waits till robot reached seacond pose
	{
		n.getParam("is_at_2nd_pose", is_at_2nd_pose);
		if (is_at_2nd_pose)
			break;
	}

	// adding new marker at the destination
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	// poses are kept same as pick_objects node
	marker.pose.position.x = -1;
	marker.pose.position.y = 3;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	ROS_INFO("Virtual object added at destination");
	while (ros::ok())
	{
		
		// Set the frame ID and timestamp
		marker.header.stamp = ros::Time::now();

		// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		marker.action = visualization_msgs::Marker::ADD;

		
		marker.lifetime = ros::Duration();

		// Publish the marker
		while (marker_pub.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				return 0;
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		marker_pub.publish(marker);
		
		r.sleep();
	}  // end of while
}
