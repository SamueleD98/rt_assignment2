#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "second_assignment/Velocity.h"

ros::ServiceClient change_speed;

int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "bot_gui");  
	ros::NodeHandle nh;
		
	ros::ServiceClient reset =  nh.serviceClient<std_srvs::Empty>("/reset_positions");
	std_srvs::Empty rst;		
	
	change_speed = nh.serviceClient<second_assignment::Velocity>("/velocity");
	
	ROS_INFO("\n\n\nWelcome, please type:\n	'r' to reset the position,\n	'w' to increase the speed,\n	's' to decrease the speed.\n");
	
	second_assignment::Velocity vel;	
	
	while (ros::ok()) 
	{	
	
		std::string inputString1;
		std::getline(std::cin, inputString1);
		
		int cmd=0;

		if (inputString1 == "r") 
		{
			reset.call(rst);
			ROS_INFO("Position resetted!");
		}else{
			if (inputString1 == "s") cmd = -1;
			if (inputString1 == "w") cmd = 1;		
		
			vel.request.command = cmd;
			
			change_speed.call(vel);
			
			if(vel.response.resp){
				ROS_INFO("Speed changed!");
			}else{
				ROS_INFO("Error, please type again.");
			}
		}

	}
	
	ros::spin();
	return 0;
}











