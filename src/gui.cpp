#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "rt_assignment2/Velocity.h"


int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "bot_gui");  
	ros::NodeHandle nh;
		
	// Setup the service client for the resetting of the robot position	
	ros::ServiceClient reset =  nh.serviceClient<std_srvs::Empty>("/reset_positions");
	// Declaring the argument of the reset call. The type is given by "rosservice info /reset_positions"
	std_srvs::Empty rst;		
	
	// Setup the service client for the increase and decrease of the robot speed
	ros::ServiceClient change_speed = nh.serviceClient<rt_assignment2::Velocity>("/velocity");
	// Declaring the argument of the change speed call
	rt_assignment2::Velocity vel;
	
	ROS_INFO("\n\n\nWelcome, please type:\n	'r' to reset the position,\n	'w' to increase the speed,\n	's' to decrease the speed,\n	't' to toggle the helper,\n	'x' to kill the gui node and so the simulation.\n");
	
	//Declaring the input string
	char inputString1;
	
	// The node needs to be ready to receive input until the node is running	
	while (ros::ok()) 
	{	
		
		//Writing the input string
		std::cin >> inputString1;		
		
		switch (inputString1) {
			case 'r':
				// Calling the service
				reset.call(rst);
				ROS_INFO("Position resetted!");			
				break;
				
			case 'w':
				// Setting the command to increase the speed
				vel.request.command = 1;
				// Calling the service
				change_speed.call(vel);
				// Analyzing the response
				ROS_INFO("Speed increased to: %.1f",
					vel.response.new_vel); 
				if(!vel.response.resp)		ROS_INFO("Warning, speed too high.");					
				break;
				
			case 's':
				// Setting the command to increase the speed
				vel.request.command = -1;
				// Calling the service
				change_speed.call(vel);
				// Analyzing the response
				if(vel.response.resp){
					ROS_INFO("Speed decreased to: %.1f",
					vel.response.new_vel); 
				}else{
					ROS_INFO("Error, the bot is not moving.");
				}
				break;
			case 't':
				vel.request.toggle_helper= 1;
				vel.request.command = 0;
				change_speed.call(vel);
				if(vel.response.resp){
					ROS_INFO("Helper activated!");
				}else{
					ROS_INFO("Helper deactivated!");
				}
				break;
				
			case 'x':
				ROS_INFO("Goodbye!");
				return 0;
				
			default:
				ROS_INFO("Wrong character, please type again.");		
		}
		
		// cin should ignore every other char until it finds a new line
		std::cin.ignore(1000,'\n');
	}
	
	ros::spin();
	return 0;
}











