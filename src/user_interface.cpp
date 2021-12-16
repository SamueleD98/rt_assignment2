#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "rt_assignment2/Command.h"


int main (int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system  
	ros::init(argc, argv, "bot_ui");  
	ros::NodeHandle nh;
		
	// Setup the service client for the resetting of the robot position	
	ros::ServiceClient reset =  nh.serviceClient<std_srvs::Empty>("/reset_positions");
	// Declaring the argument of the reset call. The type is given by "rosservice info /reset_positions"
	std_srvs::Empty rst;		
	
	// Setup the service client for the transmission of commands to the controller
	ros::ServiceClient new_command = nh.serviceClient<rt_assignment2::Command>("/command");
	// Declaring the argument of the command call
	rt_assignment2::Command cmd;
	
	ROS_INFO("\n\n\nWelcome, please type:\n	' r ' to reset the position,\n	' w ' to increase the speed,\n	' s ' to decrease the speed,\n	' t ' to toggle the helper,\n	' x ' to kill the gui node and so the simulation.\n\n");
	ROS_INFO("Remember the speed is now 0.0 and the helper is not active\n");
	
	//Declaring the input string
	char inputString;
	
	float linear_vel = 0;
	
	// The node needs to be ready to receive input until the node is running	
	while (ros::ok()) 
	{		
		//Writing the input string
		std::cin >> inputString;		
		
		switch (inputString) {
			case 'r':
				// Calling the service
				reset.call(rst);
				ROS_INFO("Position resetted!");			
				break;
				
			case 's':
				if (linear_vel == 0.0)
				{
					ROS_INFO("Error, the bot is not moving.");
					
				}else{
					// Setting the command to decrease the speed
					cmd.request.command = 1;
					// Calling the service
					new_command.call(cmd);
					linear_vel = cmd.response.new_vel;
					
					ROS_INFO("Speed decreased to: %.1f",
						linear_vel); 
					if(linear_vel >= 2.0)
					{
						ROS_INFO("Warning, speed too high.");	
					}
				}
				break;
				
			case 'w':
				// Setting the command to increase the speed
				cmd.request.command = 2;
				// Calling the service
				new_command.call(cmd);
				linear_vel = cmd.response.new_vel;
					
				ROS_INFO("Speed increased to: %.1f",
					linear_vel); 
				if(linear_vel >= 2.0)
				{
					ROS_INFO("Warning, speed too high.");	
				}			
				break;
				
			case 't':
				cmd.request.command = 3;
				new_command.call(cmd);
				if(cmd.response.helper_status){
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











