#include "ros/ros.h"
#include <iostream>
#include <math.h>   	// std::abs
#include <algorithm>    // std::min

#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h"
#include "rt_assignment2/Command.h"

ros::Publisher pub;

geometry_msgs::Twist my_vel;

float target_vel_linear_x = 0.0;//Desired linear velocity value
float vel_linear_x = 0.0;	//Actual linear velocity value
bool helper_status = false;	//Helper status (turned on/off)
int number_of_sectors = 13;	//Number of subsections of the ranges vector

//Functions declarations (definitions after the main)
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void discretize_fov(const sensor_msgs::LaserScan::ConstPtr& msg, float * m, int number_of_sectors);	//Function to discretize the Field Of View. More info in the ReadMe.
void take_action(float * m);										//Function that change angular e linear* velocity. *(Only if the "Helper" is active) 
bool commandService(rt_assignment2::Command::Request &req, rt_assignment2::Command::Response &res);	//Function that execute the user command given by the service client: the user_interface node
float angular(float difference, int factor);								//Function that computes the angular velocity as proportional to a given factor

int main (int argc, char **argv)
{
	ros::init(argc, argv, "bot_controller");  
	ros::NodeHandle nh;
	
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);				//To change the velocity
	
	ros::Subscriber sub = nh.subscribe("/base_scan", 1, laserCallback);		//To see the obstacles
	
	ros::ServiceServer command = nh.advertiseService("/command", commandService);	//To receive commands from user_interface node
	
	ros::spin();
	return 0;
}

void take_action(float * m)
	{
		float diff = *(m+1) - *(m+3);	//The difference between opposite subsections
		if (-0.3 < diff < 0.3) 		//If those are similar
		{	
			diff = *(m+0) - *(m+4);	//Take the external subsections
			if (-0.3 < diff < 0.3)	//If even those are similar
			{
				my_vel.linear.x = 0.1;	//Reduce a lot the speed
				vel_linear_x = my_vel.linear.x;	//Update the actual speed
				my_vel.angular.z = 100;	//and turn
				pub.publish(my_vel);	
				return;
			}
		}
		
		my_vel.linear.x = target_vel_linear_x;	//Take the target speed, if the helper is not active the speed won't change
		
		int turning_factor;
		
		if (*(m+2) < 1.5) //Very very close obstacle
		{
			if (helper_status && my_vel.linear.x >= 2.0) my_vel.linear.x = 0.5;	//Reduce the speed if the "Helper" is active			
			turning_factor = 100;
		
		}else if (*(m+2) < 2.0)	//Very close obstacle
		{
			if (helper_status && my_vel.linear.x >= 2.0) my_vel.linear.x = 1.0;
			turning_factor = 75;
					
		}else if (*(m+2) < 2.5) //Close obstacle
		{
			if (helper_status && my_vel.linear.x >= 2.0) my_vel.linear.x = 1.5;
			turning_factor = 50;

		}else if ((vel_linear_x + 1.0) < target_vel_linear_x )				//If the actual speed is far from the targeted one then the bot has just avoided an obstacle
		{
			if (helper_status) my_vel.linear.x = vel_linear_x + 1;			//If the helper is active slowly increase the speed
			turning_factor = 50;							//Keep turning 
		}else{	
			//No obstacles in front of the bot
			turning_factor = 1;	//Adjust very slowly the robot direction
		} 	
		
		my_vel.angular.z = angular(diff, turning_factor);	//Turn in the best direction
		
		vel_linear_x = my_vel.linear.x;	
		pub.publish(my_vel);	
	}

bool commandService(rt_assignment2::Command::Request &req, rt_assignment2::Command::Response &res)
	{
		switch (req.command)  
		{		
			case 1:	//Decreasing speed
				target_vel_linear_x = target_vel_linear_x - 0.5;
				break;
			case 2:	//Increasing speed
				target_vel_linear_x = target_vel_linear_x + 0.5;	
				break;		
			case 3:	//Toggling the helper
				helper_status = !helper_status;
				break;
		}
		
		//Filling the response msg
		res.helper_status = helper_status;	//Set the helper status, if not modified it will be the same
		res.new_vel = target_vel_linear_x;		//Set the speed, 	 if not modified it will be the same
		
		//Publishing the new velocity only if the helper is not active.
		//If it is active, the take_action function will modify the linear velocity according to the desired one only when possible.
		if(!helper_status)
		{
			my_vel.linear.x = target_vel_linear_x;
			vel_linear_x = my_vel.linear.x;
			pub.publish(my_vel);
		}		
		return true;
	}  
	
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{		
		float min[5] = {10, 10, 10, 10, 10};		
		float * m = min;
		
		discretize_fov(msg, m, number_of_sectors);
		
		take_action(m);		
        }

void discretize_fov(const sensor_msgs::LaserScan::ConstPtr& msg, float * m, int number_of_sectors)
	{
		float sector_angle = M_PI/number_of_sectors;	//angle of one of the sectors
		
		int n_of_values_per_sector = round( sector_angle / msg->angle_increment );
		
		float min_r[number_of_sectors];		
		
		for (int j=0; j<number_of_sectors; j++)		//for each one of the five sectors I need to find the minimum
		{
			min_r[j]=10;	//set the highest minimum
			for (int i=0 + (j * n_of_values_per_sector); i < ((j+1) * n_of_values_per_sector); i++)
			{
				if (min_r[j] > msg->ranges[i])
				{
					min_r[j] = msg->ranges[i];
			      	}
			}
		}			
		
		int n_raw_sectors_per_sector = (number_of_sectors -1)/4;
		
		for (int j = 0; j<n_raw_sectors_per_sector; j++)
		{
			*m = std::min(*m, min_r[j]);
		}
		m++;
		for (int j = n_raw_sectors_per_sector; j<2*n_raw_sectors_per_sector; j++)
		{
			*m = std::min(*m, min_r[j]);
		}
		m++;
		//central sector		
		*m = min_r[(number_of_sectors-1)/2];
		m++;
		for (int j = ((number_of_sectors-1)/2) + 1; j<(((number_of_sectors-1)/2) + 1 + n_raw_sectors_per_sector); j++)
		{
			*m = std::min(*m, min_r[j]);
		}
		m++;
		for (int j = ((number_of_sectors-1)/2) + 1 + n_raw_sectors_per_sector; j<(((number_of_sectors-1)/2) + 1 + 2 * n_raw_sectors_per_sector); j++)
		{
			*m = std::min(*m, min_r[j]);
		}
			
	}
	
float angular(float difference, int factor)
	{
		return ( - difference * factor / std::abs(difference) );
	}
