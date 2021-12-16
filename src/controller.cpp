#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>   
#include "rt_assignment2/Command.h"
#include <algorithm>    // std::min

//Variables Declaration 
ros::Publisher pub;
ros::Subscriber sub;
geometry_msgs::Twist my_vel;
float	vel_linear_x = 0.0;
bool 	helper_status = false;


//Functions declaration (the definition after the main)

//Function to discretize the Field Of View. More info in the ReadMe.
void discretize_fov(const sensor_msgs::LaserScan::ConstPtr& msg, float * m, int number_of_sectors);	

void botCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

bool commandCallback(rt_assignment2::Command::Request &req, rt_assignment2::Command::Response &res);

int main (int argc, char **argv)
{
	ros::init(argc, argv, "bot_controller");  
	ros::NodeHandle nh;
	
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);				//To change the velocity
	sub = nh.subscribe("/base_scan", 1,botCallback);		//To see the obstacles
	
	ros::ServiceServer command= nh.advertiseService("/command", commandCallback);	//To know when to change speed
	
	//ROS_INFO("Executing...\n");
	
	ros::spin();
	return 0;
}

void botCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		//Dividing the ostacles in sectors
		int number_of_sectors = 9;
		
		float min[5] = {10, 10, 10, 10, 10};
		
		float * m = min;
		
		discretize_fov(msg, m, number_of_sectors);
		
		float diff1 = min[1] - min[3];
		float diff2 = min[0] - min[4];
		
		my_vel.linear.x = vel_linear_x;	
		my_vel.angular.z = - diff1 / std::abs(diff1) * 100;
		
		if (helper_status && min[2] < 2.0 && vel_linear_x >= 2.0) {
			my_vel.linear.x = 1.5;
		} 		
		
		pub.publish(my_vel);
        }

bool commandCallback(rt_assignment2::Command::Request &req, rt_assignment2::Command::Response &res)
	{
		switch (req.command)  
		{		
			case 1:	//Decreasing speed
				vel_linear_x = vel_linear_x - 0.5;
				break;
			case 2:	//Increasing speed
				vel_linear_x = vel_linear_x + 0.5;	
				break;		
			case 3:	//Toggling the helper
				helper_status = !helper_status;
				break;
		}
		
		//filling the response msg
		res.helper_status = helper_status;
		res.new_vel = vel_linear_x;
		
		//publishing the new velocity
		my_vel.linear.x = vel_linear_x;
		pub.publish(my_vel);
		
		return true;
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

























