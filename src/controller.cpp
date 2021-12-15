#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>   
#include "rt_assignment2/Command.h"

ros::Publisher pub;
ros::Subscriber sub;

float	vel_linear_x = 0.0;
bool 	helper_status = 0;

void botCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		geometry_msgs::Twist my_vel;
				
		//Dividing the ostacles in sectors
		int number_of_sectors = 9;
		
		float sector_angle = M_PI/number_of_sectors;	//angle of one of the five sectors
		int n_of_values_per_sector = round( sector_angle / msg->angle_increment );
		float min[number_of_sectors];		
		
		for (int j=0; j<number_of_sectors; j++)		//for each one of the five sectors I need to find the minimum
		{
			min[j]=10;	//set the highest minimum
			for (int i=0 + (j * n_of_values_per_sector); i < ((j+1) * n_of_values_per_sector); i++)
			{
				if (min[j] > msg->ranges[i])
				{
					min[j] = msg->ranges[i];
			      	}
			}
		}	
		
		float min_r[5];
		min_r[2] = min[4];
		if (min[3]>min[2]) min_r[1] = min[2];
		else {min_r[1] = min[3];}
		
		if (min[1]>min[0]) min_r[0] = min[0];
		else {min_r[0] = min[1];}
		
		if (min[5]>min[6]) min_r[3] = min[6];
		else {min_r[3] = min[5];}
		
		if (min[7]>min[8]) min_r[4] = min[8];
		else {min_r[4] = min[7];}
		
		//Deciding where to turn
		float diff1 = min_r[1] - min_r[3];
		float diff2 = min_r[0] - min_r[4];
		
		//if (helper_status && min[2] < 2.0 && vel_linear_x >= 2.0) {
		if (helper_status && min_r[2] < 2.0 && vel_linear_x >= 2.0) {
			
			my_vel.linear.x = 1.5;
			//ROS_INFO("WEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n");
		
		
		} else {
		
			my_vel.linear.x = vel_linear_x;	
		}
		
		
		
		my_vel.angular.z = - diff1 / std::abs(diff1) * 1000;
		
				
		//	if (std::abs(diff1) > std::abs(diff2)) 
		//	{			
		//		my_vel.angular.z = - diff1 / std::abs(diff1) * 30;		
		//	}else{
		//		my_vel.angular.z = - diff2 / std::abs(diff2) * 100;				
		//	}					
		
		
		
		
		pub.publish(my_vel);
        }
  
bool commandCallback(rt_assignment2::Command::Request &req, rt_assignment2::Command::Response &res)
	{

		geometry_msgs::Twist my_vel;	
		
		switch (req.command)  
		{		
			case 1:
				vel_linear_x = vel_linear_x - 0.5;
				break;
			
			case 2:
				vel_linear_x = vel_linear_x + 0.5;	
				break;		
			
			case 3:
				helper_status = !helper_status;
				break;
		}
		
		res.helper_status = helper_status;
		my_vel.linear.x = vel_linear_x;
		res.new_vel = vel_linear_x;
		
		pub.publish(my_vel);
		return true;
	}  

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




























