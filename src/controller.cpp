#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>   
#include "rt_assignment2/Velocity.h"

ros::Publisher pub;

float	vel_linear_x = 0.0;
bool 	helper_status = 0;

void botCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		geometry_msgs::Twist my_vel;
				
		//Dividing the ostacles in sectors
		int number_of_sectors = 5;
		
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
		
		//Deciding where to turn
		float diff1 = min[1] - min[3];
		float diff2 = min[0] - min[4];
		if (min[2] < 0.8) 	//emergenza
		{		
			if (std::abs(diff1) > std::abs(diff2)) 
			{			
				my_vel.angular.z = - diff1 / std::abs(diff1) * 30;		
			}else{
				my_vel.angular.z = - diff2 / std::abs(diff2) * 100;				
			}					
		
		}else{ 			//addrizzo solamente
			my_vel.angular.z = - diff1 / std::abs(diff1) * vel_linear_x * 100;
			//ROS_INFO("Going..");		
		}		
		if (min[2] < 0.2) 	//addio
		{
			ROS_INFO("Collision!");
		}	
		
		my_vel.linear.x = vel_linear_x;	
		pub.publish(my_vel);
        }
  
bool velocityCallback (rt_assignment2::Velocity::Request &req, rt_assignment2::Velocity::Response &res){

	geometry_msgs::Twist my_vel;	
	
	res.resp = true;	  
	
	if (req.command == req.acc) {
		vel_linear_x = vel_linear_x + 0.5;
		if (vel_linear_x >= 2.0) res.resp = false;		
	}else if (req.command == req.dec && vel_linear_x >= 0.5) {
		vel_linear_x = vel_linear_x - 0.5;
	}else if (req.toggle_helper == 1){
		//toggle
		helper_status = !helper_status;
		if (helper_status) res.resp = false;
	}else {
		res.resp = false;
	}
	
	
	// se helper attivo allora devo controllare la velocità in base agli ostacoli
	my_vel.linear.x = vel_linear_x;
	pub.publish(my_vel);
	
	return true;
}  

int main (int argc, char **argv)
{
	ros::init(argc, argv, "bot_controller");  
	ros::NodeHandle nh;
	
	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);				//To change the velocity
	ros::Subscriber sub = nh.subscribe("/base_scan", 1,botCallback);		//To see the obstacles
	
	ros::ServiceServer velocity= nh.advertiseService("/velocity", velocityCallback);	//To know when to change speed
	
	ROS_INFO("Executing...\n");
	
	ros::spin();
	return 0;
}




























