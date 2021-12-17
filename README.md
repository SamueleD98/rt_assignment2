# Python Robotics Simulator
This is a software architecture in ROS that implements two nodes to control a robot in the given environment.  
The robot will change its direction autonomously, avoiding colliding with the wall (at least until the speed is low enough). The user interface node will allow the user to increase or decrease the speed of the robot and to reset its position to the initial one. Furthermore, the user can engage the helper which will also modify the linear velocity when approaching a curve.

## Preparation phase
To control the robot in the given environment it's necessary to know its surroundings and how to change its parameters.  
Once the simulation environment is runned using `rosrun stage_ros stageros $(rospack find rt_assignment2)/world/my_world.world`, the `rostopic list` command will list every active topic. The informations about the necessary topics are given with the `rostopic info <topic name>`:  

![topics](/images/topics.png)  

In the *base_scan* topic, the *stageros* node will publish information about the surrounding environment, indeed it is signed as a Publisher.    
The *stageros* is instead a Subscriber when it comes to the *cmd_vel* topic. In the latter it's published the velocity that the robot needs to have.    
In the images above, it is also shown the type of the message for each topic. Typing `rosmsg show <type>`, the fields of the message, as their type, are shown:  

![msg](/images/msg.png)  

In the following an extract of a message published on the *base_scan* topic:  

![base_scan](/images/base_scan.png)  

*angle_min* and *angle_max* prove that the robot can see the obstacles in the [-PI/2, PI/2] range meanwhile the *angle_increment* value will be used to compute the number of values in that range. Ultimately, the *ranges* vector has the distances of the obstacles in the described range.  

About the *geometry_msgs/Twist* type, it's now known how to write the message to change both the linear and the angular velocity. The software will focus only on the *x* value of the linear velocity and on the *z* value of the angular one, since it's assumed that the robot can't move transversally to the pointed direction and it can't roll on itself (*x* and *y* values of the angular velocity are always equal to zero).


## Running
The repository has a launch file that will run, in order:  
- the world representation;  
- the robot controller node;
- the robot interface node.

The ROS Master node will be automatically called when launching the file. To launch use `roslaunch rt_assignment2 simulation.launch`

## Robot behaviour 
The robot sensors can detect the obstacles around all directions but, in order to understand the best direction to turn into, the field of view is *discretized* into 5 subsections. Each one of them will be represented by the distance of the closest object in that direction. The third one will be the central one, and so it will be the distance of the object right in front of the robot. The controller node will adjust the trajectory proportionally to the difference between opposite subsections. 

## User Interface node
The UI node takes care of the user inputs for the control of the robot. It offers five options, as shown in the image:  

![user_interface](/images/roslaunch.png)

The code implements the following algorithm:  
<pre>
<b>while</b> the program is running
	input user choice
	<b>switch</b> user choice
		<b>case</b> 'r'
			call the reset service
			<b>break</b>
		<b>case</b> 's'
			<b>if</b> linear velocity == 0
				print "Error, the robot is not moving"
			<b>else</b> 
				set the command to decrease the speed
				call the service with the command
				update the linear velocity 
				print the new velocity
				<b>if</b> the speed is greater than the dangerous one
					print a warning
			<b>break</b>
		<b>case</b> 'w'
			set the command to increase the speed
			call the service with the command
			update the linear velocity 
			print the new velocity
			<b>if</b> the speed is greater than the dangerous one
				print a warning
			<b>break</b>
		<b>case</b> 't'
			set the command to toggle the helper
			call the service with the command
			<b>if</b> the helper is active
				print "Helper activated"
			<b>else</b>
				print "Helper deactivated"
			<b>break</b>
		<b>default</b>
			print "Wrong character, please type again."
			
	ignore other chars in the same line
</pre>

## Controller node
This is the node the actually modifies the robot velocity, both the linear and the angular one, with the instructions given by the user. It behaves as a server for the *command* service, as a subscriber of the *base_scan* topic and as a publisher of the *cmd_vel* topic.  
Each time something is published on the *base_scan* topic, the laserCallback function is executed:
<pre>
call the *discretize field of view* function
call the *take action* function
</pre>
The *discretize_fov* function takes as arguments the msg published on the *base_scan* topic, the pointer to an array and an integer. The latter is the number of subsections in which the ranges vector will be divided. The function discretize the ranges vector in the given number of subsections, and then it will unify the adjacent sectors. In the end there will be only five sectors as shown in the image:
![Robot_field_of_view](/images/robot_view.png)  
An increasing number of subsections reduce the width of the central one, and so the robot won't change its direction for an obstacle which is not really on its trajectory. This has been proved to result in a less fragmented trajectory.   
Algorithm:
<pre>
compute the sector angle as PI radians on the given number of sectors
compute the number of values per sector as the rounding of the sector angle on the increment angle 
<b>for</b> each one of the sectors
	set the min to 10
	<b>for</b> each one of the values in that sector
		<b>if</b> the value is less than the min
			set the min to that value
compute the number of adjacent sectors that will be unified as the total number of sectors minus one, all divided by 4
<b>for</b> each one of the final five sections
	compute the min distance as the min value between the n adjacent sectors			
</pre>
As soon as the *min* vector is updated, the *take action* function is called:
<pre>
compute *diff* as the difference between the second and the fourth sector	
<b>if</b> the value is inferior to a tollerance range
	compute *diff* as the difference between the first and the fifth sector
	<b>if</b> the value is inferior to a tollerance range
		set the linear velocity to 0.1
		set the angular velocity to 100
		publish the new velocity
		return
set the velocity to the old value
<b>if</b> the value of the third sector (the central one) is less than 1.2
	<b>if</b> the helper is active and the linear velocity is greater than the dangerous one
		set the linear velocity to 0.5
	set the angular velocity as minus the fraction of *diff* on its absolute, multiplied by 100
<b>else</b> <b>if</b> the value of the third sector (the central one) is less than 1.6
	<b>if</b> the helper is active and the linear velocity is greater than the dangerous one
		set the linear velocity to 1.0
	set the angular velocity as minus the fraction of *diff* on its absolute, multiplied by 75
<b>else</b> <b>if</b> the value of the third sector (the central one) is less than 2.0
	<b>if</b> the helper is active and the linear velocity is greater than the dangerous one
		set the linear velocity to 1.5
	set the angular velocity as minus the fraction of *diff* on its absolute, multiplied by 50
<b>else</b>
	set the angular velocity as minus the fraction of *diff* on its absolute
publish the new velocity
</pre>  

Ultimately, the *commandCallback function*:  
<pre>
<b>switch</b> the given command
	<b>case</b> 1
		set the linear velocity as the old one minus 0.5
		<b>break</b>
	<b>case</b> 2
		set the linear velocity as the old one plus 0.5
		<b>break</b>
	<b>case</b> 3
		set the helper status as the negation of itself
		<b>break</b>
set the answer to the service call with the helper status and the linear velocity
publish the new velocity
</pre>

## Future
 - n sectors as parameter
 - secure distance which changes with the speed
