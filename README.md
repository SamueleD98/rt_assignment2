# Python Robotics Simulator
This is a simple robot simulator that implements two ROS nodes to control a robot in the given environment.
The robot will change its direction autonomously, avoiding to collide with the wall (at least until the speed is low enough). The gui node will allow the user to increase or decrease the speed of the robot and to reset its position to the initial one. Furthermore the user can engage the *helper* which will also modify the linear velocity when approaching a curve.

## Running
The repository has a launch file that will run, in order:  
- the world representation;  
- the robot controller node;
- the robot interface node.

The ros master node will be automatically called when launching the file. To launch use `roslaunch rt_assignment2 launcher.launch`

## Robot behaviour 
The robot sensors can detect the obstacles around all directions but, in order to understand the best direction to turn into, the field of view is *discretisize* into 5 subsections. Each one of them will be represented by the distance of the closest object in that direction. The third one will be the central one, and so it will be the distance of the object right in front of the robot. The controller node will adjust the trajectory proportionally to the difference between opposite subsections. 

## User Interface node
The UI node takes care of the user inputs for the control of the robot. It will accept 4 kind of different input, as shown in the image.  
!!!!!!!!!!!!!!!!!!!!!!!  

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
The *discretize_fov* function takes as arguments the msg pubblished on the topic, the pointer to a vector and an integer. The latter is the number of subsections in which the ranges vector will be divided. The bigger this number is, the smaller the central subsection will be. 
![Robot_field_of_view](/images/robot_view.png)  


## Further improvement
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
