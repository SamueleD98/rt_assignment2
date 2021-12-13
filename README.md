# Python Robotics Simulator
This is a simple robot simulator that implements two ROS nodes to control a robot in the given environment.
The gui node will allow the user to increase or decrease the speed of the robot and to reset its position to the initial one. The robot will change its direction autonomously, avoiding to collide with the wall while the speed is low enough.  

## Running
The repository has a launch file that will run, in order:  
-the ros master  
-the world representation  
-the robot controller node  
-the robot gui node  
  
rosrun stage_ros stageros $(rospack find rt_assignment2)/world/my_world.world  
  
  
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CODICE LANCIO!!!!!!!!!!!!!!!!!!!!!!!!!!

## Robot behaviour 
The robot moves always in the same direction 

The robot sensors, actually, can detect boxes around all directions but, without limiting the fields of view, the robot will keep reaching (or avoiding) the same silver (golden) box as it is the closest to the robot.


The code implements the following algorithm:  !!!!!!!!!!!!!!
<pre>
<b>while</b> the program is running
	retrieve the position of the closest golden box   
	<b>if</b> its distance is less than an arbitrary one  
		stop the robot  
		<b>if</b> the robot has been trying to avoid obstacles for 10 timesteps/turns   
			move the robot backward a little   
		<b>else</b>  
			search for the best direction to turn into  
			turn  
	<b>else</b>  
 		retrieve the position of the closest silver box  
		<b>if</b> its distance is less than an arbitrary one  
			stop the robot  
			grab the silver box  
			<b>if</b> the action was succesful  
				turn by 180° degrees  
				release the silver box  
				turn by 180° degrees   
			<b>else</b>   
				adjust the trajectory according to the angle of the silver box with respect to the robot direction  
		<b>else</b> <b>if</b> the silver box is in sight  
			set a slower speed  
			adjust the trajectory according to the angle of the silver box with respect to the robot direction  
		<b>else</b>   
			set cruising speed
</pre>

## Further improvement
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
