# Python Robotics Simulator
This is a simple robot simulator that implements two ROS nodes to control a robot in the given environment.
The gui node will allow the user to increase or decrease the speed of the robot and to reset its position to the initial one. The robot will change its direction autonomously, avoiding to collide with the wall while the speed is low enough.  

## Running
The repository has a launch file that will run, in order:
-the ros master
-the world representation
-the robot controller node
-the robot gui node

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

As can be seen, the robot has no memory of the past surrounding state. This makes it capable of moving in a dynamic environment, in fact it is constantly looking for obstacles: if an object moves between the robot and a silver box it is trying to grab, the robot will not continue to advance but it will stop and change direction.  
The only thing the robot remembers is the number of times it tries to change direction to avoid an obstacle. This is necessary because sometimes the robot finds itself in a situation where there are obstacles ahead, on the left and on the right. Then it will start turning between left and right until the program stops. Knowing the number of times it tries to turn, it is possible to have the robot drive in reverse and find a different way to keep driving.  

The code uses some new functions and improves others:
 + The **drive** function has been replaced by the **set_speed** function. The latter doesn't stop the robot at the end of its execution in order 
 to have a smoother ride.  
 + The functions to which the search for boxes is assigned are:  
 	- The **find_next_silver_token** that filters, among all the tokens that the robot can see, the nearest silver token in the preset viewing angle  
 	- The **find_next_golden_token** that does the same thing as the find_next_silver_token function, but with a smaller viewing angle  
 + The **check_direction** function compares the number of obstacles on the right of the robot with the ones on the left and decides into which direction to turn. If the numbers match the searching restarts with a greater searching distance  
 + The **adjust_trajectory** turns the robot of the given angle with a speed that follows a sinusoidal law: the bigger is the angle, the faster the robots turns. If the angle is close to zero then it is better to avoid to make a turn, let alone a fast one, because it could deviate the route.
 


## Further improvement
The robot is able to turn continuously around the circuit but unfortunately after several turns it is possible that the robot changes the direction of rotation. For this it is possible to design an algorithm that makes the **robot able to recognize if it has turned more than 180° and adjust its trajectory accordingly**.

Additionally, the continuous movement of the boxes inevitably brings them closer to the wall causing a collision or a direct passage of the robot (without it grasping the box). This can be solved by **repositioning the box halfway between the two walls each time the robot grabs it**. After the release, the robot will have to repeat the same actions in reverse order to return to the starting position and thus avoid changing the direction of rotation.

Furthermore, the robot, with the current configuration, needs a long time to adjust the trajectory in order to drive in the long passage. The solution is not as simple as reducing the steering angle when approaching an obstacle: it would work but it would also increase the time it takes to go around corners. It should distinguish the different situations and apply the best solution.  

Finally, adapting the speed to the distance of the closest object could **improve the overall performance of the robot**.  
