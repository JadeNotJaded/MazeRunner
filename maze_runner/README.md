# MazeRunner
This program allows a Turtlebot3 to escape a maze autonomously. To run this file start up roscore on the turtlebot3 or simulator and run the launch file:

roslaunch maze_runner maze_runner.launch

The nodes that we use are controller.py and object_detector.py. Object_detector subscribes to scan and publishes the distance to the nearest object within a rectangular area infront of the turtlebot. This area is approximately the area through which the robot will travel when going forward. 

The next node (which is our main node) is controller.py. This node subscribes to distance and scan and publishes to cmd_vel sending actions to our turtlebot. We set our rospy rate to 10 so movements are published 10 times per second.

In controller, the first variable that comes into play is time. We use time to make sure that the robot initially finds a wall before it starts following it. If the last time that the robot was close to the wall becomes too great, the robot will stop turning and will just move forward until it comes close to the wall. This algorithm will also execute if (for some unknown reason) the turtlebot is taken away from the wall or (tragically) moves away from it autonomously. 

The next piece of data that comes into play is the distance ahead. If the distance ahead is too small (less than our distance ahead threshold) the angular velocity increases and thus the robot makes a left turn.

The final important piece of information is the diagonal distance. If this is too large (greater than our diagnonal threshold) our robot 'knows' that the wall has disappeared and thus makes a right turn until it has located the wall again.

Our algorithm is simple, effective and easy to understand, however it does not deal with the problem of oscillation. Our next job would be to update this oscillation using PID, controllers or a combination of both.


