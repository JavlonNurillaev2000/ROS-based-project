# ROS-based-project
#Here is an example of a ROS-based project that uses a TurtleBot3 robot to navigate through a maze:

#1. Create a new ROS package using the following command in the terminal:
$ catkin_create_pkg turtlebot3_maze rospy

#2. Next, create a Python script called turtlebot3_maze.py in the scripts directory of your package. This script will contain the code for the project. Here is an example of what the code might look like:
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MazeNavigator:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist = Twist()
        
    def laser_callback(self, data):
        # Check the distance to the nearest object in front of the robot
        front_dist = min(data.ranges[0:30] + data.ranges[330:360])
        
        # If there is an obstacle in front of the robot, turn left
        if front_dist < 0.5:
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.3
        # Otherwise, move forward
        else:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        
        # Publish the Twist message to control the robot
        self.cmd_vel_pub.publish(self.twist)
        
if __name__ == '__main__':
    rospy.init_node('maze_navigator')
    navigator = MazeNavigator()
    rospy.spin()
#This code creates a MazeNavigator class that subscribes to the scan topic (which provides data from the robot's LiDAR sensor) and publishes to the cmd_vel topic (which controls the robot's movement). In the laser_callback method, the robot checks the distance to the nearest obstacle in front of it, and turns left if there is an obstacle, or moves forward if there is not. The rospy.spin() method is used to keep the node running until it is shutdown.

#3. Finally. Lauching the project:
$ roslaunch turtlebot3_maze turtlebot3_maze.launch

#This will start the project and launch the TurtleBot3 robot in a simulated maze environment. The turtlebot3_maze.launch file should be created in the launch directory of your package, and should include the necessary configuration for launching the robot and any other required nodes or packages.
