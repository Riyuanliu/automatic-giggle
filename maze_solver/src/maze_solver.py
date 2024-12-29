#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from math import sqrt, pi
import heapq

class MazeSolver:
    def __init__(self):
        rospy.init_node("maze_solver", anonymous=True)

        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.sub_lidar = rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        self.sub_map = rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.vel = Twist()

        # LiDAR data
        self.front_distance = None
        self.left_distance = None
        self.right_distance = None
        self.safe_distance = 0.4  # Minimum distance to be considered safe to move forward

        # Map data
        self.map_data = None  # Occupancy grid data (from SLAM)
        self.map_resolution = None
        self.map_origin = None

        # Start and goal positions
        self.start_pos = (0, 0)  # Placeholder, update with actual start position
        self.goal_pos = (5, 5)  # Placeholder, update with actual goal position

        rospy.on_shutdown(self.shutdownhook)
        rospy.loginfo("Node initialized: solving maze")

    def lidar_callback(self, msg):
        """Callback for LiDAR data."""
        self.front_distance = min(min(msg.ranges[0:30]), min(msg.ranges[330:360]))  # Front
        self.left_distance = min(msg.ranges[60:120])  # Left side
        self.right_distance = min(msg.ranges[240:300])  # Right side

    def map_callback(self, msg):
        """Callback for map data."""
        self.map_data = msg  # Store the map data
        self.map_resolution = msg.info.resolution  # Get map resolution
        self.map_origin = msg.info.origin  # Get map origin (position of (0,0) in world space)

    def shutdownhook(self):
        """Stop the robot when the node is shut down."""
        self.pub.publish(Twist())  # Stop the robot
        rospy.loginfo("Shutting down...")

    def move_to_goal(self, goal):
        """Move towards the goal (pathfinding)."""
        # Implement a simple pathfinding algorithm (e.g., A*) to reach the goal.
        # This function should control the robot to navigate to the goal position.
        # For simplicity, let's assume the robot is directly moving towards the goal.
        rospy.loginfo(f"Moving towards goal: {goal}")
        self.vel.linear.x = 0.2
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    def explore(self):
        """Explore the maze by moving in open directions."""
        while not rospy.is_shutdown():
            if self.front_distance is None or self.left_distance is None or self.right_distance is None:
                rospy.loginfo("Waiting for LiDAR data...")
                self.rate.sleep()
                continue

            # Check if there's a wall in front of the robot
            if self.front_distance < self.safe_distance:
                # There is a wall in front, so we need to turn
                if self.left_distance > self.right_distance:
                    # Turn left if the left side is open
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = 0.2  # Turn left
                else:
                    # Turn right if the right side is open
                    self.vel.linear.x = 0.0
                    self.vel.angular.z = -0.2  # Turn right
            else:
                # No wall in front, move forward
                self.vel.linear.x = 0.2  # Move forward at 0.2 m/s
                self.vel.angular.z = 0.0  # No turning

            # Publish the velocity command
            self.pub.publish(self.vel)
            self.rate.sleep()

    def check_unexplored_area(self):
        """Check if the robot is approaching an unexplored area based on the map."""
        if self.map_data:
            # Convert current position to grid coordinates (this can be done via odometry or localization)
            # For now, assuming robot is at the center of the map
            width = self.map_data.info.width
            height = self.map_data.info.height
            grid_x = width // 2  # Example: robot is at the center of the map (this is a simplification)
            grid_y = height // 2
            index = grid_y * width + grid_x  # Get the index in the 1D array of the map

            # Check if the current position is unexplored (value -1 in occupancy grid)
            if self.map_data.data[index] == -1:
                rospy.loginfo("Unexplored area detected!")
                # Implement strategy to move towards unexplored area (you can use pathfinding here)
                self.vel.linear.x = 0.2
                self.vel.angular.z = 0.0
            else:
                rospy.loginfo("Explored area. Moving to next space.")
                # Continue moving forward or explore more based on strategy
                self.vel.linear.x = 0.2
                self.vel.angular.z = 0.0

            self.pub.publish(self.vel)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = MazeSolver()
        node.explore()  # Start exploration first
        node.move_to_goal(node.goal_pos)  # After exploration, move towards the goal
    except rospy.ROSInterruptException:
        pass
