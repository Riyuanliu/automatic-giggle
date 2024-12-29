#!/usr/bin/env python3
import rospy
from TurtlebotDriving import TurtlebotDriving
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np

class WallFollower:
    def __init__(self, speed, distance_wall, side):
        """Initialize the WallFollower class with parameters and ROS setup."""
        self.speed = speed
        self.distance_wall = distance_wall
        self.wall_lead = 0.5
        self.side = 1 if side == 'left' else -1
        self.g_alpha = 0
        
        # History (for future use, not used in the current implementation)
        self.x_history = []
        self.y_history = []

        # ROS Node Initialization
        rospy.init_node('PythonControl')
        self.rate = rospy.Rate(10)

        # ROS Publisher and Subscriber
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
        rospy.Subscriber("scan", LaserScan, self.scan_callback)
        
        print(f'Following wall on the {side} side...')
        rospy.sleep(1)

    # --------------------------
    # ROS Control Functions
    # --------------------------

    def update_vel(self, linear_vel, angular_vel):
        """Update the turtlebot's velocity based on the given linear and angular velocities."""
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.cmd_vel_pub.publish(msg)

    def scan_callback(self, msg):
        """Process the laser scan data and adjust robot movement accordingly."""
        scan_max_value = msg.range_max

        # Extract scan regions for easier access
        self.regions = self.get_scan_regions(msg, scan_max_value)

        # Print current distance to the wall being tracked (left or right)
        if self.side == 1:  # Following left wall
            print(f"Current distance to left wall: {self.regions['W']} meters")
        else:  # Following right wall
            print(f"Current distance to right wall: {self.regions['E']} meters")

        # Calculate and adjust robot's turn angle based on proximity to wall
        self.adjust_turn(scan_max_value)

    # --------------------------
    # Scan Data Processing
    # --------------------------

    def get_scan_regions(self, msg, scan_max_value):
        """Extract relevant regions from the scan data."""
        return {
            'N': min(min(msg.ranges[0:5] + msg.ranges[-5:]), scan_max_value),
            'NNW': min(min(msg.ranges[11:20]), scan_max_value),
            'NW': min(min(msg.ranges[41:50]), scan_max_value),
            'WNW': min(min(msg.ranges[64:73]), scan_max_value),
            'W': min(min(msg.ranges[86:95]), scan_max_value),
            'E': min(min(msg.ranges[266:275]), scan_max_value),
            'ENE': min(min(msg.ranges[289:298]), scan_max_value),
            'NE': min(min(msg.ranges[311:320]), scan_max_value),
            'NNE': min(min(msg.ranges[341:350]), scan_max_value),
            'frontwide': min(min(msg.ranges[0:40] + msg.ranges[-40:]), 10)
        }

    def adjust_turn(self, scan_max_value):
        """Adjust the robot's turning angle based on scan data."""
        y0 = self.regions['E'] if self.side == -1 else self.regions['W']
        x1 = (self.regions['ENE'] if self.side == -1 else self.regions['WNW']) * math.sin(math.radians(23))
        y1 = (self.regions['ENE'] if self.side == -1 else self.regions['WNW']) * math.cos(math.radians(23))

        # If the robot is heading towards a wall, turn sideways
        if y0 >= self.distance_wall * 2 and self.regions['N'] < scan_max_value:
            print("Turning sideways")
            self.g_alpha = -math.pi / 4 * self.side
        else:
            # Determine if there is a wall in front and adjust the turn angle accordingly
            front_scan = min([ 
                self.regions['N'],
                self.regions['NNW'] + (scan_max_value - self.regions['WNW']),
                self.regions['NNE'] + (scan_max_value - self.regions['ENE'])
            ])

            turn_fix = 0 if front_scan >= 0.5 else 1 - front_scan

            # Calculate angular velocity to maintain wall distance
            abs_alpha = math.atan2(y1 - self.distance_wall, x1 + self.wall_lead - y0) - turn_fix * 1.5
            self.g_alpha = self.side * abs_alpha

    # --------------------------
    # Main Execution Loop
    # --------------------------

    def run(self):
        """Run the wall-following logic and monitor the robot's progress."""
        try:
            bot = TurtlebotDriving()
            completed = False

            # Display initial configuration
            print(f'1) Linear speed: {self.speed}')
            print(f'2) Distance to wall: {self.distance_wall}\n')

            t0 = time.time()

            # Main loop: Stop when no obstacle is detected in the front
            while self.regions['frontwide'] < 10 and not rospy.is_shutdown():
                self.update_vel(self.speed, self.g_alpha)
                self.rate.sleep()

            t1 = time.time()

            print("Maze Solved!")
            bot.stop()
            path = bot.obtainpath()
            bot.plot_trajectory('Wall Following')
            bot.relaunch()
            completed = True

            return path, len(path), t1 - t0, completed

        except rospy.ROSInterruptException:
            print("ROS Interrupt Exception occurred!")
            pass

        except Exception as e:
            print(f"An error occurred: {e}")
            bot.stop()
