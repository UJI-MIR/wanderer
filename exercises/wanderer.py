# Import the necessary libraries
import numpy as np
import random
import time

import rclpy
from rclpy.node import Node

# Import the ros messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Create a class for Wanderer Behavior
class Wanderer(Node):

    def __init__(self):
        #Init the node
        super().__init__('wanderer')

        # Create a publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Define the QOS policy such that the data sent by the turtlebot is compatible with the subscriber. Change the default behavior
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        # Create a subscriber to scan topic 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)

        self.subscription  # prevent unused variable warning

        # Set the properties
        self.collision_th = 0.3
        self.vel_x = float(0.1)
        self.vel_theta = float(0.0)
        self.rotation_vel = 0.5 # Rotational Velocity to change heading

        self.scan_range = 90 # In degress. -45 to 45 degrees

    # Define the scan callback function
    def scan_callback(self, msg):
        # Create an empty list to store the scan ranges
        front_scan_ranges = []
        # Store the range values from 0 to scan_range/2
        front_scan_ranges.extend(msg.ranges[0:int(self.scan_range/2)])
        # Store the range values from 360 - scan_range/2 to 360
        front_scan_ranges.extend(msg.ranges[len(msg.ranges)-int(self.scan_range/2):])

        # Filter the values
        front_range = [x for x in front_scan_ranges if x > msg.range_min and x < msg.range_max]

        # Take the minimum distance from front ranges and check if it is below collision threshold
        obj_detected = np.min(front_range) < self.collision_th if len(front_range) > 0 else True

        # Debug print
        print("Object Detected ", obj_detected)

        # If object is not detected, move straight
        if(not obj_detected):
            self.move(self.vel_x, self.vel_theta)
        # If object is detected then take random direction and rotate the robot for 2 seconds
        else:
            direction = -1 if random.randint(0, 1) == 0 else 1
            self.move(0.0, direction*self.rotation_vel)
            time.sleep(2)

    # Move the vehicle with input velocities
    def move(self, vel_x, vel_theta):
        # Create a Twist message
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_theta
        # Publish the data
        self.publisher_.publish(msg)
        print("Send the velocities: ", vel_x, " and ", vel_theta)

# Define main function for ROS to begin with
def main(args=None):
    # Init the ros node
    rclpy.init(args=args)

    # Create a Wanderer Behavior
    wanderer = Wanderer()

    # Spin the node to register all callbacks
    rclpy.spin(wanderer)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wanderer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()