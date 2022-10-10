# Import the necessary libraries
import numpy as np
import rclpy
from rclpy.node import Node

# Import the ros messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Create a class for Moving Forward and stop if object is detected
class MoveForward(Node):

    def __init__(self):
        #Init the node
        super().__init__('move_forward')

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
        self.vel_x = 0.1
        self.vel_theta = 0.0


    # Define the scan callback function
    def scan_callback(self, msg):
        # Compute the range from the object in front by taking mean of ranges in -5 to 5 degrees
        front_range = (np.mean(msg.ranges[0:5]) + np.mean(msg.ranges[355:]))/2

        # Check if the range is below threshold
        obj_detected = front_range < self.collision_th

        # Print the range for debugging
        print("Range is ", obj_detected, front_range)

        # If object not detected, move the robot with predefined velocity
        if(not obj_detected):
            self.move(self.vel_x, self.vel_theta)
        else:
            # Stop the robot if object detected
            self.move(0.0, 0.0)


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

    # Create a Move Forward object
    move_forward = MoveForward()

    # Spin the node to register all callbacks
    rclpy.spin(move_forward)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_forward.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()