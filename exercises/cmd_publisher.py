# Import the necessary libraries
import numpy as np
import rclpy
from rclpy.node import Node

# Import the ros messages
from geometry_msgs.msg import Twist

# Create a class to send the velocity commands to turtlebot via terminal
class CmdPublisher(Node):

    def __init__(self, start_timer = True):
        #Init the node
        super().__init__('cmd_publisher')

        # Create a publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Check if the input is given or not
        self.receiving_input = True

        # Init the values for linear and rotational velocities
        self.vel_x = 0.0
        self.vel_theta = 0.0
        
        # If the timer is started then publish the velocity commands with a defined frequency
        if(start_timer):
            # Send velocity commands every 0.1 seconds
            cmd_publisher_period = 0.1  # seconds

            # Ask for velocity every 2 seconds
            input_period = 2  # seconds

            # Create timers for receiving input and sending velocity commands
            self.timer = self.create_timer(cmd_publisher_period, self.velocity_timer)
            self.timer2 = self.create_timer(input_period, self.receive_input_timer)

    # Function to send velocity commands
    def velocity_timer(self):
        # if the input is not being received
        if(not self.receiving_input):
            # Move the vehicle
            self.move(self.vel_x, self.vel_theta)
        else:
            # Implement the behavior for the interval the input is being received such as stop
            # self.move(0.0, 0.0)
            return

    # Move the vehicle with input velocities
    def move(self, vel_x, vel_theta):
        # Create a Twist message
        msg = Twist()
        msg.linear.x = vel_x
        msg.angular.z = vel_theta
        # Publish the data
        self.publisher_.publish(msg)
        print("Send the velocities: ", vel_x, " and ", vel_theta)

    # Receive input
    def receive_input_timer(self):
        # Turn on the flag 
        self.receiving_input = True
        print('Enter Data')

        # Receive the inputs
        self.vel_x = float(input("Enter Linear Velocity: "))
        self.vel_theta = float(input("Enter Angular Velocity: "))

        # Change the flag
        self.receiving_input = False


# Define main function for ROS to begin with
def main(args=None):
    # Init the ros node
    rclpy.init(args=args)

    # Create a Command publisher
    cmd_publisher = CmdPublisher()

    # Spin the node to register all callbacks
    rclpy.spin(cmd_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cmd_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()