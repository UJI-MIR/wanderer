# Import the necessary libraries
import numpy as np
import rclpy
from rclpy.node import Node

# Import the ros messages
from nav_msgs.msg import Odometry

# Create a class for Odom Subscriber
class OdomSubscriber(Node):

    def __init__(self):
        #Init the node
        super().__init__('odom_subscriber')

        # Create a subscriber to odometry topic 
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.subscription  # prevent unused variable warning

    # Define the odometry callback function
    def odom_callback(self, msg):
        print('Odom Data: ')
        print('Position')

        # Get the position and orientation data
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # Convert quaternion to euler
        (roll, pitch, yaw) = self.euler_from_quaternion (orientation)

        # Display the data
        print("X: ", position.x)
        print("Y: ", position.y)
        print("Z: ", position.z)
        print("roll: ", roll)
        print("pitch: ", pitch)
        print("yaw: ", yaw)


    # Convert the quaternion into euler angles
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

# Define main function for ROS to begin with
def main(args=None):
    # Init the ros node
    rclpy.init(args=args)

    # Create a Odom Subscriber
    odom_subscriber = OdomSubscriber()

    # Spin the node to register all callbacks
    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()