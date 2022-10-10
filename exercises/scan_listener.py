# Import the necessary libraries
import numpy as np
import rclpy
from rclpy.node import Node

# Import the ros messages
from sensor_msgs.msg import LaserScan


# Create a class for Scan Data Subscriber
class ScanSubscriber(Node):

    def __init__(self):
        #Init the node
        super().__init__('scan_subscriber')

        # Define the QOS policy such that the data sent by the turtlebot is compatible with the subscriber. Change the default behavior
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Create a subscriber to scan topic 
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_policy)

        self.subscription  # prevent unused variable warning

    # Define the scan callback function
    def scan_callback(self, msg):
        print('Scan Data: ')
        print('angle_min: ', msg.angle_min)
        print('angle_max: ', msg.angle_max)
        print('angle_increment: ', msg.angle_increment)
        print('num of elements: ', len(msg.ranges))
        print("Range in front", np.mean(msg.ranges[0]))

# Define main function for ROS to begin with
def main(args=None):
    # Init the ros node
    rclpy.init(args=args)

    # Create a Scan Subscriber
    scan_subscriber = ScanSubscriber()

    # Spin the node to register all callbacks
    rclpy.spin(scan_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()