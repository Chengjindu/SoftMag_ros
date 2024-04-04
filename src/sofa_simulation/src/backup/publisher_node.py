#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile
import time

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/animation/receiver/position', 10)
        self.timer = self.create_timer(1.0, self.publish_position)
        self.msg = Float32MultiArray()

    def publish_position(self):
        # Create a sample message with a single numeric value
        self.msg.data = [42.0]  # Change this value to whatever you need
        self.get_logger().info('Publishing: {}'.format(self.msg.data))
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = PublisherNode()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

