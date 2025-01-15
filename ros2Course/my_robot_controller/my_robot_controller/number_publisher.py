#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number_to_publish",10)
        self.declare_parameter("publish_frequency",1)
        self.number = self.get_parameter("number_to_publish").value
        self.publish_frequency = self.get_parameter("publish_frequency").value
        self.publisher = self.create_publisher(Int64,"number",10)
        self.create_timer(1.0/self.publish_frequency,self.timer_callback)
        self.get_logger().info("Number Publisher has been started")


    def timer_callback(self):
        number = Int64()
        number.data = self.number
        self.publisher.publish(number)




def main(args = None):
    rclpy.init(args = args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()