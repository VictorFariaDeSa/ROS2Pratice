#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class AddTwoServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")
        self.server = self.create_service(AddTwoInts,"add_two_ints",self.callback__add_two_ints)

    def callback__add_two_ints(self,request,response):
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response


def main(args = None):
    rclpy.init()
    node = AddTwoServerNode()
    rclpy.spin(node)
    rclpy.shutdown()