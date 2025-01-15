#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class TemplateNode(Node): #MODIFY NAME

    def __init__(self):
        super().__init__("pnode_name") #MODIFY NAME
        self.get_logger().info("nome_name has been started")

        


def main(args = None):
    rclpy.init(args=args)
    node = TemplateNode() #MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
