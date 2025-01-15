#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist #Como estamos usando o package dentro do my_robot_controller package temos que adiciona-lo como dependencia no package.xml

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmv_vel_pub = self.create_publisher(Twist, "turtle1/cmd_vel",10) #Passamos como parametro o tipo da men sagem que ele vai publicar, o nome e o cue size
        self.get_logger().info("Draw circle node has been started")
        self.timer = self.create_timer(0.5,self.send_velocity_command)


    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmv_vel_pub.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()