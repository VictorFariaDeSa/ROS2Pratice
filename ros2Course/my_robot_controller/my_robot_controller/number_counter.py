#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool


class NumberCounterNode(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.get_logger().info("Number Counter has been started")
        self.publisher = self.create_publisher(Int64,"number_count",10)
        self.subcriber = self.create_subscription(Int64,"number",self.numberCallback,10)
        self.counter = 0
        self.restart_counter_service = self.create_service(SetBool,"restart_counter",self.callback_restart_counter)

    def callback_restart_counter(self,request,response):
        if request.data:
            self.counter = 0
            response.success = True
            response.message = "Counter been reset"
        else:
            response.success = False
            response.message = "Counter has not been reset"
        return response


    def numberCallback(self,msg:Int64):
        self.counter += msg.data
        #self.get_logger().info(str(self.counter))
        number = Int64()
        number.data = self.counter
        self.publisher.publish(number)





def main(args = None):
    rclpy.init(args = args)
    node = NumberCounterNode()
    rclpy.spin(node)
    rclpy.shutdown()