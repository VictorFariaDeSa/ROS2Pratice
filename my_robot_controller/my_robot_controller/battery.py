#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial

from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery")
        self.battery_state = "full"
        self.last_time_battery_state_changed = self.get_current_time_in_secondes()
        self.battery_timer = self.create_timer(0.1,self.check_battery_state)
        
        self.get_logger().info("battery node has been started")

    def get_current_time_in_secondes(self):
        secs, nsecs = self.get_clock().now().seconds_nanoseconds()
        return secs + nsecs/1e9
    
    def check_battery_state(self):
        time_now = self.get_current_time_in_secondes()
        if self.battery_state == "full":
            if time_now - self.last_time_battery_state_changed > 4.0:
                self.battery_state = "empty"
                self.get_logger().info("Battery is empty! Charging battery")
                self.last_time_battery_state_changed = time_now
                self.call_set_led_server(3,1)
        else:
            if time_now - self.last_time_battery_state_changed > 6.0:
                self.battery_state = "full"
                self.get_logger().info("Battery is full again!")
                self.last_time_battery_state_changed = time_now
                self.call_set_led_server(3,0)

    def call_set_led_server(self, led_number, state):
            client = self.create_client(SetLed,"set_led")
            while not client.wait_for_service(1.0):
                self.get_logger().warn("Waiting for Server SetLed...")
            request = SetLed.Request()
            request.led_number = led_number
            request.state = state

            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_call_set_led,led_number=led_number,state=state))

    def callback_call_set_led(self, future, led_number ,state):
        try:
            response = future.result()
            self.get_logger().info(str(response.sucess))
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")



def main(args = None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
