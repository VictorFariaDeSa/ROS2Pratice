import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedStateArray
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel")
        self.declare_parameter("led_states",[0,0,0])
        self.led_states = self.get_parameter("led_states").value
        self.led_states_publisher = self.create_publisher(LedStateArray,"led_states",10)
        self.led_states_timer = self.create_timer(4,self.publish_led_states)
        self.set_led_service = self.create_service(SetLed,"set_led",self.set_led_callback)
        
        self.get_logger().info("led_panel has been started")

    def publish_led_states(self):
        msg = LedStateArray()
        msg.led_states = self.led_states
        self.led_states_publisher.publish(msg)
        
    def set_led_callback(self,request,response):
        led_number = request.led_number
        state = request.state

        if led_number > len(self.led_states) or led_number <= 0:
            response.sucess = False
            return response
        
        if state not in [0,1]:
            response.sucess = False
            return response
        
        self.led_states[led_number-1] = state
        response.sucess = True
        self.publish_led_states()
        return response

def main(args = None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
