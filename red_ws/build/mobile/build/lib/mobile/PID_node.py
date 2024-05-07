from Function import PositionController

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray

class MobileNode(Node):

    def __init__(self):
        super().__init__("mobile_node")
        self.publisher_ = self.create_publisher(Float32MultiArray, "vel_data", 10)
        self.subscription_ = self.create_subscription(Float32MultiArray, "pos_data" ,self.listener_callback, 10)
        self.subscription_  # prevent unused variable warning
        timer_period = 0.01  # 100 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vx, self.vy, self.vz = 0, 0, 0
        self.pos_x, self.pos_y, self.pos_z = 0,0,0
        self.pos_control = PositionController()
        
    def listener_callback(self, msg):
        self.pos_x = msg.data[0]
        self.pos_y = msg.data[1]
        self.pos_z = msg.data[2]

    def timer_callback(self):        
        self.pos_control.go_to_position(5,0,0)
        msg = Float32MultiArray()
        msg.data = [self.vx,self.vy,self.vz]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    mobile_node = MobileNode()

    rclpy.spin(mobile_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mobile_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
