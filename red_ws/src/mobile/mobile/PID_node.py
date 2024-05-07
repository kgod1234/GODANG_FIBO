import Function

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MobileNode(Node):

    def __init__(self):
        super().__init__("mobile_node")
        self.publisher_ = self.create_publisher(String, "vel_data", 10)
        self.subscriptions_ = self.create_subscription(String, "pos_data" , 10)
        timer_period = 0.01  # 100 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        # PID_out = self.lt.PID_Read()
        # Con_state = self.lt.condition_met()
        
        msg = Float32MultiArray()
        msg.data = [1.0,2.5, 3.9]
        # msg.layout.dim.append(MultiArrayDimension(label='rows', size=3, stride=3))
        # msg.layout.dim.append(MultiArrayDimension(label='columns', size=1, stride=1))
        # msg.data = [PID_out,0,0]
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
    
