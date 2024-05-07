import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

state = 0;

class StateNode(Node):

    def __init__(self):
        super().__init__("state_node")
        self.publisher_ = self.create_publisher(Int32, "state_data", 10)
        timer_period = 0.01  # 100 hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int32
        msg.data = 0;
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    state_node = StateNode()

    rclpy.spin(state_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
