import rclpy
from rclpy.node import Node

from std_msgs.msg import float, Float32MultiArray, MultiArrayDimension

#Program
import tracking


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(float, "pid_data", 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lt =  tracking.LineTracker()

    def timer_callback(self):
        PID_out = self.lt.PID_Read()
        Con_state = self.lt.condition_met()
        
        msg = Float32MultiArray()
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=3, stride=3))
        msg.layout.dim.append(MultiArrayDimension(label='columns', size=1, stride=1))
        msg.data = [PID_out,0,0]
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
