import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from config import map_dimension


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('goal_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal', 10)
        self.goal()

    def goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        while True:
            x = float(input("Print goal X in metres: "))
            y = float(input("Print goal Y in metres: "))
            print('Publishing goal')
            msg.pose.position.x = x #-0.07
            msg.pose.position.y = y
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


if __name__ == '__main__':
    main()
