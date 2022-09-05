import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from config import map_dimension


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('goal_pub')
        self.publisher_SPAM = self.create_publisher(PoseStamped, '/goal', 10)
        self.publisher_Controller = self.create_publisher(PoseStamped, '/Controller/goal', 10)
        self.goal()

    def goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        while True:
            x = float(input("Print goal X in metres: "))
            y = float(input("Print goal Y in metres: "))
            path = bool(input("Generate Path? "))
            
            msg.pose.position.x = x #- 0.025 #goal - offset - contreller goal 
            msg.pose.position.y = y #- 0.05
            if path:
                self.publisher_SPAM.publish(msg)
            else:
                self.publisher_Controller.publish(msg)
            print('Publishing goal', msg.pose.position)


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
