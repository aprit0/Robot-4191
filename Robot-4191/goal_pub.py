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
        msg1 = PoseStamped()
        msg1.header.frame_id = 'map'

        msg2 = PoseStamped()
        msg2.header.frame_id = 'map'

        while True:
            #first waypoint direct path
            x1 = float(input("Print first goal X in metres: "))
            y1 = float(input("Print first goal Y in metres: "))
            msg1.pose.position.x = x1 #- 0.025 #goal - offset - controller goal
            msg1.pose.position.y = y1 #- 0.05

            #second waypoint with path planning
            x2 = float(input("Print second goal X in metres: "))
            y2 = float(input("Print second goal Y in metres: "))
            msg2.pose.position.x = x2
            msg2.pose.position.y = y2
            self.publisher_Controller.publish(msg1)
            self.publisher_SPAM.publish(msg2)


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
