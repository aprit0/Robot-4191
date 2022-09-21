import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray
import matplotlib.pyplot as plt


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'video_frames',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        frame = msg.data
        frame = np.reshape(frame,(480, 640, 3))
        plt.imshow(frame)
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
