import rclpy
from rclpy.node import Node
from Utils.utils import to_odometry
from nav_msgs.msg import Odometry


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Odometry, '/robot/odom', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.

    def timer_callback(self):
        self.i = self.i if self.i < 10 else 0.0
        odom = {}
        odom['x'] = self.i
        odom['y'] = self.i
        odom['dx'] = self.i*0.1
        odom['dy'] = self.i*0.1
        odom['theta'] = self.i*0.1
        odom['dtheta'] = 0.
        msg = to_odometry(odom)
        self.publisher_.publish(msg)
        self.i += 1.0


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
