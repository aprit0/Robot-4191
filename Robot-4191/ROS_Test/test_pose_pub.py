import rclpy
from rclpy.node import Node
from Utils.lidar_utils import to_odometry
import numpy as np
from nav_msgs.msg import Odometry


class TestPose(Node):

    def __init__(self):
        super().__init__('test_pose')
        self.publisher_ = self.create_publisher(Odometry, '/robot/odom', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0.

    def timer_callback(self):
        x = input('x(m): ')
        y = input('y(m): ')
        theta = input('theta(degrees): ')
        odom = {}
        odom['x'] = float(x)
        odom['y'] = float(y)
        odom['dx'] = 0.
        odom['dy'] = 0.
        odom['theta'] = float(theta)*np.pi/180#np.pi/4
        odom['dtheta'] = 0.
        msg = to_odometry(odom)
        self.publisher_.publish(msg)
        print('New_pose: ', odom['x'], odom['y'], odom['theta'])
        
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestPose()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
