import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder
from sensor_msgs.msg import LaserScan
import time

from YDLidarX2_python.LidarX2 import LidarX2

class Laser(Node):

    def __init__(self):
        super().__init__('laser')
        self.publisher_ = self.create_publisher(LaserScan, '/laser/scan', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar = LidarX2("/dev/ttyUSB0")
        self.i = 0
        if not self.lidar.open():
            print("Cannot open lidar")


    def timer_callback(self):
        lidar_measurements = self.lidar.getMeasures()
        if len(lidar_measurements) > 0:
            distances = []
            angles = []
            for point in lidar_measurements:
                angles.append(point.angle)
                distances.append(0.001 * point.distance)
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'laser'
            msg.angle_min = -3.1415
            msg.angle_max = 3.1415
            msg.angle_increment = 3.1415/len(angles)
            msg.time_increment = 0.0
            msg.scan_time = 0.25
            msg.range_min = 0.0 
            msg.range_max = 32.0
            msg.ranges = distances
            print(distances)
            #self.i += 1
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    laser_node = Laser()

    rclpy.spin(laser_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
