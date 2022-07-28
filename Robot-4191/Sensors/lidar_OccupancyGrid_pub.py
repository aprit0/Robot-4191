import rclpy
from rclpy.node import Node
from gpiozero import RotaryEncoder
from nav_msgs import OccupancyGrid
import time

import YLidarX2_python.lidar_to_grid_map as lg
from YDLidarX2_python.LidarX2 import LidarX2
# https://answers.ros.org/question/286221/create-2d-occupancy-grid-map-by-laser-data/
class Laser(Node):

    def __init__(self):
        super().__init__('laser')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/laser/scan', 10)
        timer_period = 0.25  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.lidar = LidarX2("/dev/ttyUSB0")
        self.i = 0
        if not self.lidar.open():
            print("Cannot open lidar")


    def timer_callback(self)::w

        ang, dist = self.read_variables()
        if len(ang) > 0:
            ox = np.sin(ang) * dist
            oy = np.cos(ang) * dist
            map_size = [-10,10]
            occupancy_map, min_x, max_x, min_y, max_y, xy_resolution,center_x,clearPoints = lg.generate_ray_casting_grid_map(
        ox, oy, xy_resolution, True,map_size = map_size)
            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.info.resolution = resolution
            msg.info.width = math.ceil(map_size[1] / resolution)
            msg.info.height = math.ceil(map_size[1] / resolution)
            msg.info.origin.position.x= map_size[1]/2
            msg.info.origin.position.y= map_size[1]/2 
            msg.data = occupancy_map

            self.publisher_.publish(msg)

    def read_variables(self):
        laserScan = self.lidar.getMeasures()
        angles = []
        distances = []
        for point in laserScan:
            if point[1] > 0:
                angles.append(float(point[0])*3.1415/180)
                distances.append(float(point[1])/1000)
        angles = np.array(angles)
        distances = np.array(distances)
        return angles, distances
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
