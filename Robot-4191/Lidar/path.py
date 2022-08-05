import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from std_msgs.msg import Header
import numpy as np
import time
import pyastar2d

'''
Path planner
Subscribes:  
- X Pose
- X Goal
- Occupancy
Publishes:
- Path
'''


class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.publisher_ = self.create_publisher(Path, 'path', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map/occupancy',
            self.listener_callback,
            10)
        # self.subscription
        timer_period = 5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = [0.0, 0.0, 0.0]
        self.goal = [1.0, 1.0]
        self.path = []
        self.timeout = 10
        self.path = Path()

    def listener_callback(self, msg):
        print('msg received')
        map = msg.data  # type: class: array.array
        size = int(np.shape(map)[0] ** 0.5)
        map_arr = np.reshape(map, (size, size))
        print(type(map), np.shape(map), map_arr.shape)
        t_0 = time.time()
        #waypoints = astar(map_arr, (0, 0), (50, 151), self.timeout)
        waypoints = pyastar2d.astar_path(np.float32(map_arr), (0, 0), (50, 151), allow_diagonal=True)
        if len(waypoints.shape) < 2:
            print('Invalid path')
        else:
            print('Path calculated in: ', time.time() - t_0)
            self.generate_path(waypoints)
            print('PATH: ', len(self.path.poses))
            self.timer_callback()

    def timer_callback(self):
        print('pub_path')
        self.publisher_.publish(self.path)
    
    def generate_path(self, waypoints):
        # waypoints = [(i,2*i) for i in range(5)]
        new_path = Path()
        new_path.header.frame_id = 'map'
        for way in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = float(0.05 * way[0])
            pose_stamped.pose.position.y = float(0.05 * way[1])
            pose_stamped.pose.position.z = float(0.0)

            new_path.poses.append(pose_stamped)
        self.path = new_path

def main(args=None):
    rclpy.init(args=args)

    path_planner = PathPlanner()
    rclpy.spin(path_planner)

    path_planner.destroy_node()
    rclpy.shutdown()


def astar(maze, start, end, timeout=0.2):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""
    print(maze)

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = []

    # Add the start node
    open_list.append(start_node)

    t_0 = time.time()
    # Loop until you find the end
    while len(open_list) > 0:
        if time.time() - t_0 > timeout:
            print('timeout', time.time() - t_0)
            return []

        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.append(current_node)

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]  # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (
                    len(maze[len(maze) - 1]) - 1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[node_position[0]][node_position[1]] >= 100:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:

            # Child is on the closed list
            for closed_child in closed_list:
                if child == closed_child:
                    continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                    (child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position


if __name__ == '__main__':
    main()
