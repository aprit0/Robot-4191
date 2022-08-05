from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import math

def to_odometry(odom):
    '''
    Sample:
    odom = {}
    odom['x'] = 0
    odom['y'] = 0
    odom['dy'] = 0
    odom['theta'] = 0
    odom['dtheta'] = 0
    '''
    quaternion = Quaternion()
    quaternion.z = math.sin(odom['theta'] / 2.0)
    quaternion.w = math.cos(odom['theta'] / 2.0)
    msg = Odometry()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = odom['x']
    msg.pose.pose.position.y = odom['y']
    msg.pose.pose.orientation = quaternion
    msg.child_frame_id = "base_link"
    msg.twist.twist.linear.x = odom['dx']
    msg.twist.twist.angular.z = odom['dtheta']
    return msg


def from_odometry(msg):
    odom = {}
    odom['x'] = msg.pose.pose.position.x
    odom['y'] = msg.pose.pose.position.y
    quat = msg.pose.pose.orientation
    euler = math.atan2(2.0*(quat.w * quat.z),
                       1.0 - 2.0 * (quat.z **2))
    odom['theta'] = euler
    odom['dx'] = msg.twist.twist.linear.x
    odom['dtheta'] = msg.twist.twist.angular.z
    return odom