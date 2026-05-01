"""Transformaciones geométricas robot ↔ mapa."""
import math
from geometry_msgs.msg import PoseStamped, Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    return q


def make_pose_stamped(x: float, y: float, yaw: float,
                      frame_id: str = 'map') -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = quaternion_from_yaw(yaw)
    return pose
