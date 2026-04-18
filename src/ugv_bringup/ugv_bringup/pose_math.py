import math

from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = 0.5 * yaw
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(half_yaw)
    quaternion.w = math.cos(half_yaw)
    return quaternion


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        (quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_planar(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        (cos_yaw * x) - (sin_yaw * y),
        (sin_yaw * x) + (cos_yaw * y),
    )


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
