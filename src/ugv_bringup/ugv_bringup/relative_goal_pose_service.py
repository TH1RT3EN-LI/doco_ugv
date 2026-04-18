import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from ugv_bringup.pose_math import (
    normalize_angle,
    rotate_planar,
    yaw_from_quaternion,
    yaw_to_quaternion,
)
from ugv_bringup_interfaces.srv import GoRelativeXY


def resolve_goal_from_relative_xy(
    *,
    current_x: float,
    current_y: float,
    current_yaw: float,
    relative_x: float,
    relative_y: float,
    align_yaw_to_motion: bool,
) -> tuple[float, float, float]:
    delta_x, delta_y = rotate_planar(relative_x, relative_y, current_yaw)
    goal_x = current_x + delta_x
    goal_y = current_y + delta_y

    goal_yaw = current_yaw
    if align_yaw_to_motion and (
        abs(relative_x) > 1.0e-6 or abs(relative_y) > 1.0e-6
    ):
        goal_yaw = normalize_angle(current_yaw + math.atan2(relative_y, relative_x))

    return goal_x, goal_y, goal_yaw


class RelativeGoalPoseServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("relative_goal_pose_service")

        self.service_name = str(
            self.declare_parameter(
                "service_name", "/ugv/navigation/go_relative_xy"
            ).value
        )
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/ugv/goal_pose").value
        )
        self.map_frame = str(
            self.declare_parameter("map_frame", "ugv_map").value
        )
        self.base_frame = str(
            self.declare_parameter("base_frame", "ugv_base_footprint").value
        )
        self.transform_timeout_sec = max(
            0.0, float(self.declare_parameter("transform_timeout_sec", 0.3).value)
        )
        self.align_yaw_to_motion = bool(
            self.declare_parameter("align_yaw_to_motion", False).value
        )

        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.service = self.create_service(
            GoRelativeXY,
            self.service_name,
            self._handle_go_relative_xy,
        )

        self.get_logger().info(
            "relative_goal_pose_service ready: service=%s, goal_topic=%s, map_frame=%s, base_frame=%s, align_yaw_to_motion=%s"
            % (
                self.service_name,
                self.goal_topic,
                self.map_frame,
                self.base_frame,
                "true" if self.align_yaw_to_motion else "false",
            )
        )

    def _lookup_current_pose(self):
        timeout = Duration(seconds=self.transform_timeout_sec)
        transform = self.tf_buffer.lookup_transform(
            self.map_frame,
            self.base_frame,
            Time(),
            timeout=timeout,
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            yaw_from_quaternion(rotation),
        )

    def _build_goal_pose(
        self,
        *,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> PoseStamped:
        goal_pose = PoseStamped()
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation = yaw_to_quaternion(goal_yaw)
        return goal_pose

    def _handle_go_relative_xy(self, request, response):
        try:
            current_x, current_y, current_yaw = self._lookup_current_pose()
        except TransformException as exc:
            response.success = False
            response.message = (
                f"Failed to resolve current pose from {self.map_frame} to {self.base_frame}: {exc}"
            )
            response.goal_frame_id = self.map_frame
            response.goal_x = 0.0
            response.goal_y = 0.0
            response.goal_yaw = 0.0
            self.get_logger().warning(response.message)
            return response

        goal_x, goal_y, goal_yaw = resolve_goal_from_relative_xy(
            current_x=current_x,
            current_y=current_y,
            current_yaw=current_yaw,
            relative_x=float(request.x),
            relative_y=float(request.y),
            align_yaw_to_motion=self.align_yaw_to_motion,
        )

        self.goal_publisher.publish(
            self._build_goal_pose(goal_x=goal_x, goal_y=goal_y, goal_yaw=goal_yaw)
        )

        response.success = True
        response.message = (
            f"Published goal pose to {self.goal_topic} from relative body delta "
            f"(x={request.x:.3f}, y={request.y:.3f})"
        )
        response.goal_frame_id = self.map_frame
        response.goal_x = float(goal_x)
        response.goal_y = float(goal_y)
        response.goal_yaw = float(goal_yaw)

        self.get_logger().info(
            "Published relative goal: body_delta=(%.3f, %.3f) -> %s frame goal=(%.3f, %.3f, yaw=%.3f)"
            % (
                request.x,
                request.y,
                self.map_frame,
                goal_x,
                goal_y,
                goal_yaw,
            )
        )
        return response


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = RelativeGoalPoseServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
