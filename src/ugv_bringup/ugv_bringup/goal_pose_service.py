from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from ugv_bringup.pose_math import normalize_angle, yaw_to_quaternion
from ugv_bringup_interfaces.srv import GoToPose


def resolve_goal_from_absolute_xy_yaw(
    *,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
    normalize_goal_yaw: bool,
) -> tuple[float, float, float]:
    resolved_goal_yaw = float(goal_yaw)
    if normalize_goal_yaw:
        resolved_goal_yaw = normalize_angle(resolved_goal_yaw)

    return float(goal_x), float(goal_y), resolved_goal_yaw


class GoalPoseServiceNode(Node):
    def __init__(self) -> None:
        super().__init__("goal_pose_service")

        self.service_name = str(
            self.declare_parameter(
                "service_name", "/ugv/navigation/go_to_pose"
            ).value
        )
        self.goal_topic = str(
            self.declare_parameter("goal_topic", "/ugv/goal_pose").value
        )
        self.map_frame = str(
            self.declare_parameter("map_frame", "ugv_map").value
        )
        self.normalize_goal_yaw = bool(
            self.declare_parameter("normalize_goal_yaw", True).value
        )

        self.goal_publisher = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.service = self.create_service(
            GoToPose,
            self.service_name,
            self._handle_go_to_pose,
        )

        self.get_logger().info(
            "goal_pose_service ready: service=%s, goal_topic=%s, map_frame=%s, normalize_goal_yaw=%s"
            % (
                self.service_name,
                self.goal_topic,
                self.map_frame,
                "true" if self.normalize_goal_yaw else "false",
            )
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

    def _handle_go_to_pose(self, request, response):
        goal_x, goal_y, goal_yaw = resolve_goal_from_absolute_xy_yaw(
            goal_x=float(request.x),
            goal_y=float(request.y),
            goal_yaw=float(request.yaw),
            normalize_goal_yaw=self.normalize_goal_yaw,
        )

        self.goal_publisher.publish(
            self._build_goal_pose(goal_x=goal_x, goal_y=goal_y, goal_yaw=goal_yaw)
        )

        response.success = True
        response.message = (
            f"Published goal pose to {self.goal_topic} "
            f"(x={goal_x:.3f}, y={goal_y:.3f}, yaw={goal_yaw:.3f})"
        )
        response.goal_frame_id = self.map_frame
        response.goal_x = goal_x
        response.goal_y = goal_y
        response.goal_yaw = goal_yaw

        self.get_logger().info(
            "Published absolute goal: %s frame goal=(%.3f, %.3f, yaw=%.3f)"
            % (
                self.map_frame,
                goal_x,
                goal_y,
                goal_yaw,
            )
        )
        return response


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = GoalPoseServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
