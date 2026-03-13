import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from rclpy.node import Node


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = 0.5 * yaw
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(half)
    quaternion.w = math.cos(half)
    return quaternion


def build_conservative_initial_pose_covariance() -> list[float]:
    covariance = [0.0] * 36
    covariance[0] = 0.25
    covariance[7] = 0.25
    covariance[35] = math.radians(10.0) ** 2
    return covariance


class InitialPosePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('initial_pose_publisher')
        self.topic = self.declare_parameter('topic', '/ugv/initialpose').value
        self.amcl_pose_topic = self.declare_parameter('amcl_pose_topic', '/amcl_pose').value
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        self.x = float(self.declare_parameter('x', 0.0).value)
        self.y = float(self.declare_parameter('y', 0.0).value)
        self.yaw = float(self.declare_parameter('yaw', 0.0).value)
        self.delay_sec = max(0.0, float(self.declare_parameter('delay_sec', 1.0).value))
        self.publish_count = max(1, int(self.declare_parameter('publish_count', 5).value))
        self.publish_period_sec = max(0.01, float(self.declare_parameter('publish_period_sec', 0.2).value))
        self.require_subscriber = bool(self.declare_parameter('require_subscriber', True).value)
        self.wait_for_amcl_pose = bool(self.declare_parameter('wait_for_amcl_pose', True).value)
        self.max_wait_sec = float(self.declare_parameter('max_wait_sec', 120.0).value)
        self.status_log_period_sec = max(0.1, float(self.declare_parameter('status_log_period_sec', 2.0).value))

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, self.topic, 10)
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self._on_amcl_pose,
            10,
        )

        self.start_time = time.monotonic()
        self.activation_time = self.start_time + self.delay_sec
        self.last_status_log_time = self.start_time
        self.published_count = 0
        self.done = False
        self.waiting_for_amcl = False
        self.timer = self.create_timer(self.publish_period_sec, self._on_timer)

        self.get_logger().info(
            "initial_pose_publisher armed: will publish to %s in frame '%s' after %.2fs "
            "(x=%.3f, y=%.3f, yaw=%.3f, initial_burst=%d, wait_for_amcl_pose=%s)"
            % (
                self.topic,
                self.frame_id,
                self.delay_sec,
                self.x,
                self.y,
                self.yaw,
                self.publish_count,
                'true' if self.wait_for_amcl_pose else 'false',
            )
        )

    def _on_amcl_pose(self, _: PoseWithCovarianceStamped) -> None:
        if self.done or not self.waiting_for_amcl:
            return
        self._mark_done('AMCL pose received; initial pose accepted, shutting down.')

    def _log_status(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_status_log_time >= self.status_log_period_sec:
            self.get_logger().info(message)
            self.last_status_log_time = now

    def _mark_done(self, message: str) -> None:
        if self.done:
            return
        self.done = True
        self.timer.cancel()
        self.get_logger().info(message)

    def _build_message(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        msg.pose.covariance = build_conservative_initial_pose_covariance()
        return msg

    def _on_timer(self) -> None:
        if self.done:
            return

        now = time.monotonic()
        elapsed_sec = now - self.start_time
        if self.max_wait_sec > 0.0 and elapsed_sec > self.max_wait_sec:
            self._mark_done(
                f'Timed out after {self.max_wait_sec:.1f}s waiting for AMCL to accept the initial pose.'
            )
            return

        if now < self.activation_time:
            return

        if self.require_subscriber and self.count_subscribers(self.topic) == 0:
            self._log_status(f'Waiting for a subscriber on {self.topic} before publishing the initial pose...')
            return

        self.publisher.publish(self._build_message())
        self.published_count += 1

        if self.published_count >= self.publish_count:
            if not self.wait_for_amcl_pose:
                self._mark_done('Initial pose burst published; shutting down.')
                return
            if not self.waiting_for_amcl:
                self.waiting_for_amcl = True
                self.get_logger().info(
                    f'Published initial burst ({self.published_count}x); continuing retries until {self.amcl_pose_topic} arrives.'
                )
            else:
                self._log_status(
                    f'Published {self.published_count} initial pose message(s); still waiting for {self.amcl_pose_topic}...'
                )


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisherNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
