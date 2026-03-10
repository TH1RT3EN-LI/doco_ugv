import geometry_msgs.msg
import rcl_interfaces.msg

from .keyboard_teleop_core import KeyboardTeleopCore, TeleopCommand


COMMON_KEYBOARD_MESSAGE = """
当前按键布局：
   Q    W    E
   A    S    D
        X

W/S : 前进/后退 (线速度 x)
A/D : 左移/右移 (线速度 y)
Q/E : 左转/右转 (角速度 z)
X   : 紧急停止

I/K : 线速度 增加/减少（固定步长）
O/L : 角速度 增加/减少（固定步长）
"""


class TwistPublisherAdapter:
    def __init__(self, node, publisher, twist_msg, twist, stamped: bool):
        self._node = node
        self._publisher = publisher
        self._twist_msg = twist_msg
        self._twist = twist
        self._stamped = bool(stamped)

    def publish(self, command: TeleopCommand) -> None:
        if self._stamped:
            self._twist_msg.header.stamp = self._node.get_clock().now().to_msg()

        self._twist.linear.x = command.linear_x
        self._twist.linear.y = command.linear_y
        self._twist.linear.z = command.linear_z
        self._twist.angular.x = 0.0
        self._twist.angular.y = 0.0
        self._twist.angular.z = command.angular_z
        self._publisher.publish(self._twist_msg)

    def publish_zero(self, core: KeyboardTeleopCore | None = None) -> None:
        command = TeleopCommand(
            linear_x=0.0,
            linear_y=0.0,
            linear_z=0.0,
            angular_z=0.0,
            linear_speed=0.0 if core is None else core.status()["linear_speed"],
            angular_speed=0.0 if core is None else core.status()["angular_speed"],
            active_keys=tuple(),
        )
        self.publish(command)


def declare_common_parameters(node) -> dict[str, object]:
    read_only_descriptor = rcl_interfaces.msg.ParameterDescriptor(read_only=True)
    params = {
        "stamped": node.declare_parameter("stamped", False, read_only_descriptor).value,
        "frame_id": node.declare_parameter("frame_id", "", read_only_descriptor).value,
        "linear_speed": node.declare_parameter("linear_speed", 0.5, read_only_descriptor).value,
        "angular_speed": node.declare_parameter("angular_speed", 1.0, read_only_descriptor).value,
        "speed_step": node.declare_parameter("speed_step", 0.05, read_only_descriptor).value,
        "turn_step": node.declare_parameter("turn_step", 0.1, read_only_descriptor).value,
        "cmd_vel_topic": node.declare_parameter(
            "cmd_vel_topic",
            "/ugv/cmd_vel_keyboard",
            read_only_descriptor,
        ).value,
        "publish_rate": node.declare_parameter("publish_rate", 60.0, read_only_descriptor).value,
        "accel_limit_linear": node.declare_parameter("accel_limit_linear", 2.0, read_only_descriptor).value,
        "decel_limit_linear": node.declare_parameter("decel_limit_linear", 3.0, read_only_descriptor).value,
        "accel_limit_angular": node.declare_parameter("accel_limit_angular", 6.0, read_only_descriptor).value,
        "decel_limit_angular": node.declare_parameter("decel_limit_angular", 8.0, read_only_descriptor).value,
        "idle_timeout_sec": node.declare_parameter("idle_timeout_sec", 0.25, read_only_descriptor).value,
    }
    if not params["stamped"] and params["frame_id"]:
        raise ValueError("'frame_id' can only be set when 'stamped' is True")
    return params


def create_core_from_params(params: dict[str, object]) -> KeyboardTeleopCore:
    return KeyboardTeleopCore(
        linear_speed=float(params["linear_speed"]),
        angular_speed=float(params["angular_speed"]),
        speed_step=float(params["speed_step"]),
        turn_step=float(params["turn_step"]),
        accel_limit_linear=float(params["accel_limit_linear"]),
        decel_limit_linear=float(params["decel_limit_linear"]),
        accel_limit_angular=float(params["accel_limit_angular"]),
        decel_limit_angular=float(params["decel_limit_angular"]),
        idle_timeout_sec=float(params["idle_timeout_sec"]),
    )


def create_twist_publisher_adapter(node, params: dict[str, object]) -> TwistPublisherAdapter:
    stamped = bool(params["stamped"])
    frame_id = str(params["frame_id"])
    if stamped:
        twist_msg = geometry_msgs.msg.TwistStamped()
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
        twist = twist_msg.twist
        publisher = node.create_publisher(geometry_msgs.msg.TwistStamped, params["cmd_vel_topic"], 10)
    else:
        twist_msg = geometry_msgs.msg.Twist()
        twist = twist_msg
        publisher = node.create_publisher(geometry_msgs.msg.Twist, params["cmd_vel_topic"], 10)
    return TwistPublisherAdapter(node, publisher, twist_msg, twist, stamped)
