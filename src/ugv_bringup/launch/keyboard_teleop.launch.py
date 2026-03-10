import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    teleop_share = get_package_share_directory("ugv_teleop")
    keyboard_config_path = os.path.join(teleop_share, "config", "keyboard_teleop.yaml")
    try:
        launch_tty_device = os.ttyname(0)
    except OSError:
        launch_tty_device = ""

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    keyboard_backend = LaunchConfiguration("keyboard_backend")

    gui_node = Node(
        package="ugv_teleop",
        executable="keyboard_gui_teleop_node",
        name="keyboard_gui_teleop_node",
        parameters=[keyboard_config_path, {"use_sim_time": use_sim_time_param}],
        output="screen",
        condition=IfCondition(PythonExpression(['"', keyboard_backend, '" == "gui"'])),
    )

    tty_node = Node(
        package="ugv_teleop",
        executable="keyboard_tty_teleop_node",
        name="keyboard_tty_teleop_node",
        parameters=[
            keyboard_config_path,
            {
                "use_sim_time": use_sim_time_param,
                "tty_device_path": launch_tty_device,
            },
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['"', keyboard_backend, '" == "tty"'])),
    )

    evdev_legacy_node = Node(
        package="ugv_teleop",
        executable="keyboard_teleop_node",
        name="keyboard_teleop_node",
        parameters=[keyboard_config_path, {"use_sim_time": use_sim_time_param}],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(PythonExpression(['"', keyboard_backend, '" == "evdev_legacy"'])),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false"),
            ),
            DeclareLaunchArgument("keyboard_backend", default_value="tty"),
            gui_node,
            tty_node,
            evdev_legacy_node,
        ]
    )
