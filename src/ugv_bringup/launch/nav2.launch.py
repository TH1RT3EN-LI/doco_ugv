import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from ugv_bringup.launch_helpers import resolve_default_map_yaml


def generate_launch_description():
    bringup_share = get_package_share_directory("ugv_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_map_yaml = resolve_default_map_yaml(bringup_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    map_frame = LaunchConfiguration("map_frame")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "navigation.rviz")

    nav2_launch = GroupAction(
        actions=[
            SetRemap(src="cmd_vel", dst="/ugv/cmd_vel_nav"),
            SetRemap(src="/cmd_vel", dst="/ugv/cmd_vel_nav"),
            SetRemap(src="initialpose", dst="/ugv/initialpose"),
            SetRemap(src="/initialpose", dst="/ugv/initialpose"),
            SetRemap(src="goal_pose", dst="/ugv/goal_pose"),
            SetRemap(src="/goal_pose", dst="/ugv/goal_pose"),
            SetRemap(src="clicked_point", dst="/ugv/clicked_point"),
            SetRemap(src="/clicked_point", dst="/ugv/clicked_point"),
            SetRemap(src="move_base_simple/goal", dst="/ugv/goal_pose"),
            SetRemap(src="/move_base_simple/goal", dst="/ugv/goal_pose"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "bringup_launch.py")),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )

    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_rviz),
    )

    initial_pose_pub = Node(
        package="ugv_sim_tools",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"topic": "/ugv/initialpose"},
            {"frame_id": map_frame},
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"delay_sec": 2.0, "publish_count": 10, "publish_period_sec": 0.2},
            # AMCL may not expose its subscription immediately during composed startup.
            {"require_subscriber": False},
        ],
        condition=IfCondition(auto_initial_pose),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=os.path.join(bringup_share, "config", "nav2.yaml")),
            DeclareLaunchArgument("map", default_value=default_map_yaml),
            DeclareLaunchArgument("map_frame", default_value="ugv_map"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            nav2_launch,
            rviz_launch,
            initial_pose_pub,
        ]
    )
