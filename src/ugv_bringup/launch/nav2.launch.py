import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from ugv_bringup.launch_helpers import cleanup_temp_dir, create_merged_nav2_params, resolve_default_map_yaml


def generate_launch_description():
    bringup_share = get_package_share_directory("ugv_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    config_dir = os.path.join(bringup_share, "config")
    default_map_yaml = resolve_default_map_yaml(bringup_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    params_file = LaunchConfiguration("params_file")
    config_preset = LaunchConfiguration("config_preset")
    config_overlay = LaunchConfiguration("config_overlay")
    resolved_params_file = LaunchConfiguration("resolved_params_file")
    generated_config_dir = LaunchConfiguration("generated_config_dir")
    map_yaml = LaunchConfiguration("map")
    map_frame = LaunchConfiguration("map_frame")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "navigation.rviz")

    def prepare_nav2_params(context):
        temp_dir, merged_path = create_merged_nav2_params(
            config_dir=config_dir,
            config_preset=config_preset.perform(context),
            params_file=params_file.perform(context),
            config_overlay=config_overlay.perform(context),
        )
        return [
            SetLaunchConfiguration("generated_config_dir", temp_dir),
            SetLaunchConfiguration("resolved_params_file", merged_path),
        ]

    prepare_nav2_params_action = OpaqueFunction(function=prepare_nav2_params)

    def cleanup_generated_params(context, *args, **kwargs):
        cleanup_temp_dir(generated_config_dir.perform(context))
        return []

    cleanup_generated_params_action = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_generated_params)])
    )

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
                    "params_file": resolved_params_file,
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
        package="ugv_bringup",
        executable="initial_pose_publisher",
        name="initial_pose_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time_param},
            {"topic": "/ugv/initialpose"},
            {"frame_id": map_frame},
            {"x": 0.0, "y": 0.0, "yaw": 0.0},
            {"delay_sec": 2.0, "publish_count": 10, "publish_period_sec": 0.2},
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
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("config_preset", default_value="hw"),
            DeclareLaunchArgument("config_overlay", default_value=""),
            DeclareLaunchArgument("resolved_params_file", default_value=""),
            DeclareLaunchArgument("generated_config_dir", default_value=""),
            DeclareLaunchArgument("map", default_value=default_map_yaml),
            DeclareLaunchArgument("map_frame", default_value="ugv_map"),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            prepare_nav2_params_action,
            cleanup_generated_params_action,
            nav2_launch,
            rviz_launch,
            initial_pose_pub,
        ]
    )
