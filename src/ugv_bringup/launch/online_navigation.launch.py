import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("ugv_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_params_file = os.path.join(bringup_share, "config", "nav2_online_slam_tuned.yaml")
    default_rviz_config = os.path.join(bringup_share, "config", "rviz", "online_navigation.rviz")
    lattice_filepath = os.path.join(
        bringup_share,
        "config",
        "lattice_primitives",
        "ugv_omni_2cm_rich_lattice.json",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    log_level = LaunchConfiguration("log_level")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    params_file = LaunchConfiguration("params_file")
    effective_params_file = LaunchConfiguration("effective_params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")

    rewritten_params = RewrittenYaml(
        source_file=effective_params_file,
        param_rewrites={"lattice_filepath": lattice_filepath},
        convert_types=True,
    )

    def resolve_effective_params_file(context):
        resolved_params_file = params_file.perform(context).strip() or default_params_file
        return [SetLaunchConfiguration("effective_params_file", resolved_params_file)]

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "base.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_foxglove": use_foxglove,
            "use_lidar": "true",
            "odom0": odom0,
            "imu0": imu0,
            "global_frame": global_frame,
            "ugv_map_frame": ugv_map_frame,
            "global_to_ugv_map": global_to_ugv_map,
            "publish_global_map_tf": publish_global_map_tf,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "slam.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    nav2_navigation_launch = GroupAction(
        actions=[
            SetRemap(src="cmd_vel", dst="/ugv/cmd_vel_nav"),
            SetRemap(src="/cmd_vel", dst="/ugv/cmd_vel_nav"),
            SetRemap(src="goal_pose", dst="/ugv/goal_pose"),
            SetRemap(src="/goal_pose", dst="/ugv/goal_pose"),
            SetRemap(src="clicked_point", dst="/ugv/clicked_point"),
            SetRemap(src="/clicked_point", dst="/ugv/clicked_point"),
            SetRemap(src="move_base_simple/goal", dst="/ugv/goal_pose"),
            SetRemap(src="/move_base_simple/goal", dst="/ugv/goal_pose"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "navigation_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": rewritten_params,
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("odom0", default_value="/ugv/odom"),
            DeclareLaunchArgument("imu0", default_value="/ugv/imu"),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument("publish_global_map_tf", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("effective_params_file", default_value=default_params_file),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            OpaqueFunction(function=resolve_effective_params_file),
            base_launch,
            slam_launch,
            nav2_navigation_launch,
            rviz_launch,
        ]
    )
