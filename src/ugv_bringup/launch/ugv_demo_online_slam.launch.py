import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def next_bag_run_id(base_dir: Path, prefix: str) -> int:
    if not base_dir.exists():
        return 1

    max_id = 0
    stem = f"{prefix}_"
    for entry in base_dir.iterdir():
        if not entry.name.startswith(stem):
            continue

        suffix = entry.name[len(stem) :]
        run_id, _, _ = suffix.partition("_")
        if run_id.isdigit():
            max_id = max(max_id, int(run_id))

    return max_id + 1


def generate_launch_description():
    bringup_share = get_package_share_directory("ugv_bringup")
    relative_position_fusion_share = get_package_share_directory("relative_position_fusion")

    default_nav2_params_file = os.path.join(
        bringup_share, "config", "nav2_online_slam.yaml"
    )
    default_rviz_config = os.path.join(
        bringup_share, "config", "rviz", "online_navigation.rviz"
    )
    default_relative_fusion_config = os.path.join(
        bringup_share, "config", "relative_position_fusion_ugv_demo.yaml"
    )

    bag_root_dir = Path.home() / "ugv_bags"
    default_bag_id = next_bag_run_id(bag_root_dir, "ugv_demo_online_slam")
    default_bag_output_dir = str(
        bag_root_dir
        / f"ugv_demo_online_slam_{default_bag_id:03d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    log_level = LaunchConfiguration("log_level")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    enable_relative_position_fusion = LaunchConfiguration(
        "enable_relative_position_fusion"
    )
    relative_position_fusion_config = LaunchConfiguration(
        "relative_position_fusion_config"
    )
    uav_body_frame = LaunchConfiguration("uav_body_frame")
    uav_odom_topic = LaunchConfiguration("uav_odom_topic")
    uav_twist_in_child_frame = LaunchConfiguration("uav_twist_in_child_frame")
    uav_tag_detection_topic = LaunchConfiguration("uav_tag_detection_topic")
    uav_tag_pose_topic = LaunchConfiguration("uav_tag_pose_topic")
    use_record_bag = LaunchConfiguration("use_record_bag")
    bag_output_dir = LaunchConfiguration("bag_output_dir")

    online_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "online_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": use_rviz,
            "autostart": autostart,
            "use_foxglove": use_foxglove,
            "log_level": log_level,
            "odom0": odom0,
            "imu0": imu0,
            "params_file": params_file,
            "rviz_config": rviz_config,
            "global_frame": global_frame,
            "ugv_map_frame": ugv_map_frame,
            "global_to_ugv_map": global_to_ugv_map,
            "publish_global_map_tf": publish_global_map_tf,
        }.items(),
    )

    relative_position_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                relative_position_fusion_share,
                "launch",
                "relative_position_fusion.launch.py",
            )
        ),
        condition=IfCondition(enable_relative_position_fusion),
        launch_arguments={
            "preset": "",
            "use_sim_time": use_sim_time,
            "global_frame": global_frame,
            "uav_body_frame": uav_body_frame,
            "uav_odom_topic": uav_odom_topic,
            "uav_twist_in_child_frame": uav_twist_in_child_frame,
            "enable_relative_tracking": "false",
            "config_overlay": relative_position_fusion_config,
        }.items(),
    )

    bag_record_topics = [
        "/map",
        "/map_updates",
        "/ugv/scan",
        "/ugv/odom",
        "/ugv/imu",
        "/ugv/odometry/filtered",
        "/ugv/cmd_vel_nav",
        "/ugv/cmd_vel_safe",
        "/ugv/goal_pose",
        "/ugv/initialpose",
        "/ugv/robot_description",
        uav_odom_topic,
        uav_tag_detection_topic,
        uav_tag_pose_topic,
        "/ugv/relative_position/estimate/global",
        "/ugv/relative_position/estimate/uav_body",
        "/ugv/relative_position/debug/relative_velocity",
        "/ugv/relative_position/relocalize_requested",
        "/ugv/relative_position/diagnostics",
        "/tf",
        "/tf_static",
    ]

    record_bag = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--output", bag_output_dir, *bag_record_topics],
        output="screen",
        condition=IfCondition(use_record_bag),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("odom0", default_value="/ugv/odom"),
            DeclareLaunchArgument("imu0", default_value="/ugv/imu"),
            DeclareLaunchArgument(
                "params_file", default_value=default_nav2_params_file
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument("publish_global_map_tf", default_value="true"),
            DeclareLaunchArgument(
                "enable_relative_position_fusion", default_value="true"
            ),
            DeclareLaunchArgument(
                "relative_position_fusion_config",
                default_value=default_relative_fusion_config,
            ),
            DeclareLaunchArgument("uav_body_frame", default_value="uav_base_link"),
            DeclareLaunchArgument(
                "uav_odom_topic", default_value="/uav/state/odometry_px4"
            ),
            DeclareLaunchArgument("uav_twist_in_child_frame", default_value="true"),
            DeclareLaunchArgument(
                "uav_tag_detection_topic",
                default_value="/uav/visual_landing/apriltag_detection",
            ),
            DeclareLaunchArgument(
                "uav_tag_pose_topic",
                default_value="/uav/visual_landing/apriltag_pose",
            ),
            DeclareLaunchArgument("use_record_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_output_dir", default_value=default_bag_output_dir
            ),
            online_navigation_launch,
            relative_position_fusion_launch,
            record_bag,
        ]
    )
