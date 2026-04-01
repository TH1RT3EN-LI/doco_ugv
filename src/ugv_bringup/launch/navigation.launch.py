import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution

from ugv_bringup.launch_helpers import default_nav2_params_path, resolve_default_map_yaml


def generate_launch_description():
    robot_prefix = "ugv_"
    camera_depth_optical_frame = f"{robot_prefix}camera_depth_optical_frame"

    bringup_share = get_package_share_directory("ugv_bringup")
    orbbec_camera_share = get_package_share_directory("orbbec_camera")
    default_map_yaml = resolve_default_map_yaml(bringup_share)
    default_nav2_params_file = default_nav2_params_path(bringup_share)

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    autostart = LaunchConfiguration("autostart")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_depth_camera = LaunchConfiguration("use_depth_camera")
    depth_camera_launch_file = LaunchConfiguration("depth_camera_launch_file")
    depth_camera_name = LaunchConfiguration("depth_camera_name")
    depth_camera_serial_number = LaunchConfiguration("depth_camera_serial_number")
    depth_camera_usb_port = LaunchConfiguration("depth_camera_usb_port")
    log_level = LaunchConfiguration("log_level")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    rviz_config = LaunchConfiguration("rviz_config")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "base.launch.py")),
        launch_arguments={
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

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "nav2.launch.py")),
        launch_arguments={
            "use_rviz": use_rviz,
            "autostart": autostart,
            "map": map_yaml,
            "map_frame": ugv_map_frame,
            "params_file": params_file,
            "log_level": log_level,
            "auto_initial_pose": auto_initial_pose,
            "rviz_config": rviz_config,
        }.items(),
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([orbbec_camera_share, "launch", depth_camera_launch_file])),
        launch_arguments={
            "camera_name": depth_camera_name,
            "serial_number": depth_camera_serial_number,
            "usb_port": depth_camera_usb_port,
            "enable_depth": "true",
            "enable_color": "false",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "false",
            "publish_tf": "false",
            "cloud_frame_id": camera_depth_optical_frame,
            "log_level": "info",
        }.items(),
        condition=IfCondition(use_depth_camera),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="false"),
            DeclareLaunchArgument(
                "use_depth_camera",
                default_value=EnvironmentVariable("UGV_USE_DEPTH_CAMERA", default_value="false"),
            ),
            DeclareLaunchArgument(
                "depth_camera_launch_file",
                default_value=EnvironmentVariable("UGV_DEPTH_CAMERA_LAUNCH_FILE", default_value="astra_pro_plus.launch.py"),
            ),
            DeclareLaunchArgument("depth_camera_name", default_value="ugv/camera"),
            DeclareLaunchArgument("depth_camera_serial_number", default_value=""),
            DeclareLaunchArgument("depth_camera_usb_port", default_value=""),
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
            DeclareLaunchArgument("map", default_value=default_map_yaml),
            DeclareLaunchArgument("params_file", default_value=default_nav2_params_file),
            DeclareLaunchArgument("rviz_config", default_value=os.path.join(bringup_share, "config", "rviz", "navigation.rviz")),
            base_launch,
            depth_camera_launch,
            nav_launch,
        ]
    )
