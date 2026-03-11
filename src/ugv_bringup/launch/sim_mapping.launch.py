import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bringup_share = get_package_share_directory("ugv_bringup")
    laser_frame = "ugv_laser"

    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_profile = LaunchConfiguration("sim_profile")
    use_sim_camera = LaunchConfiguration("use_sim_camera")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    use_rviz = LaunchConfiguration("use_rviz")
    use_foxglove = LaunchConfiguration("use_foxglove")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py")),
        launch_arguments={
            "headless": headless,
            "sim_profile": sim_profile,
            "use_sim_camera": use_sim_camera,
            "use_sim_tf": "true",
            "publish_map_tf": "false",
            "global_frame": global_frame,
            "ugv_map_frame": ugv_map_frame,
            "global_to_ugv_map": global_to_ugv_map,
            "publish_global_map_tf": publish_global_map_tf,
            "use_teleop": "true",
            "use_foxglove": use_foxglove,
            "use_rviz": use_rviz,
            "rviz_config": rviz_config,
            "rviz_software_gl": rviz_software_gl,
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "slam.launch.py")),
    )

    scan_rewriter = Node(
        package="ugv_sim_tools",
        executable="scan_frame_rewriter",
        name="scan_frame_rewriter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_topic": "/ugv/scan_raw",
                "output_topic": "/ugv/scan",
                "output_frame_id": laser_frame,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
            DeclareLaunchArgument("sim_profile", default_value=EnvironmentVariable("UGV_SIM_PROFILE", default_value="gpu")),
            DeclareLaunchArgument(
                "use_sim_camera",
                default_value=EnvironmentVariable("UGV_SIM_CAMERA_ENABLED", default_value="false"),
            ),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("UGV_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            DeclareLaunchArgument("rviz_config", default_value=os.path.join(bringup_share, "config", "rviz", "mapping.rviz")),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument("publish_global_map_tf", default_value="true"),
            sim_launch,
            slam_launch,
            scan_rewriter,
        ]
    )
