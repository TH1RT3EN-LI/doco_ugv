import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ugv_bringup.launch_helpers import create_global_to_ugv_map_tf_action


def generate_launch_description():
    robot_prefix = "ugv_"
    odom_frame = f"{robot_prefix}odom"
    base_footprint_frame = f"{robot_prefix}base_footprint"
    imu_frame = f"{robot_prefix}imu_link"

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_foxglove = LaunchConfiguration("use_foxglove")
    use_lidar = LaunchConfiguration("use_lidar")
    use_joy_teleop = LaunchConfiguration("use_joy_teleop")
    keyboard_enabled = LaunchConfiguration("keyboard_enabled")
    keyboard_backend = LaunchConfiguration("keyboard_backend")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    publish_joint_states = LaunchConfiguration("publish_joint_states")
    odom0 = LaunchConfiguration("odom0")
    imu0 = LaunchConfiguration("imu0")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")

    bringup_share = get_package_share_directory("ugv_bringup")
    description_share = get_package_share_directory("ugv_description")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    ekf_config_path = os.path.join(bringup_share, "config", "ekf.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_share, "config", "foxglove", "bridge.yaml")
    watchdog_share = get_package_share_directory("ugv_safety_watchdog")
    watchdog_config_path = os.path.join(watchdog_share, "config", "safety_watchdog.yaml")
    twist_mux_config_path = os.path.join(bringup_share, "config", "twist_mux.yaml")

    urdf_file = os.path.join(description_share, "urdf", "ugv.xacro")
    robot_description = ParameterValue(
        Command(["xacro", " ", urdf_file, " prefix:=", robot_prefix]),
        value_type=str,
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        remappings=[
            ("/joint_states", "/ugv/joint_states"),
            ("/robot_description", "/ugv/robot_description"),
        ],
        condition=IfCondition(publish_joint_states),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        remappings=[
            ("/joint_states", "/ugv/joint_states"),
            ("/robot_description", "/ugv/robot_description"),
        ],
        condition=IfCondition(publish_robot_model),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config_path,
            {"use_sim_time": use_sim_time_param},
            {"odom0": odom0},
            {"imu0": imu0},
        ],
        remappings=[("/odometry/filtered", "/ugv/odometry/filtered")],
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_foxglove),
    )

    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "joy_teleop.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
        condition=IfCondition(use_joy_teleop),
    )
    keyboard_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "keyboard_teleop.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "keyboard_backend": keyboard_backend,
        }.items(),
        condition=IfCondition(keyboard_enabled),
    )
    lslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("lslidar_driver"), "launch", "lslidar_launch.py")
        ),
        condition=IfCondition(use_lidar),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config_path, {"use_sim_time": use_sim_time_param}],
        remappings=[("cmd_vel_out", "/ugv/cmd_vel_safe")],
    )

    base_driver_node = Node(
        package="ugv_base_driver",
        executable="ugv_base_driver_node",
        name="base_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "odom_frame_id": odom_frame,
                "robot_frame_id": base_footprint_frame,
                "imu_frame_id": imu_frame,
            }
        ],
        remappings=[("/ugv/cmd_vel", "/ugv/cmd_vel_safe")],
    )

    safety_watchdog_node = Node(
        package="ugv_safety_watchdog",
        executable="safety_watchdog_node",
        name="safety_watchdog",
        output="screen",
        parameters=[watchdog_config_path, {"use_sim_time": use_sim_time_param}],
    )

    global_to_ugv_map_tf_action = create_global_to_ugv_map_tf_action(
        global_frame=global_frame,
        ugv_map_frame=ugv_map_frame,
        global_to_ugv_map=global_to_ugv_map,
        publish_global_map_tf=publish_global_map_tf,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="false")),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            DeclareLaunchArgument("use_foxglove", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("use_joy_teleop", default_value="true"),
            DeclareLaunchArgument("keyboard_enabled", default_value="false"),
            DeclareLaunchArgument("keyboard_backend", default_value="gui"),
            DeclareLaunchArgument("odom0", default_value="/ugv/odom"),
            DeclareLaunchArgument("imu0", default_value="/ugv/imu"),
            DeclareLaunchArgument("publish_robot_model", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            DeclareLaunchArgument("global_frame", default_value="global"),
            DeclareLaunchArgument("ugv_map_frame", default_value="ugv_map"),
            DeclareLaunchArgument(
                "global_to_ugv_map",
                default_value="0,0,0,0,0,0",
                description="Static transform x,y,z,roll,pitch,yaw from global_frame to ugv_map_frame",
            ),
            DeclareLaunchArgument(
                "publish_global_map_tf",
                default_value="true",
                description="Whether to publish static transform from global_frame to ugv_map_frame",
            ),
            global_to_ugv_map_tf_action,
            twist_mux_node,
            base_driver_node,
            safety_watchdog_node,
            joint_state_publisher,
            robot_state_publisher,
            ekf_node,
            foxglove_bridge,
            joy_teleop_launch,
            keyboard_teleop_launch,
            lslidar_launch,
        ]
    )
