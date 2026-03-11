import os
import subprocess
import tempfile
import xml.etree.ElementTree as element_tree
from functools import partial

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from sim_worlds.launch_common import (
    create_launch_summary_action,
    create_rviz_conditions,
    create_static_transform_node,
    create_true_only_notice_action,
    is_true,
    normalize_clock_mode,
    parse_six_dof,
    resolve_world_launch_configurations,
)

SIM_WORLDS_SHARE = get_package_share_directory("sim_worlds")


def generate_launch_description():
    robot_prefix = "ugv_"
    odom_frame = "ugv_odom"
    base_footprint_frame = "ugv_base_footprint"
    imu_frame = "ugv_imu_link"
    camera_depth_optical_frame = "ugv_camera_depth_optical_frame"
    camera_color_optical_frame = "ugv_camera_color_optical_frame"
    camera_ir_optical_frame = "ugv_camera_ir_optical_frame"

    pkg_prefix = get_package_prefix("ugv_bringup")
    bringup_share = get_package_share_directory("ugv_bringup")
    sim_worlds_share = get_package_share_directory("sim_worlds")
    workspace_root = os.path.dirname(os.path.dirname(pkg_prefix))
    bringup_src = os.path.join(workspace_root, "src", "ugv_bringup")
    description_src = os.path.join(workspace_root, "src", "ugv_description")

    ekf_config_path = os.path.join(bringup_src, "config", "ekf.yaml")
    bridge_cfg = os.path.join(bringup_src, "config", "ros_gz_bridge.yaml")
    foxglove_bridge_config_path = os.path.join(bringup_src, "config", "foxglove", "bridge.yaml")
    default_rviz_config = os.path.join(bringup_src, "config", "rviz", "sim.rviz")
    watchdog_share = get_package_share_directory("ugv_safety_watchdog")
    watchdog_config_path = os.path.join(watchdog_share, "config", "safety_watchdog.yaml")
    default_gz_partition = f"ugv_{os.getpid()}"
    urdf_file = os.path.join(description_src, "urdf", "ugv_sim.xacro")

    world = LaunchConfiguration("world")
    resolved_world_id = LaunchConfiguration("resolved_world_id")
    resolved_world_sdf_path = LaunchConfiguration("resolved_world_sdf_path")
    resolved_gz_world_name = LaunchConfiguration("resolved_gz_world_name")
    headless = LaunchConfiguration("headless")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)
    sim_profile = LaunchConfiguration("sim_profile")
    render_engine = LaunchConfiguration("render_engine")
    software_gl = LaunchConfiguration("software_gl")
    gz_partition = LaunchConfiguration("gz_partition")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_software_gl = LaunchConfiguration("rviz_software_gl")
    rviz_config = LaunchConfiguration("rviz_config")
    launch_ros_clock = LaunchConfiguration("launch_ros_clock")
    clock_mode = LaunchConfiguration("clock_mode")
    effective_clock_mode = LaunchConfiguration("effective_clock_mode")

    use_sim_tf = LaunchConfiguration("use_sim_tf")
    global_frame = LaunchConfiguration("global_frame")
    ugv_map_frame = LaunchConfiguration("ugv_map_frame")
    global_to_ugv_map = LaunchConfiguration("global_to_ugv_map")
    publish_global_map_tf = LaunchConfiguration("publish_global_map_tf")
    use_sim_camera = LaunchConfiguration("use_sim_camera")
    controller_port = LaunchConfiguration("controller_port")
    use_teleop = LaunchConfiguration("use_teleop")
    keyboard_enabled = LaunchConfiguration("keyboard_enabled")
    keyboard_backend = LaunchConfiguration("keyboard_backend")
    use_foxglove = LaunchConfiguration("use_foxglove")
    base_driver_start_delay = LaunchConfiguration("base_driver_start_delay")
    publish_map_tf = LaunchConfiguration("publish_map_tf")
    robot_prefix_cfg = LaunchConfiguration("robot_prefix")
    ugv_model_sdf_path = LaunchConfiguration("ugv_model_sdf_path")
    ugv_spawn_entity_name = LaunchConfiguration("ugv_spawn_entity_name")
    enable_dynamic_global_alignment = LaunchConfiguration("enable_dynamic_global_alignment")
    base_driver_wait_logged = LaunchConfiguration("base_driver_wait_logged")

    robot_description = ParameterValue(
        Command(["xacro", " ", urdf_file, " prefix:=", robot_prefix_cfg]),
        value_type=str,
    )

    resolve_world_action = OpaqueFunction(
        function=partial(
            resolve_world_launch_configurations,
            package_share=SIM_WORLDS_SHARE,
        )
    )

    def resolve_effective_clock_mode(context):
        requested_clock_mode = clock_mode.perform(context)
        if requested_clock_mode.strip():
            return [
                SetLaunchConfiguration(
                    "effective_clock_mode",
                    normalize_clock_mode(requested_clock_mode, default_value="internal"),
                )
            ]

        legacy_launch_ros_clock = launch_ros_clock.perform(context)
        actions = [
            SetLaunchConfiguration(
                "effective_clock_mode",
                "internal" if is_true(legacy_launch_ros_clock) else "external",
            )
        ]
        if not is_true(legacy_launch_ros_clock):
            actions.append(
                LogInfo(
                    msg="[ugv_sim] 'launch_ros_clock' is deprecated; use 'clock_mode:=external' instead."
                )
            )
        return actions

    resolve_clock_mode_action = OpaqueFunction(function=resolve_effective_clock_mode)

    def prepare_ugv_model_sdf(context):
        resolved_world_id_value = resolved_world_id.perform(context)
        use_sim_camera_value = use_sim_camera.perform(context)
        robot_prefix_value = robot_prefix_cfg.perform(context)
        env = os.environ.copy()
        env["UGV_SIM_CAMERA_ENABLED"] = use_sim_camera_value
        robot_urdf = subprocess.check_output(
            ["xacro", urdf_file, f"prefix:={robot_prefix_value}"],
            text=True,
            env=env,
        )
        temp_dir = tempfile.mkdtemp(prefix="ugv_sim_model_")
        urdf_path = os.path.join(temp_dir, "ugv_sim.urdf")
        with open(urdf_path, "w", encoding="utf-8") as urdf_output:
            urdf_output.write(robot_urdf)

        model_sdf = subprocess.check_output(["gz", "sdf", "-p", urdf_path], text=True, env=env)
        model_sdf_path = os.path.join(temp_dir, f"{resolved_world_id_value}_ugv.sdf")
        with open(model_sdf_path, "w", encoding="utf-8") as model_output:
            model_output.write(model_sdf)

        model_root = element_tree.fromstring(model_sdf)
        model_element = model_root if model_root.tag == "model" else model_root.find("model")
        entity_name = "duojin01"
        if model_element is not None and model_element.attrib.get("name"):
            entity_name = model_element.attrib["name"]

        return [
            SetLaunchConfiguration("ugv_model_sdf_path", model_sdf_path),
            SetLaunchConfiguration("ugv_spawn_entity_name", entity_name),
        ]

    prepare_ugv_model_action = OpaqueFunction(function=prepare_ugv_model_sdf)

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "world": world,
            "resolved_world_id": resolved_world_id,
            "resolved_world_sdf_path": resolved_world_sdf_path,
            "resolved_gz_world_name": resolved_gz_world_name,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
    )

    spawn_ugv_model = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ugv_model",
        output="screen",
        parameters=[
            {
                "world": resolved_gz_world_name,
                "file": ugv_model_sdf_path,
                "name": ugv_spawn_entity_name,
                "allow_renaming": False,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "R": 0.0,
                "P": 0.0,
                "Y": 0.0,
            }
        ],
    )

    internal_clock_condition = IfCondition(
        PythonExpression(['"', effective_clock_mode, '" == "internal"'])
    )
    sim_clock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_worlds_share, "launch", "sim_clock.launch.py")),
        launch_arguments={
            "gz_partition": gz_partition,
        }.items(),
        condition=internal_clock_condition,
    )

    def create_global_to_ugv_map_tf(context):
        if not is_true(publish_global_map_tf.perform(context)):
            return []

        return [
            create_static_transform_node(
                node_name="global_to_ugv_map_tf",
                transform_values=parse_six_dof(global_to_ugv_map.perform(context), "global_to_ugv_map"),
                parent_frame=global_frame.perform(context),
                child_frame=ugv_map_frame.perform(context),
            )
        ]

    global_to_ugv_map_tf_action = OpaqueFunction(function=create_global_to_ugv_map_tf)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time_param}],
        remappings=[
            ("/joint_states", "/ugv/joint_states"),
            ("/robot_description", "/ugv/robot_description"),
        ],
    )

    joint_state_stamp_fix = Node(
        package="ugv_sim_tools",
        executable="joint_state_stamp_fix_node",
        name="joint_state_stamp_fix",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_topic": "/ugv/joint_states_raw",
                "output_topic": "/ugv/joint_states",
            }
        ],
    )

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        namespace="sim_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time_param, "lazy": True, "config_file": bridge_cfg}],
    )

    joy_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "joy_teleop.launch.py")),
        condition=IfCondition(use_teleop),
    )
    keyboard_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "keyboard_teleop.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "keyboard_backend": keyboard_backend,
        }.items(),
        condition=IfCondition(keyboard_enabled),
    )

    controller_emulator = Node(
        package="ugv_controller_emulator",
        executable="ugv_controller_emulator_node",
        name="ugv_controller_emulator",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "pty_link_path": controller_port,
                "odom_topic": "/ugv/sim/odom",
                "cmd_vel_out_topic": "/ugv/cmd_vel_sim",
            }
        ],
    )

    sim_odom_to_tf = Node(
        package="ugv_sim_tools",
        executable="odom_to_tf_node",
        name="sim_odom_to_tf",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "input_odom_topic": "/ugv/sim/odom",
                "odom_frame_id": odom_frame,
                "child_frame_id": base_footprint_frame,
            }
        ],
        condition=IfCondition(use_sim_tf),
    )

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_ugv_odom_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", ugv_map_frame, odom_frame],
        condition=IfCondition(publish_map_tf),
    )
    orbbec_topic_compat = Node(
        package="ugv_sim_tools",
        executable="orbbec_topic_compat_node",
        name="orbbec_topic_compat",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "color_frame_id": camera_color_optical_frame,
                "depth_frame_id": camera_depth_optical_frame,
                "ir_frame_id": camera_ir_optical_frame,
                "cloud_frame_id": camera_depth_optical_frame,
            }
        ],
        condition=IfCondition(use_sim_camera),
    )

    twist_mux_config_path = os.path.join(bringup_src, "config", "twist_mux.yaml")

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_config_path, {"use_sim_time": False}],
        remappings=[("cmd_vel_out", "/ugv/cmd_vel_safe")],
    )

    base_driver = Node(
        package="ugv_base_driver",
        executable="ugv_base_driver_node",
        name="ugv_base_driver",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time_param,
                "usart_port_name": controller_port,
                "odom_frame_id": odom_frame,
                "robot_frame_id": base_footprint_frame,
                "imu_frame_id": imu_frame,
            }
        ],
        remappings=[("/ugv/cmd_vel", "/ugv/cmd_vel_safe")],
    )

    def launch_base_driver_when_ready(context):
        controller_port_path = controller_port.perform(context)
        if os.path.exists(controller_port_path):
            return [
                LogInfo(msg=f"[ugv_sim] controller port ready at {controller_port_path}; starting ugv_base_driver."),
                base_driver,
            ]

        actions = []
        if not is_true(base_driver_wait_logged.perform(context)):
            actions.extend([
                SetLaunchConfiguration("base_driver_wait_logged", "true"),
                LogInfo(msg=f"[ugv_sim] waiting for controller port: {controller_port_path}"),
            ])
        actions.append(
            TimerAction(period=0.5, actions=[OpaqueFunction(function=launch_base_driver_when_ready)])
        )
        return actions

    base_driver_delayed = TimerAction(
        period=base_driver_start_delay,
        actions=[OpaqueFunction(function=launch_base_driver_when_ready)],
    )

    safety_watchdog = Node(
        package="ugv_safety_watchdog",
        executable="safety_watchdog_node",
        name="safety_watchdog",
        output="screen",
        parameters=[
            watchdog_config_path,
            {"use_sim_time": use_sim_time_param},
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_path, {"use_sim_time": use_sim_time_param}],
        condition=UnlessCondition(use_sim_tf),
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge_node",
        output="screen",
        parameters=[foxglove_bridge_config_path, {"use_sim_time": use_sim_time_param}],
        condition=IfCondition(use_foxglove),
    )
    rviz_soft_condition, rviz_hw_condition = create_rviz_conditions(use_rviz, rviz_software_gl)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
        additional_env={
            "LIBGL_DRI3_DISABLE": "1",
            "LIBGL_ALWAYS_SOFTWARE": "1",
            "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
            "QT_XCB_GL_INTEGRATION": "none",
            "QT_OPENGL": "software",
        },
        condition=rviz_soft_condition,
    )
    rviz_node_hw = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
        condition=rviz_hw_condition,
    )

    summary_action = create_launch_summary_action(
        "ugv_sim",
        items=[
            ("clock_mode", effective_clock_mode),
            ("gz_partition", gz_partition),
            ("world", resolved_world_id),
            ("gz_world", resolved_gz_world_name),
        ],
    )
    no_op_alignment_notice = create_true_only_notice_action(
        "ugv_sim",
        argument_name="enable_dynamic_global_alignment",
        argument_value=enable_dynamic_global_alignment,
        message="The dynamic global alignment hook has no runtime consumer in the current stack.",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=EnvironmentVariable("UGV_WORLD", default_value="test"),
                description="Registered sim_worlds world id",
            ),
            DeclareLaunchArgument("resolved_world_id", default_value=""),
            DeclareLaunchArgument("resolved_world_sdf_path", default_value=""),
            DeclareLaunchArgument("resolved_gz_world_name", default_value=""),
            DeclareLaunchArgument("resolved_ground_height", default_value=""),
            DeclareLaunchArgument("robot_prefix", default_value=robot_prefix),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("launch_ros_clock", default_value="true"),
            DeclareLaunchArgument("clock_mode", default_value=""),
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
            DeclareLaunchArgument("enable_dynamic_global_alignment", default_value="false"),
            DeclareLaunchArgument(
                "gz_partition",
                default_value=EnvironmentVariable("UGV_GZ_PARTITION", default_value=default_gz_partition),
            ),
            DeclareLaunchArgument("use_sim_time", default_value=EnvironmentVariable("USE_SIM_TIME", default_value="true")),
            DeclareLaunchArgument(
                "use_sim_camera",
                default_value=EnvironmentVariable("UGV_SIM_CAMERA_ENABLED", default_value="false"),
            ),
            DeclareLaunchArgument(
                "sim_profile",
                default_value=EnvironmentVariable("UGV_SIM_PROFILE", default_value="gpu"),
                description="simulation profile: gpu or cpu",
            ),
            DeclareLaunchArgument(
                "render_engine",
                default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2"),
            ),
            DeclareLaunchArgument(
                "software_gl",
                default_value=PythonExpression(['"true" if "', sim_profile, '" == "cpu" else "false"']),
            ),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "rviz_software_gl",
                default_value=EnvironmentVariable("UGV_RVIZ_SOFTWARE_GL", default_value="true"),
            ),
            DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
            DeclareLaunchArgument("use_sim_tf", default_value="true"),
            DeclareLaunchArgument("publish_map_tf", default_value="true"),
            DeclareLaunchArgument(
                "controller_port",
                default_value=EnvironmentVariable("UGV_CONTROLLER_PORT", default_value="/tmp/ugv_controller"),
            ),
            DeclareLaunchArgument("use_teleop", default_value="true"),
            DeclareLaunchArgument("keyboard_enabled", default_value="false"),
            DeclareLaunchArgument("keyboard_backend", default_value="tty"),
            DeclareLaunchArgument("use_foxglove", default_value="false"),
            DeclareLaunchArgument("base_driver_start_delay", default_value="6.0"),
            DeclareLaunchArgument("effective_clock_mode", default_value=""),
            DeclareLaunchArgument("ugv_model_sdf_path", default_value=""),
            DeclareLaunchArgument("ugv_spawn_entity_name", default_value="duojin01"),
            DeclareLaunchArgument("base_driver_wait_logged", default_value="false"),
            SetEnvironmentVariable("USE_SIM_TIME", use_sim_time),
            SetEnvironmentVariable("UGV_SIM_CAMERA_ENABLED", use_sim_camera),
            SetEnvironmentVariable("UGV_SIM_PROFILE", sim_profile),
            SetEnvironmentVariable("UGV_GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("GZ_PARTITION", gz_partition),
            SetEnvironmentVariable("UGV_RVIZ_SOFTWARE_GL", rviz_software_gl),
            SetEnvironmentVariable("LIBGL_DRI3_DISABLE", "1"),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("MESA_LOADER_DRIVER_OVERRIDE", "llvmpipe", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_XCB_GL_INTEGRATION", "none", condition=IfCondition(software_gl)),
            SetEnvironmentVariable("QT_OPENGL", "software", condition=IfCondition(software_gl)),
            resolve_world_action,
            resolve_clock_mode_action,
            no_op_alignment_notice,
            summary_action,
            prepare_ugv_model_action,
            gz_sim_launch,
            sim_clock_launch,
            spawn_ugv_model,
            global_to_ugv_map_tf_action,
            robot_state_publisher,
            joint_state_stamp_fix,
            ros_gz_bridge,
            orbbec_topic_compat,
            controller_emulator,
            sim_odom_to_tf,
            map_to_odom_tf,
            twist_mux_node,
            base_driver_delayed,
            safety_watchdog,
            ekf_node,
            joy_teleop_launch,
            keyboard_teleop_launch,
            foxglove_bridge,
            rviz_node,
            rviz_node_hw,
        ]
    )
