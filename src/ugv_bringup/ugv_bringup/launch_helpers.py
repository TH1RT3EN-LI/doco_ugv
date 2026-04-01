import os

from ugv_bringup.map_paths import get_workspace_maps_dir, pick_latest_map_yaml


def runtime_maps_dir() -> str:
    return str(get_workspace_maps_dir())


def resolve_default_map_yaml(bringup_share: str, runtime_dir: str | None = None) -> str:
    resolved_runtime_dir = runtime_dir or runtime_maps_dir()
    try:
        return pick_latest_map_yaml(maps_dir=resolved_runtime_dir)
    except FileNotFoundError:
        return os.path.join(resolved_runtime_dir, "default.yaml")


def default_nav2_params_path(bringup_share: str) -> str:
    return os.path.join(bringup_share, "config", "nav2.yaml")


def parse_six_dof(raw_value: str, arg_name: str):
    tokens = [token.strip() for token in raw_value.replace(",", " ").split() if token.strip()]
    if len(tokens) != 6:
        raise RuntimeError(
            f"Launch argument '{arg_name}' must contain 6 numeric values (x y z roll pitch yaw), got: '{raw_value}'"
        )
    return tokens


def create_global_to_ugv_map_tf_action(
    *,
    global_frame,
    ugv_map_frame,
    global_to_ugv_map,
    publish_global_map_tf,
    node_name: str = "global_to_ugv_map_tf",
):
    from launch.actions import OpaqueFunction
    from launch_ros.actions import Node

    def _create(context):
        if publish_global_map_tf.perform(context).strip().lower() != "true":
            return []
        return [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=node_name,
                output="screen",
                arguments=[
                    *parse_six_dof(global_to_ugv_map.perform(context), "global_to_ugv_map"),
                    global_frame.perform(context),
                    ugv_map_frame.perform(context),
                ],
            )
        ]

    return OpaqueFunction(function=_create)


def create_controller_device_ready_gate(*, controller_device_path, actions, label: str):
    from launch.actions import LogInfo, OpaqueFunction, TimerAction

    state = {"wait_logged": False}

    def _gate(context):
        path_value = controller_device_path.perform(context)
        if os.path.exists(path_value):
            return [LogInfo(msg=f"[{label}] controller device ready at {path_value}; starting dependent actions."), *actions]

        result = []
        if not state["wait_logged"]:
            state["wait_logged"] = True
            result.append(LogInfo(msg=f"[{label}] waiting for controller device: {path_value}"))
        result.append(TimerAction(period=0.5, actions=[OpaqueFunction(function=_gate)]))
        return result

    return OpaqueFunction(function=_gate)
