import os


def runtime_maps_dir() -> str:
    xdg_data_home = os.environ.get("XDG_DATA_HOME")
    if not xdg_data_home:
        xdg_data_home = os.path.join(os.path.expanduser("~"), ".local", "share")
    return os.path.join(os.path.abspath(os.path.expanduser(xdg_data_home)), "ugv_bringup", "maps")


def _resolve_latest_map_yaml_in_dir(maps_dir: str):
    numeric_maps = []
    newest_map = None
    newest_mtime = -1.0

    if not os.path.isdir(maps_dir):
        return None

    for filename in os.listdir(maps_dir):
        if not filename.endswith(".yaml"):
            continue
        path = os.path.join(maps_dir, filename)
        if not os.path.isfile(path):
            continue
        stem = os.path.splitext(filename)[0]
        if stem.isdigit():
            numeric_maps.append((int(stem), path))
        mtime = os.path.getmtime(path)
        if mtime > newest_mtime:
            newest_mtime = mtime
            newest_map = path

    if numeric_maps:
        numeric_maps.sort(key=lambda item: item[0])
        return numeric_maps[-1][1]
    return newest_map


def resolve_default_map_yaml(bringup_share: str, runtime_dir: str | None = None) -> str:
    resolved_runtime_dir = runtime_dir or runtime_maps_dir()
    for maps_dir in (resolved_runtime_dir, os.path.join(bringup_share, "maps")):
        resolved = _resolve_latest_map_yaml_in_dir(maps_dir)
        if resolved is not None:
            return resolved
    return os.path.join(bringup_share, "maps", "default.yaml")


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

