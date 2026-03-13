import copy
import os
import shutil
import tempfile

import yaml


NAV2_PRESET_OVERLAYS = {
    "none": None,
    "hw": "nav2.hw.overlay.yaml",
    "sim": "nav2.sim.overlay.yaml",
}


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


def _load_yaml_mapping(path: str):
    with open(path, "r", encoding="utf-8") as input_stream:
        data = yaml.safe_load(input_stream)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at {path}, got {type(data).__name__}")
    return data


def _deep_merge_mappings(base, overlay):
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return copy.deepcopy(overlay)

    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _deep_merge_mappings(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _nav2_preset_overlay_path(config_dir: str, preset: str | None):
    resolved_preset = (preset or "hw").strip().lower()
    if resolved_preset not in NAV2_PRESET_OVERLAYS:
        raise RuntimeError(
            f"Unknown nav2 config_preset '{preset}'. Expected one of: {', '.join(sorted(NAV2_PRESET_OVERLAYS))}"
        )
    overlay_name = NAV2_PRESET_OVERLAYS[resolved_preset]
    if overlay_name is None:
        return None
    return os.path.join(config_dir, overlay_name)


def resolve_nav2_config_stack(
    *,
    config_dir: str,
    config_preset: str,
    params_file: str | None = None,
    config_overlay: str | None = None,
):
    base_path = params_file.strip() if params_file else ""
    if not base_path:
        base_path = os.path.join(config_dir, "nav2.common.yaml")
    if not os.path.isfile(base_path):
        raise RuntimeError(f"Nav2 params base file not found: {base_path}")

    overlay_paths = [base_path]
    preset_overlay = _nav2_preset_overlay_path(config_dir, config_preset)
    if preset_overlay:
        if not os.path.isfile(preset_overlay):
            raise RuntimeError(f"Nav2 preset overlay not found: {preset_overlay}")
        overlay_paths.append(preset_overlay)

    config_overlay_path = config_overlay.strip() if config_overlay else ""
    if config_overlay_path:
        if not os.path.isfile(config_overlay_path):
            raise RuntimeError(f"Nav2 config_overlay not found: {config_overlay_path}")
        overlay_paths.append(config_overlay_path)

    return overlay_paths


def create_merged_nav2_params(
    *,
    config_dir: str,
    config_preset: str,
    params_file: str | None = None,
    config_overlay: str | None = None,
):
    config_stack = resolve_nav2_config_stack(
        config_dir=config_dir,
        config_preset=config_preset,
        params_file=params_file,
        config_overlay=config_overlay,
    )

    merged = {}
    for path in config_stack:
        merged = _deep_merge_mappings(merged, _load_yaml_mapping(path))

    temp_dir = tempfile.mkdtemp(prefix="ugv_nav2_")
    merged_path = os.path.join(temp_dir, "nav2.params.yaml")
    with open(merged_path, "w", encoding="utf-8") as output_stream:
        yaml.safe_dump(merged, output_stream, sort_keys=False)

    return temp_dir, merged_path


def cleanup_temp_dir(path: str | None):
    if path and os.path.isdir(path):
        shutil.rmtree(path, ignore_errors=True)


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
