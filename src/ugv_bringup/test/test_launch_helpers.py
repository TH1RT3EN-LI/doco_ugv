import os
from pathlib import Path

import yaml

from ugv_bringup.launch_helpers import (
    default_nav2_params_path,
    parse_six_dof,
    resolve_default_map_yaml,
    runtime_maps_dir,
)
from ugv_bringup.map_paths import get_workspace_maps_dir, resolve_map_yaml


def test_parse_six_dof_accepts_comma_and_space():
    assert parse_six_dof("1, 2 3,4 5 6", "arg") == ["1", "2", "3", "4", "5", "6"]


def test_get_workspace_maps_dir_uses_workspace_root_from_install_prefix(tmp_path):
    package_prefix = tmp_path / "workspace" / "install" / "ugv_bringup"
    package_prefix.mkdir(parents=True)

    resolved = get_workspace_maps_dir(package_prefix=package_prefix)
    assert resolved == tmp_path / "workspace" / "maps"


def test_runtime_maps_dir_uses_workspace_maps(monkeypatch, tmp_path):
    monkeypatch.setattr(
        "ugv_bringup.launch_helpers.get_workspace_maps_dir",
        lambda: tmp_path / "workspace" / "maps",
    )

    assert runtime_maps_dir() == str(tmp_path / "workspace" / "maps")


def test_resolve_default_map_yaml_prefers_runtime_dir(tmp_path):
    bringup_share = tmp_path / "bringup"
    runtime_maps = tmp_path / "runtime"
    runtime_maps.mkdir(parents=True)

    (runtime_maps / "1.yaml").write_text("image: 1.pgm\n", encoding="utf-8")

    resolved = resolve_default_map_yaml(str(bringup_share), runtime_dir=str(runtime_maps))
    assert resolved == os.path.join(runtime_maps, "1.yaml")


def test_resolve_default_map_yaml_defaults_to_workspace_default_yaml(tmp_path):
    bringup_share = tmp_path / "bringup"
    runtime_maps = tmp_path / "runtime"

    resolved = resolve_default_map_yaml(str(bringup_share), runtime_dir=str(runtime_maps))
    assert resolved == os.path.join(runtime_maps, "default.yaml")


def test_resolve_map_yaml_uses_workspace_root_for_relative_paths(tmp_path):
    workspace_root = tmp_path / "workspace"
    maps_dir = workspace_root / "maps"
    maps_dir.mkdir(parents=True)
    target = maps_dir / "7.yaml"
    target.write_text("image: 7.pgm\n", encoding="utf-8")

    resolved = resolve_map_yaml("maps/7.yaml", package_prefix=workspace_root / "install" / "ugv_bringup")
    assert resolved == str(target)


def test_default_nav2_params_path_points_to_single_nav2_yaml(tmp_path):
    bringup_share = tmp_path / "bringup"
    assert default_nav2_params_path(str(bringup_share)) == os.path.join(bringup_share, "config", "nav2.yaml")


def test_nav2_yaml_uses_single_file_hardware_defaults():
    config_path = Path(__file__).resolve().parents[1] / "config" / "nav2.yaml"
    with open(config_path, "r", encoding="utf-8") as input_stream:
        nav2 = yaml.safe_load(input_stream)

    controller = nav2["controller_server"]["ros__parameters"]
    local_costmap = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    planner = nav2["planner_server"]["ros__parameters"]["GridBased"]
    velocity_smoother = nav2["velocity_smoother"]["ros__parameters"]

    assert controller["odom_topic"] == "/ugv/odometry/filtered"
    assert controller["FollowPath"]["vx_max"] == 0.5
    assert controller["FollowPath"]["vy_max"] == 0.15
    assert controller["FollowPath"]["wz_max"] == 1.0
    assert local_costmap["plugins"] == ["static_layer", "obstacle_layer", "inflation_layer"]
    assert local_costmap["footprint"] == "[[0.24, 0.24], [0.24, -0.24], [-0.24, -0.24], [-0.24, 0.24]]"
    assert planner["plugin"] == "nav2_smac_planner/SmacPlannerLattice"
    assert planner["lattice_filepath"] == "__LATTICE_FILEPATH__"
    assert velocity_smoother["odom_topic"] == "/ugv/odometry/filtered"
    assert velocity_smoother["max_velocity"] == [0.5, 0.15, 1.0]
