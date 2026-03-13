import os

import yaml

from ugv_bringup.launch_helpers import (
    create_merged_nav2_params,
    parse_six_dof,
    resolve_default_map_yaml,
    resolve_nav2_config_stack,
)


def test_parse_six_dof_accepts_comma_and_space():
    assert parse_six_dof("1, 2 3,4 5 6", "arg") == ["1", "2", "3", "4", "5", "6"]


def test_resolve_default_map_yaml_prefers_runtime_dir(tmp_path):
    bringup_share = tmp_path / "bringup"
    package_maps = bringup_share / "maps"
    runtime_maps = tmp_path / "runtime"
    package_maps.mkdir(parents=True)
    runtime_maps.mkdir(parents=True)

    (package_maps / "default.yaml").write_text("image: default.pgm\n", encoding="utf-8")
    (runtime_maps / "1.yaml").write_text("image: 1.pgm\n", encoding="utf-8")

    resolved = resolve_default_map_yaml(str(bringup_share), runtime_dir=str(runtime_maps))
    assert resolved == os.path.join(runtime_maps, "1.yaml")


def test_resolve_default_map_yaml_falls_back_to_package_maps(tmp_path):
    bringup_share = tmp_path / "bringup"
    package_maps = bringup_share / "maps"
    package_maps.mkdir(parents=True)

    (package_maps / "default.yaml").write_text("image: default.pgm\n", encoding="utf-8")
    resolved = resolve_default_map_yaml(str(bringup_share), runtime_dir=str(tmp_path / "missing"))
    assert resolved == os.path.join(package_maps, "default.yaml")


def test_resolve_nav2_config_stack_uses_common_and_preset_overlay(tmp_path):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    (config_dir / "nav2.common.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /base\n",
        encoding="utf-8",
    )
    (config_dir / "nav2.hw.overlay.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /hw\n",
        encoding="utf-8",
    )
    (config_dir / "nav2.sim.overlay.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /sim\n",
        encoding="utf-8",
    )

    stack = resolve_nav2_config_stack(config_dir=str(config_dir), config_preset="sim")
    assert stack == [
        os.path.join(config_dir, "nav2.common.yaml"),
        os.path.join(config_dir, "nav2.sim.overlay.yaml"),
    ]


def test_create_merged_nav2_params_applies_external_overlay(tmp_path):
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    (config_dir / "nav2.common.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /base\n    use_sim_time: false\n",
        encoding="utf-8",
    )
    (config_dir / "nav2.hw.overlay.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /hw\n",
        encoding="utf-8",
    )
    (config_dir / "nav2.sim.overlay.yaml").write_text(
        "controller_server:\n  ros__parameters:\n    odom_topic: /sim\n    use_sim_time: true\n",
        encoding="utf-8",
    )
    external_overlay = config_dir / "custom.overlay.yaml"
    external_overlay.write_text(
        "controller_server:\n  ros__parameters:\n    max_vel_x: 0.42\n",
        encoding="utf-8",
    )

    _, merged_path = create_merged_nav2_params(
        config_dir=str(config_dir),
        config_preset="sim",
        config_overlay=str(external_overlay),
    )
    with open(merged_path, "r", encoding="utf-8") as input_stream:
        merged = yaml.safe_load(input_stream)

    assert merged["controller_server"]["ros__parameters"]["odom_topic"] == "/sim"
    assert merged["controller_server"]["ros__parameters"]["use_sim_time"] is True
    assert merged["controller_server"]["ros__parameters"]["max_vel_x"] == 0.42
