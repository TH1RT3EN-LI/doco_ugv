import os

from ugv_bringup.launch_helpers import parse_six_dof, resolve_default_map_yaml


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
