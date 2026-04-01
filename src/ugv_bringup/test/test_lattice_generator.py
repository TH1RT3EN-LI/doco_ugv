import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ugv_bringup.lattice_generator import (
    bootstrap_spec_from_lattice,
    generate_lattice_from_spec,
    load_lattice_spec,
    validate_lattice,
)
from ugv_bringup.omni_lattice_preset import build_rich_omni_spec


def _checked_in_spec_path() -> Path:
    return Path(__file__).resolve().parents[1] / "config" / "lattice_primitives" / "ugv_omni_2cm_spec.json"


def test_checked_in_spec_generates_self_consistent_lattice():
    spec = load_lattice_spec(_checked_in_spec_path())

    lattice = generate_lattice_from_spec(spec, date_generated="2026-04-01")

    assert lattice["lattice_metadata"]["number_of_trajectories"] == len(lattice["primitives"])
    assert validate_lattice(lattice, spec=spec) == []


def test_bootstrap_spec_round_trips_generated_lattice():
    spec = load_lattice_spec(_checked_in_spec_path())
    lattice = generate_lattice_from_spec(spec, date_generated="2026-04-01")

    bootstrapped_spec = bootstrap_spec_from_lattice(lattice)
    regenerated = generate_lattice_from_spec(bootstrapped_spec, date_generated="2026-04-01")

    assert regenerated == lattice


def test_validate_reports_mismatch_against_spec():
    spec = load_lattice_spec(_checked_in_spec_path())
    lattice = generate_lattice_from_spec(spec, date_generated="2026-04-01")
    lattice["primitives"][0]["trajectory_length"] += 0.01

    errors = validate_lattice(lattice, spec=spec)

    assert any("does not match the supplied source spec" in error for error in errors)


def test_segment_sequence_templates_generate_expected_order():
    spec = {
        "version": 1.0,
        "date_generated": "2026-04-01",
        "generation": {
            "linear_sampling_step": 0.1,
            "angular_sampling_step": math.pi / 8.0,
            "pose_rounding": 5,
            "yaw_rounding": 12,
        },
        "lattice_metadata": {
            "motion_model": "omni",
            "turning_radius": 0.4,
            "grid_resolution": 0.1,
            "stopping_threshold": 1,
            "num_of_headings": 4,
            "heading_angles": [0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0],
            "number_of_trajectories": 0,
        },
        "templates": [
            {
                "name": "rotate_left",
                "template_type": "segment_sequence",
                "start_heading_indices": [0],
                "left_turn": True,
                "segments": [{"type": "rotate", "delta_heading_steps": 1}],
            },
            {
                "name": "forward",
                "template_type": "segment_sequence",
                "start_heading_indices": [0],
                "left_turn": True,
                "segments": [{"type": "translate", "distance": 0.2}],
            },
        ],
    }

    lattice = generate_lattice_from_spec(spec, date_generated="2026-04-01")

    assert [primitive["end_angle_index"] for primitive in lattice["primitives"]] == [0, 1]
    assert lattice["primitives"][0]["poses"][-1] == [0.2, 0.0, 0.0]
    assert validate_lattice(lattice, spec=spec) == []


def test_rich_omni_preset_builds_a_valid_high_coverage_lattice():
    root = Path(__file__).resolve().parents[1]
    spec = build_rich_omni_spec(
        nav2_yaml_path=root / "config" / "nav2.yaml",
        lattice_config_path=root / "config" / "lattice_primitives" / "ugv_omni_2cm_config.json",
        base_lattice_path=root / "config" / "lattice_primitives" / "ugv_omni_2cm_lattice.json",
    )

    lattice = generate_lattice_from_spec(spec, date_generated="2026-04-01")
    template_names = {template["name"] for template in spec["templates"]}

    assert "left_axis_micro" in template_names
    assert "back_left_axis_micro" in template_names
    assert "back_right_oblique_short" in template_names
    assert len(spec["templates"]) > 49
    assert len(lattice["primitives"]) > 288
    assert validate_lattice(lattice, spec=spec) == []
