from __future__ import annotations

import argparse
import ast
import datetime as dt
import math
from fractions import Fraction
from pathlib import Path
from typing import Any

import yaml

from ugv_bringup.lattice_generator import (
    bootstrap_spec_from_lattice,
    generate_lattice_file,
    load_lattice_spec,
    validate_lattice_file,
    _load_json,
    _write_json,
)


def _round_to_grid(value: float, grid_resolution: float) -> float:
    steps = max(1, round(value / grid_resolution))
    return steps * grid_resolution


def _load_half_extents_from_nav2(nav2_yaml_path: str | Path) -> tuple[float, float]:
    with open(nav2_yaml_path, "r", encoding="utf-8") as input_stream:
        nav2 = yaml.safe_load(input_stream)

    footprint_literal = nav2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
    footprint = ast.literal_eval(footprint_literal)
    xs = [float(point[0]) for point in footprint]
    ys = [float(point[1]) for point in footprint]
    return max(abs(x) for x in xs), max(abs(y) for y in ys)


def _load_goal_tolerance_from_nav2(nav2_yaml_path: str | Path) -> float:
    with open(nav2_yaml_path, "r", encoding="utf-8") as input_stream:
        nav2 = yaml.safe_load(input_stream)

    return float(nav2["controller_server"]["ros__parameters"]["general_goal_checker"]["xy_goal_tolerance"])


def _normalize_heading_basis(angle: float) -> tuple[int, int]:
    cosine = abs(math.cos(angle))
    sine = abs(math.sin(angle))
    reduced_angle = math.atan2(sine, cosine)
    best_pair: tuple[int, int] | None = None
    best_error = float("inf")

    for x in range(0, 4):
        for y in range(0, 4):
            if x == 0 and y == 0:
                continue
            if math.gcd(x, y) != 1:
                continue
            candidate_angle = math.atan2(y, x)
            error = abs(candidate_angle - reduced_angle)
            if error < best_error:
                best_error = error
                best_pair = (x, y)

    if best_pair is None:
        raise RuntimeError(f"Unable to resolve heading basis for angle {angle}.")

    return best_pair


def _heading_norm_family(heading_angle: float) -> tuple[str, float]:
    basis_x, basis_y = _normalize_heading_basis(heading_angle)
    norm = math.hypot(basis_x, basis_y)
    if abs(norm - 1.0) < 1e-6:
        return "axis", norm
    if abs(norm - math.sqrt(2.0)) < 1e-6:
        return "diag", norm
    if abs(norm - math.sqrt(5.0)) < 1e-6:
        return "oblique", norm
    return f"norm_{norm:.3f}".replace(".", "_"), norm


def _family_start_indices(heading_angles: list[float]) -> dict[str, list[int]]:
    families: dict[str, list[int]] = {}
    for index, angle in enumerate(heading_angles):
        family_name, _ = _heading_norm_family(angle)
        families.setdefault(family_name, []).append(index)
    return families


def _family_norms(heading_angles: list[float]) -> dict[str, float]:
    norms: dict[str, float] = {}
    for angle in heading_angles:
        family_name, norm = _heading_norm_family(angle)
        norms[family_name] = norm
    return norms


def _target_distance_labels(half_extent: float, goal_tolerance: float, grid_resolution: float) -> dict[str, float]:
    micro = _round_to_grid(min(0.06, max(0.04, goal_tolerance * 1.6)), grid_resolution)
    short = _round_to_grid(min(0.10, max(micro + grid_resolution, half_extent * 0.35)), grid_resolution)
    medium = _round_to_grid(min(0.18, max(0.14, half_extent * 0.67)), grid_resolution)
    long = _round_to_grid(min(0.28, max(0.22, half_extent)), grid_resolution)
    return {"micro": micro, "short": short, "medium": medium, "long": long}


def _steps_for_target(target_distance: float, family_norm: float, grid_resolution: float) -> int:
    return max(1, round(target_distance / (family_norm * grid_resolution)))


def _translate_template(
    *,
    name: str,
    description: str,
    start_heading_indices: list[int],
    direction: float,
    distance: float,
) -> dict[str, Any]:
    return {
        "name": name,
        "description": description,
        "template_type": "segment_sequence",
        "start_heading_indices": start_heading_indices,
        "left_turn": True,
        "segments": [{"type": "translate", "direction": direction, "distance": distance}],
    }


def _translation_distance(
    *,
    move_name: str,
    target_distance: float,
    family_norm: float,
    grid_resolution: float,
) -> float:
    local_axis_distance = _steps_for_target(target_distance, family_norm, grid_resolution) * family_norm * grid_resolution
    if move_name.startswith("front_") or move_name.startswith("back_"):
        return local_axis_distance * math.sqrt(2.0)
    return local_axis_distance


def _rotate_template(name: str, delta_heading_steps: int) -> dict[str, Any]:
    return {
        "name": name,
        "description": f"Rotate in place by {delta_heading_steps} heading steps.",
        "template_type": "segment_sequence",
        "left_turn": delta_heading_steps > 0,
        "segments": [{"type": "rotate", "delta_heading_steps": delta_heading_steps}],
    }


def build_rich_omni_spec(
    *,
    nav2_yaml_path: str | Path,
    lattice_config_path: str | Path,
    base_lattice_path: str | Path,
) -> dict[str, Any]:
    lattice_config = _load_json(lattice_config_path)
    base_lattice = _load_json(base_lattice_path)
    bootstrapped = bootstrap_spec_from_lattice(base_lattice)

    metadata = {
        "motion_model": lattice_config["motion_model"],
        "turning_radius": lattice_config["turning_radius"],
        "grid_resolution": lattice_config["grid_resolution"],
        "stopping_threshold": lattice_config["stopping_threshold"],
        "num_of_headings": lattice_config["num_of_headings"],
        "heading_angles": bootstrapped["lattice_metadata"]["heading_angles"],
        "number_of_trajectories": 0,
    }

    heading_angles = metadata["heading_angles"]
    grid_resolution = metadata["grid_resolution"]
    half_x, half_y = _load_half_extents_from_nav2(nav2_yaml_path)
    goal_tolerance = _load_goal_tolerance_from_nav2(nav2_yaml_path)
    half_extent = min(half_x, half_y)
    family_indices = _family_start_indices(heading_angles)
    family_norms = _family_norms(heading_angles)
    distance_labels = _target_distance_labels(half_extent, goal_tolerance, grid_resolution)

    templates: list[dict[str, Any]] = []

    for delta_heading_steps in (-2, -1, 1, 2):
        direction = "right" if delta_heading_steps < 0 else "left"
        magnitude = abs(delta_heading_steps)
        templates.append(_rotate_template(f"rotate_{direction}_{magnitude}", delta_heading_steps))

    move_specs = [
        ("forward", 0.0, ("micro", "short", "medium", "long")),
        ("backward", math.pi, ("micro", "short", "medium")),
        ("front_left", math.pi / 4.0, ("micro", "short")),
        ("front_right", -math.pi / 4.0, ("micro", "short")),
        ("back_left", 3.0 * math.pi / 4.0, ("micro", "short")),
        ("back_right", -3.0 * math.pi / 4.0, ("micro", "short")),
        ("left", math.pi / 2.0, ("micro", "short", "medium", "long")),
        ("right", -math.pi / 2.0, ("micro", "short", "medium", "long")),
    ]

    for family_name, start_indices in family_indices.items():
        family_norm = family_norms[family_name]
        for move_name, direction, labels in move_specs:
            for label in labels:
                templates.append(
                    _translate_template(
                        name=f"{move_name}_{family_name}_{label}",
                        description=(
                            f"{move_name.replace('_', ' ')} move for {family_name} headings, "
                            f"tuned for footprint {2 * half_x:.2f} x {2 * half_y:.2f} m."
                        ),
                        start_heading_indices=start_indices,
                        direction=direction,
                        distance=_translation_distance(
                            move_name=move_name,
                            target_distance=distance_labels[label],
                            family_norm=family_norm,
                            grid_resolution=grid_resolution,
                        ),
                    )
                )

    arc_templates = [template for template in bootstrapped["templates"] if template["name"].startswith("arc_")]
    templates.extend(arc_templates)

    return {
        "version": 2.0,
        "description": (
            "Rich omni lattice built from the current vehicle footprint and the legacy arc primitives. "
            "Source of truth for the high-coverage Smac lattice."
        ),
        "date_generated": dt.date.today().isoformat(),
        "generation": {
            "linear_sampling_step": grid_resolution,
            "angular_sampling_step": bootstrapped["generation"]["angular_sampling_step"],
            "pose_rounding": 5,
            "yaw_rounding": 12,
        },
        "vehicle_profile": {
            "footprint_half_x": half_x,
            "footprint_half_y": half_y,
            "goal_xy_tolerance": goal_tolerance,
            "family_target_distances": distance_labels,
        },
        "lattice_metadata": metadata,
        "templates": templates,
    }


def build_rich_omni_files(
    *,
    nav2_yaml_path: str | Path,
    lattice_config_path: str | Path,
    base_lattice_path: str | Path,
    output_spec_path: str | Path,
    output_lattice_path: str | Path,
) -> None:
    spec = build_rich_omni_spec(
        nav2_yaml_path=nav2_yaml_path,
        lattice_config_path=lattice_config_path,
        base_lattice_path=base_lattice_path,
    )
    _write_json(output_spec_path, spec)
    generate_lattice_file(output_spec_path, output_lattice_path)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build a rich omni lattice preset from the current UGV footprint.")
    parser.add_argument("--nav2-config", required=True, help="Path to nav2.yaml for extracting the footprint.")
    parser.add_argument("--lattice-config", required=True, help="Path to the base lattice config JSON.")
    parser.add_argument("--base-lattice", required=True, help="Path to the existing lattice JSON used as the arc seed.")
    parser.add_argument("--output-spec", required=True, help="Path to write the generated source spec JSON.")
    parser.add_argument("--output-lattice", required=True, help="Path to write the expanded lattice JSON.")
    parser.add_argument("--validate", action="store_true", help="Validate the generated lattice against the generated spec.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    build_rich_omni_files(
        nav2_yaml_path=args.nav2_config,
        lattice_config_path=args.lattice_config,
        base_lattice_path=args.base_lattice,
        output_spec_path=args.output_spec,
        output_lattice_path=args.output_lattice,
    )

    if args.validate:
        errors = validate_lattice_file(args.output_lattice, spec_path=args.output_spec)
        if errors:
            for error in errors:
                print(error)
            return 1
        print("Generated rich omni lattice validated successfully.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
