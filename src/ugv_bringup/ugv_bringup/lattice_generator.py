from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any


EPSILON = 1e-9
ANGLE_EPSILON = 1e-6


class LatticeError(RuntimeError):
    pass


def _load_json(path: str | Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as input_stream:
        return json.load(input_stream)


def _write_json(path: str | Path, payload: dict[str, Any]) -> None:
    with open(path, "w", encoding="utf-8") as output_stream:
        json.dump(payload, output_stream, ensure_ascii=True, indent=2)
        output_stream.write("\n")


def _normalize_angle_signed(angle: float) -> float:
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    while angle > math.pi:
        angle -= 2.0 * math.pi
    return angle


def _normalize_angle_positive(angle: float) -> float:
    wrapped = math.fmod(angle, 2.0 * math.pi)
    if wrapped < 0.0:
        wrapped += 2.0 * math.pi
    if abs(wrapped - 2.0 * math.pi) < ANGLE_EPSILON:
        return 0.0
    return wrapped


def _shortest_angular_distance(from_angle: float, to_angle: float) -> float:
    return _normalize_angle_signed(to_angle - from_angle)


def _round_value(value: float, digits: int) -> float:
    rounded = round(float(value), digits)
    if abs(rounded) < 10 ** (-digits):
        return 0.0
    return rounded


def _rotate_point(x: float, y: float, angle: float) -> tuple[float, float]:
    return (
        math.cos(angle) * x - math.sin(angle) * y,
        math.sin(angle) * x + math.cos(angle) * y,
    )


def _path_length(poses: list[list[float]]) -> float:
    total = 0.0
    previous_x = 0.0
    previous_y = 0.0
    for x, y, _ in poses:
        total += math.hypot(x - previous_x, y - previous_y)
        previous_x = x
        previous_y = y
    return total


def _infer_pose_metrics(local_poses: list[list[float]]) -> dict[str, float]:
    if not local_poses:
        raise LatticeError("A template must contain at least one pose.")

    total_length = _path_length(local_poses)
    if total_length < EPSILON:
        return {
            "trajectory_radius": 0.0,
            "trajectory_length": 0.0,
            "arc_length": 0.0,
            "straight_length": 0.0,
        }

    headings = [pose[2] for pose in local_poses]
    if all(abs(_shortest_angular_distance(headings[0], heading)) < ANGLE_EPSILON for heading in headings[1:]):
        straight_length = math.hypot(local_poses[-1][0], local_poses[-1][1])
        return {
            "trajectory_radius": 0.0,
            "trajectory_length": straight_length,
            "arc_length": 0.0,
            "straight_length": straight_length,
        }

    final_heading = headings[-1]
    last_nonfinal_heading_index = max(
        index for index, heading in enumerate(headings) if abs(_shortest_angular_distance(heading, final_heading)) >= ANGLE_EPSILON
    )
    arc_end_index = min(last_nonfinal_heading_index + 1, len(local_poses) - 1)

    arc_end_x, arc_end_y, arc_heading_signed = local_poses[arc_end_index]
    arc_heading = abs(arc_heading_signed)
    radius_candidates = []
    if abs(math.sin(arc_heading_signed)) >= ANGLE_EPSILON:
        radius_candidates.append(arc_end_x / math.sin(arc_heading_signed))
    one_minus_cos = 1.0 - math.cos(arc_heading_signed)
    if abs(one_minus_cos) >= ANGLE_EPSILON:
        radius_candidates.append(arc_end_y / one_minus_cos)

    signed_radius = sum(radius_candidates) / len(radius_candidates) if radius_candidates else 0.0
    radius = abs(signed_radius)
    arc_length = radius * arc_heading
    final_x, final_y, _ = local_poses[-1]
    straight_length = math.hypot(final_x - arc_end_x, final_y - arc_end_y)
    total_length = arc_length + straight_length

    return {
        "trajectory_radius": radius,
        "trajectory_length": total_length,
        "arc_length": arc_length,
        "straight_length": straight_length,
    }


def _resolve_heading_index(angle: float, heading_angles: list[float], tolerance: float = 1e-4) -> int:
    positive_angle = _normalize_angle_positive(angle)
    best_index = -1
    best_error = float("inf")
    for index, candidate in enumerate(heading_angles):
        error = abs(_shortest_angular_distance(candidate, positive_angle))
        if error < best_error:
            best_index = index
            best_error = error
    if best_error > tolerance:
        raise LatticeError(f"Unable to match heading {positive_angle} to configured heading angles.")
    return best_index


def _resolve_start_indices(template: dict[str, Any], num_headings: int) -> list[int]:
    raw_indices = template.get("start_heading_indices")
    if raw_indices is None:
        return list(range(num_headings))

    indices = sorted({int(index) for index in raw_indices})
    invalid = [index for index in indices if index < 0 or index >= num_headings]
    if invalid:
        raise LatticeError(f"Template '{template.get('name', '<unnamed>')}' has invalid heading indices: {invalid}")
    return indices


def _resolve_delta_yaw(segment: dict[str, Any], heading_angles: list[float], start_heading_index: int) -> float:
    if "delta_yaw" in segment:
        return float(segment["delta_yaw"])
    if "delta_heading_steps" in segment:
        end_heading_index = (start_heading_index + int(segment["delta_heading_steps"])) % len(heading_angles)
        return _shortest_angular_distance(heading_angles[start_heading_index], heading_angles[end_heading_index])
    raise LatticeError("Rotate/arc segments require either 'delta_yaw' or 'delta_heading_steps'.")


def _generate_segment_sequence(
    *,
    template: dict[str, Any],
    heading_angles: list[float],
    generation_cfg: dict[str, Any],
    start_heading_index: int,
) -> list[list[float]]:
    linear_step = float(generation_cfg["linear_sampling_step"])
    angular_step = float(generation_cfg["angular_sampling_step"])
    poses: list[list[float]] = []
    current_x = 0.0
    current_y = 0.0
    current_yaw = 0.0

    for segment_index, segment in enumerate(template.get("segments", [])):
        segment_type = segment["type"]
        if segment_type == "translate":
            distance = float(segment["distance"])
            direction = float(segment.get("direction", 0.0))
            count = max(1, round(abs(distance) / linear_step))
            for step in range(1, count + 1):
                fraction = step / count
                travel = distance * fraction
                sample_x = current_x + travel * math.cos(current_yaw + direction)
                sample_y = current_y + travel * math.sin(current_yaw + direction)
                poses.append([sample_x, sample_y, current_yaw])
            current_x, current_y = poses[-1][0], poses[-1][1]
            continue

        if segment_type == "rotate":
            delta_yaw = _resolve_delta_yaw(segment, heading_angles, start_heading_index)
            count = max(1, round(abs(delta_yaw) / angular_step))
            if segment_index == 0 and not poses:
                poses.append([current_x, current_y, current_yaw])
            for step in range(1, count + 1):
                fraction = step / count
                poses.append([current_x, current_y, current_yaw + delta_yaw * fraction])
            current_yaw = poses[-1][2]
            continue

        if segment_type == "arc":
            radius = float(segment["radius"])
            delta_yaw = _resolve_delta_yaw(segment, heading_angles, start_heading_index)
            arc_length = abs(radius * delta_yaw)
            count = max(1, round(arc_length / linear_step))
            start_x = current_x
            start_y = current_y
            start_yaw = current_yaw
            for step in range(1, count + 1):
                fraction = step / count
                sample_delta = delta_yaw * fraction
                local_x = radius * math.sin(sample_delta)
                local_y = radius * (1.0 - math.cos(sample_delta))
                rotated_x, rotated_y = _rotate_point(local_x, local_y, start_yaw)
                poses.append([start_x + rotated_x, start_y + rotated_y, start_yaw + sample_delta])
            current_x, current_y, current_yaw = poses[-1]
            continue

        raise LatticeError(f"Unsupported segment type: {segment_type}")

    return poses


def _local_poses_for_template(
    *,
    template: dict[str, Any],
    heading_angles: list[float],
    generation_cfg: dict[str, Any],
    start_heading_index: int,
) -> list[list[float]]:
    template_type = template["template_type"]
    if template_type == "pose_sequence":
        return copy.deepcopy(template["poses"])
    if template_type == "segment_sequence":
        return _generate_segment_sequence(
            template=template,
            heading_angles=heading_angles,
            generation_cfg=generation_cfg,
            start_heading_index=start_heading_index,
        )
    raise LatticeError(f"Unsupported template_type: {template_type}")


def _transform_local_poses(
    *,
    local_poses: list[list[float]],
    start_heading: float,
    pose_rounding: int,
    yaw_rounding: int,
) -> list[list[float]]:
    transformed = []
    for local_x, local_y, local_yaw in local_poses:
        world_x, world_y = _rotate_point(local_x, local_y, start_heading)
        world_yaw = _normalize_angle_positive(start_heading + local_yaw)
        transformed.append(
            [
                _round_value(world_x, pose_rounding),
                _round_value(world_y, pose_rounding),
                _round_value(world_yaw, yaw_rounding),
            ]
        )
    return transformed


def _default_generation_cfg(spec: dict[str, Any]) -> dict[str, Any]:
    metadata = spec["lattice_metadata"]
    heading_angles = metadata["heading_angles"]
    smallest_step = min(
        (heading_angles[(index + 1) % len(heading_angles)] - heading_angles[index]) % (2.0 * math.pi)
        for index in range(len(heading_angles))
    )
    generation_cfg = copy.deepcopy(spec.get("generation", {}))
    generation_cfg.setdefault("linear_sampling_step", metadata["grid_resolution"])
    generation_cfg.setdefault("angular_sampling_step", smallest_step / 2.0)
    generation_cfg.setdefault("pose_rounding", 5)
    generation_cfg.setdefault("yaw_rounding", 12)
    return generation_cfg


def _infer_left_turn(template: dict[str, Any], local_poses: list[list[float]]) -> bool:
    if "left_turn" in template:
        return bool(template["left_turn"])
    if not local_poses:
        return True
    return local_poses[-1][2] >= 0.0


def _resolve_template_metrics(template: dict[str, Any], local_poses: list[list[float]]) -> dict[str, float]:
    raw_metrics = template.get("metrics")
    if raw_metrics is None:
        return _infer_pose_metrics(local_poses)
    return {
        "trajectory_radius": float(raw_metrics["trajectory_radius"]),
        "trajectory_length": float(raw_metrics["trajectory_length"]),
        "arc_length": float(raw_metrics["arc_length"]),
        "straight_length": float(raw_metrics["straight_length"]),
    }


def _primitive_sort_key(primitive: dict[str, Any], local_poses: list[list[float]], num_headings: int) -> tuple[int, int, int, float, float]:
    delta_heading = (primitive["end_angle_index"] - primitive["start_angle_index"]) % num_headings
    local_end_x, local_end_y, _ = local_poses[-1]

    if delta_heading == (num_headings - 1):
        return (0, 0 if primitive["trajectory_length"] > 0.0 else 1, 0, local_end_x, abs(local_end_y))

    if delta_heading == 0:
        if local_end_x > 0.0 and abs(local_end_y) < ANGLE_EPSILON:
            lateral_bucket = 0
        elif abs(local_end_x) < ANGLE_EPSILON and local_end_y > 0.0:
            lateral_bucket = 1
        elif abs(local_end_x) < ANGLE_EPSILON and local_end_y < 0.0:
            lateral_bucket = 2
        else:
            lateral_bucket = 3
        return (1, lateral_bucket, 0, local_end_x, abs(local_end_y))

    if delta_heading == 1:
        return (2, 0 if primitive["trajectory_length"] > 0.0 else 1, 0, local_end_x, abs(local_end_y))

    return (3, delta_heading, 0, local_end_x, abs(local_end_y))


def load_lattice_spec(path: str | Path) -> dict[str, Any]:
    spec = _load_json(path)
    if "lattice_metadata" not in spec:
        raise LatticeError(f"Lattice spec '{path}' is missing 'lattice_metadata'.")
    if "templates" not in spec:
        raise LatticeError(f"Lattice spec '{path}' is missing 'templates'.")
    return spec


def generate_lattice_from_spec(spec: dict[str, Any], *, date_generated: str | None = None) -> dict[str, Any]:
    generation_cfg = _default_generation_cfg(spec)
    metadata = copy.deepcopy(spec["lattice_metadata"])
    heading_angles = metadata["heading_angles"]
    pose_rounding = int(generation_cfg["pose_rounding"])
    yaw_rounding = int(generation_cfg["yaw_rounding"])

    primitives = []
    for start_heading_index in range(metadata["num_of_headings"]):
        start_heading = heading_angles[start_heading_index]
        start_primitives = []
        for template in spec["templates"]:
            if start_heading_index not in _resolve_start_indices(template, metadata["num_of_headings"]):
                continue

            local_poses = _local_poses_for_template(
                template=template,
                heading_angles=heading_angles,
                generation_cfg=generation_cfg,
                start_heading_index=start_heading_index,
            )
            if not local_poses:
                raise LatticeError(f"Template '{template.get('name', '<unnamed>')}' generated no poses.")

            metrics = _resolve_template_metrics(template, local_poses)
            world_poses = _transform_local_poses(
                local_poses=local_poses,
                start_heading=start_heading,
                pose_rounding=pose_rounding,
                yaw_rounding=yaw_rounding,
            )
            end_angle_index = _resolve_heading_index(world_poses[-1][2], heading_angles)
            start_primitives.append(
                (
                    _primitive_sort_key(
                        {
                            "start_angle_index": start_heading_index,
                            "end_angle_index": end_angle_index,
                            "trajectory_length": metrics["trajectory_length"],
                        },
                        local_poses,
                        metadata["num_of_headings"],
                    ),
                    {
                        "start_angle_index": start_heading_index,
                        "end_angle_index": end_angle_index,
                        "left_turn": _infer_left_turn(template, local_poses),
                        "trajectory_radius": _round_value(metrics["trajectory_radius"], 5),
                        "trajectory_length": _round_value(metrics["trajectory_length"], 5),
                        "arc_length": _round_value(metrics["arc_length"], 5),
                        "straight_length": _round_value(metrics["straight_length"], 5),
                        "poses": world_poses,
                    },
                )
            )
        for _, primitive in sorted(start_primitives, key=lambda item: item[0]):
            primitive["trajectory_id"] = len(primitives)
            primitives.append(primitive)

    metadata["number_of_trajectories"] = len(primitives)
    return {
        "version": spec.get("version", 1.0),
        "date_generated": date_generated or spec.get("date_generated") or dt.date.today().isoformat(),
        "lattice_metadata": metadata,
        "primitives": primitives,
    }


def generate_lattice_file(spec_path: str | Path, output_path: str | Path, *, date_generated: str | None = None) -> dict[str, Any]:
    spec = load_lattice_spec(spec_path)
    lattice = generate_lattice_from_spec(spec, date_generated=date_generated)
    _write_json(output_path, lattice)
    return lattice


def _validate_grid_alignment(x: float, y: float, grid_resolution: float, tolerance: float = 1e-4) -> bool:
    return all(abs(round(value / grid_resolution) * grid_resolution - value) <= tolerance for value in (x, y))


def validate_lattice(lattice: dict[str, Any], *, spec: dict[str, Any] | None = None) -> list[str]:
    errors: list[str] = []
    metadata = lattice["lattice_metadata"]
    heading_angles = metadata["heading_angles"]
    grid_resolution = float(metadata["grid_resolution"])

    if metadata["num_of_headings"] != len(heading_angles):
        errors.append(
            f"Metadata mismatch: num_of_headings={metadata['num_of_headings']} but heading_angles has {len(heading_angles)} entries."
        )

    primitives = lattice["primitives"]
    if metadata["number_of_trajectories"] != len(primitives):
        errors.append(
            f"Metadata mismatch: number_of_trajectories={metadata['number_of_trajectories']} but found {len(primitives)} primitives."
        )

    for expected_id, primitive in enumerate(primitives):
        if primitive["trajectory_id"] != expected_id:
            errors.append(f"Primitive at index {expected_id} has trajectory_id={primitive['trajectory_id']}.")
            continue

        if not primitive["poses"]:
            errors.append(f"Primitive {expected_id} has no poses.")
            continue

        for pose_index, (x, y, yaw) in enumerate(primitive["poses"]):
            if yaw < -ANGLE_EPSILON or yaw >= 2.0 * math.pi + ANGLE_EPSILON:
                errors.append(f"Primitive {expected_id} pose {pose_index} has yaw outside [0, 2pi): {yaw}.")

        end_x, end_y, end_yaw = primitive["poses"][-1]
        if not _validate_grid_alignment(end_x, end_y, grid_resolution):
            errors.append(f"Primitive {expected_id} ends off-grid at ({end_x}, {end_y}).")

        try:
            resolved_end_index = _resolve_heading_index(end_yaw, heading_angles)
        except LatticeError as error:
            errors.append(f"Primitive {expected_id} has invalid final yaw: {error}")
            continue

        if primitive["end_angle_index"] != resolved_end_index:
            errors.append(
                f"Primitive {expected_id} end_angle_index={primitive['end_angle_index']} does not match its final pose yaw."
            )

        if primitive["trajectory_length"] < 0.0 or primitive["arc_length"] < 0.0 or primitive["straight_length"] < 0.0:
            errors.append(f"Primitive {expected_id} has negative length metadata.")
        if abs((primitive["arc_length"] + primitive["straight_length"]) - primitive["trajectory_length"]) > 2e-4:
            errors.append(f"Primitive {expected_id} has inconsistent arc/straight/total lengths.")

    if spec is not None:
        generated = generate_lattice_from_spec(spec, date_generated=lattice.get("date_generated"))
        actual = copy.deepcopy(lattice)
        if actual != generated:
            errors.append("Checked-in lattice does not match the supplied source spec.")

    return errors


def validate_lattice_file(lattice_path: str | Path, *, spec_path: str | Path | None = None) -> list[str]:
    lattice = _load_json(lattice_path)
    spec = load_lattice_spec(spec_path) if spec_path else None
    return validate_lattice(lattice, spec=spec)


def _localize_world_poses(
    poses: list[list[float]],
    *,
    start_heading: float,
    pose_rounding: int,
    yaw_rounding: int,
) -> list[list[float]]:
    localized = []
    for x, y, yaw in poses:
        local_x = math.cos(start_heading) * x + math.sin(start_heading) * y
        local_y = -math.sin(start_heading) * x + math.cos(start_heading) * y
        localized.append(
            [
                _round_value(local_x, pose_rounding),
                _round_value(local_y, pose_rounding),
                _round_value(_shortest_angular_distance(start_heading, yaw), yaw_rounding),
            ]
        )
    return localized


def _infer_bootstrap_template_name(index: int, final_pose: list[float]) -> str:
    x, y, yaw = final_pose
    if abs(x) < ANGLE_EPSILON and abs(y) < ANGLE_EPSILON and abs(yaw) > ANGLE_EPSILON:
        prefix = "rotate_left" if yaw > 0.0 else "rotate_right"
        return f"{prefix}_group_{index:02d}"
    if abs(yaw) < ANGLE_EPSILON:
        if abs(y) < ANGLE_EPSILON and x > 0.0:
            return f"translate_forward_group_{index:02d}"
        if abs(x) < ANGLE_EPSILON and y > 0.0:
            return f"translate_left_group_{index:02d}"
        if abs(x) < ANGLE_EPSILON and y < 0.0:
            return f"translate_right_group_{index:02d}"
    if x > 0.0 and y > 0.0 and yaw > 0.0:
        return f"arc_left_group_{index:02d}"
    if x > 0.0 and y < 0.0 and yaw < 0.0:
        return f"arc_right_group_{index:02d}"
    return f"template_group_{index:02d}"


def bootstrap_spec_from_lattice(lattice: dict[str, Any]) -> dict[str, Any]:
    metadata = lattice["lattice_metadata"]
    heading_angles = metadata["heading_angles"]
    groups: defaultdict[tuple[Any, ...], list[dict[str, Any]]] = defaultdict(list)
    for primitive in lattice["primitives"]:
        start_heading = heading_angles[primitive["start_angle_index"]]
        local_poses = _localize_world_poses(
            primitive["poses"],
            start_heading=start_heading,
            pose_rounding=5,
            yaw_rounding=8,
        )
        key = (
            tuple(tuple(pose) for pose in local_poses),
            bool(primitive["left_turn"]),
        )
        groups[key].append(primitive)

    templates = []
    grouped_items = sorted(groups.values(), key=lambda primitives: min(primitive["trajectory_id"] for primitive in primitives))
    for index, primitives in enumerate(grouped_items):
        first = min(primitives, key=lambda primitive: primitive["trajectory_id"])
        local_poses = _localize_world_poses(
            first["poses"],
            start_heading=heading_angles[first["start_angle_index"]],
            pose_rounding=5,
            yaw_rounding=8,
        )
        templates.append(
            {
                "name": _infer_bootstrap_template_name(index, local_poses[-1]),
                "description": f"Bootstrapped from trajectories {min(p['trajectory_id'] for p in primitives)}..{max(p['trajectory_id'] for p in primitives)}",
                "template_type": "pose_sequence",
                "start_heading_indices": sorted({primitive["start_angle_index"] for primitive in primitives}),
                "left_turn": bool(first["left_turn"]),
                "metrics": {
                    "trajectory_radius": first["trajectory_radius"],
                    "trajectory_length": first["trajectory_length"],
                    "arc_length": first["arc_length"],
                    "straight_length": first["straight_length"],
                },
                "poses": local_poses,
            }
        )

    smallest_step = min(
        (heading_angles[(index + 1) % len(heading_angles)] - heading_angles[index]) % (2.0 * math.pi)
        for index in range(len(heading_angles))
    )

    return {
        "version": lattice.get("version", 1.0),
        "description": "Source spec bootstrapped from an expanded lattice JSON.",
        "date_generated": lattice.get("date_generated"),
        "generation": {
            "linear_sampling_step": metadata["grid_resolution"],
            "angular_sampling_step": _round_value(smallest_step / 2.0, 12),
            "pose_rounding": 5,
            "yaw_rounding": 12,
        },
        "lattice_metadata": copy.deepcopy(metadata),
        "templates": templates,
    }


def bootstrap_spec_file(lattice_path: str | Path, output_path: str | Path) -> dict[str, Any]:
    lattice = _load_json(lattice_path)
    spec = bootstrap_spec_from_lattice(lattice)
    _write_json(output_path, spec)
    return spec


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate and maintain Nav2 Smac lattice primitive files.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate_parser = subparsers.add_parser("generate", help="Generate an expanded lattice JSON from a source spec.")
    generate_parser.add_argument("--spec", required=True, help="Path to the source lattice spec JSON.")
    generate_parser.add_argument("--output", required=True, help="Path to write the expanded lattice JSON.")
    generate_parser.add_argument("--date-generated", help="Override the generated date stamp.")

    validate_parser = subparsers.add_parser("validate", help="Validate a lattice JSON, optionally against its source spec.")
    validate_parser.add_argument("--lattice", required=True, help="Path to the expanded lattice JSON.")
    validate_parser.add_argument("--spec", help="Optional path to the source lattice spec JSON.")

    bootstrap_parser = subparsers.add_parser(
        "bootstrap-spec", help="Create a source spec by compressing an existing expanded lattice JSON."
    )
    bootstrap_parser.add_argument("--lattice", required=True, help="Path to the expanded lattice JSON.")
    bootstrap_parser.add_argument("--output", required=True, help="Path to write the bootstrapped source spec JSON.")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)

    if args.command == "generate":
        generate_lattice_file(args.spec, args.output, date_generated=args.date_generated)
        return 0

    if args.command == "validate":
        errors = validate_lattice_file(args.lattice, spec_path=args.spec)
        if errors:
            for error in errors:
                print(error)
            return 1
        print("Lattice validation passed.")
        return 0

    if args.command == "bootstrap-spec":
        bootstrap_spec_file(args.lattice, args.output)
        return 0

    parser.error(f"Unsupported command: {args.command}")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
