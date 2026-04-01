from pathlib import Path


_PACKAGE_NAME = "ugv_bringup"


def get_workspace_root(package_name: str = _PACKAGE_NAME, package_prefix: str | Path | None = None) -> Path:
    if package_prefix is None:
        from ament_index_python.packages import get_package_prefix

        package_prefix_path = Path(get_package_prefix(package_name))
    else:
        package_prefix_path = Path(package_prefix)

    for candidate in (package_prefix_path, *package_prefix_path.parents):
        if candidate.name == "install" and candidate.parent != candidate:
            return candidate.parent

    raise RuntimeError(
        f"Failed to resolve workspace root from package prefix: {package_prefix_path}"
    )


def get_workspace_maps_dir(package_name: str = _PACKAGE_NAME, package_prefix: str | Path | None = None) -> Path:
    return get_workspace_root(package_name, package_prefix) / "maps"


def pick_latest_map_yaml(
    package_name: str = _PACKAGE_NAME,
    package_prefix: str | Path | None = None,
    maps_dir: str | Path | None = None,
) -> str:
    resolved_maps_dir = Path(maps_dir) if maps_dir is not None else get_workspace_maps_dir(package_name, package_prefix)
    numeric_maps: list[tuple[int, Path]] = []
    newest_map: Path | None = None
    newest_mtime = -1.0

    if resolved_maps_dir.is_dir():
        for path in resolved_maps_dir.glob("*.yaml"):
            if not path.is_file():
                continue
            if path.stem.isdigit():
                numeric_maps.append((int(path.stem), path))
            mtime = path.stat().st_mtime
            if mtime > newest_mtime:
                newest_mtime = mtime
                newest_map = path

    if numeric_maps:
        numeric_maps.sort(key=lambda item: item[0])
        return str(numeric_maps[-1][1])
    if newest_map is not None:
        return str(newest_map)

    raise FileNotFoundError(
        f"No map yaml found under workspace root maps directory: {resolved_maps_dir}. "
        f"Create a map in {resolved_maps_dir} or pass an explicit map:=... path."
    )


def resolve_map_yaml(
    map_value: str,
    package_name: str = _PACKAGE_NAME,
    package_prefix: str | Path | None = None,
) -> str:
    raw_value = map_value.strip()
    if not raw_value:
        return pick_latest_map_yaml(package_name, package_prefix)

    path = Path(raw_value).expanduser()
    if not path.is_absolute():
        path = get_workspace_root(package_name, package_prefix) / path

    resolved = path.resolve(strict=False)
    if not resolved.is_file():
        raise FileNotFoundError(f"Map yaml not found: {resolved}")

    return str(resolved)
