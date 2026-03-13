import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo


def forward_to_package_launch(*, target_package: str, target_launch_file: str, message: str) -> LaunchDescription:
    share_dir = get_package_share_directory(target_package)
    launch_path = os.path.join(share_dir, "launch", target_launch_file)
    module_name = f"{target_package}_{target_launch_file.replace('.', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load launch file: {launch_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    forwarded = module.generate_launch_description()
    return LaunchDescription([LogInfo(msg=message), *forwarded.entities])
