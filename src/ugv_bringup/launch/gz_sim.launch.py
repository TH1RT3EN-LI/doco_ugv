import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    sim_worlds_share = get_package_share_directory("sim_worlds")

    world = LaunchConfiguration("world")
    world_sdf_path = LaunchConfiguration("world_sdf_path")
    headless = LaunchConfiguration("headless")
    render_engine = LaunchConfiguration("render_engine")
    gz_partition = LaunchConfiguration("gz_partition")

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_worlds_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "world": world,
            "world_sdf_path": world_sdf_path,
            "headless": headless,
            "render_engine": render_engine,
            "gz_partition": gz_partition,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=EnvironmentVariable("UGV_WORLD", default_value="test"),
            description="Registered sim_worlds world id",
        ),
        DeclareLaunchArgument("world_sdf_path", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("gz_partition", default_value=EnvironmentVariable("GZ_PARTITION", default_value="")),
        DeclareLaunchArgument("render_engine", default_value=EnvironmentVariable("GZ_RENDER_ENGINE", default_value="ogre2")),
        gz_launch,
    ])
