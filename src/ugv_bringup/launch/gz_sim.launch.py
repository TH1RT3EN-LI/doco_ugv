from ugv_bringup.deprecated_launch import forward_to_package_launch


def generate_launch_description():
    return forward_to_package_launch(
        target_package="ugv_sim_bringup",
        target_launch_file="gz_sim.launch.py",
        message="[ugv_bringup] 'gz_sim.launch.py' has moved to package 'ugv_sim_bringup'; this wrapper will be removed in a future cleanup.",
    )
