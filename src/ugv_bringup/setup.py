import sys
from setuptools import setup
import os
from glob import glob
package_name = 'ugv_bringup'

def _sanitize_argv(argv):
    drop_exact = {
        "--uninstall",
        "--editable",
        "-e",
    }
    drop_prefix = (
        "--editable=",
        "--build-directory=",
    )
    drop_with_value = {
        "--build-directory",
    }

    out = []
    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg in drop_exact:
            i += 1
            continue
        if any(arg.startswith(prefix) for prefix in drop_prefix):
            i += 1
            continue
        if arg in drop_with_value:
            i += 2
            continue
        out.append(arg)
        i += 1
    return out


sys.argv = _sanitize_argv(sys.argv)

setup(
    name=package_name,
    version='0.0.0',
    packages=['ugv_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        
        (os.path.join('share', package_name, 'config', 'rviz'),
            glob('config/rviz/*.rviz')),
        
        (os.path.join('share', package_name, 'config', 'foxglove'),
            glob('config/foxglove/*.yaml') + glob('config/foxglove/*.json')),

        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='启动车辆的有关软硬件',
    license='GPL-3.0-only',
    entry_points={
        'console_scripts': [],
    },
)
