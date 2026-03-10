from setuptools import find_packages, setup
import os
package_name = 'ugv_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=['ugv_teleop'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [
            'config/joy_launcher.yaml',
            'config/joy_axis_selector.yaml',
            'config/keyboard_teleop.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='litianshun',
    maintainer_email='litianshun.cn@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_launcher_node = ugv_teleop.joy_launcher_node:main',
            'joy_axis_selector_node = ugv_teleop.joy_axis_selector_node:main',
            'keyboard_teleop_node = ugv_teleop.keyboard_teleop_node:main',
        ],
    },
)
