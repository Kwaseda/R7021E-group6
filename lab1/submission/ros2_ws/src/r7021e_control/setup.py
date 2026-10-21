from setuptools import find_packages, setup

package_name = 'r7021e_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Addo',
    maintainer_email='domadd-2@student.ltu.se',
    description='R7021E Lab 1 control nodes: NID position tracking, trajectories, wall following',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = r7021e_control.controller_node:main',
            'trajectory_node = r7021e_control.trajectory_node:main',
            'scan_monitor_node = r7021e_control.scan_monitor_node:main',
            'wall_follower_node = r7021e_control.wall_follower_node:main',
            'goal_marker_node = r7021e_control.goal_marker_node:main',
        ],
    },
)
