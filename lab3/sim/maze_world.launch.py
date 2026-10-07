"""Start Gazebo with a maze world and a TurtleBot3 Burger in it.

This file is not part of the ROS package. Run it by path:

    ros2 launch ~/R7021E-group6/lab3/sim/maze_world.launch.py world:=lab3_maze_small

The worlds are in the worlds/ folder next to this file. The first two were made with the course
maze generator. The blind worlds were made with a small script in the same style. Cell size is
0.8 m.

    lab3_maze_small   4.0 m by 4.0 m, 25 cells, for quick tests
    lab3_maze         7.2 m by 7.2 m, 81 cells, for the full run
    lab3_maze_blind_a 5.6 m by 5.6 m, 49 cells, a layout you have not seen
    lab3_maze_blind_b 7.2 m by 7.2 m, 81 cells, a layout you have not seen

For the two blind worlds, use gui:=false and watch only RViz. This is how the hall maze looks
to you: you learn the layout from the map that the robot builds.

The cell around the world origin is free on all sides, so the robot spawns at (0, 0).

Gazebo does not always stop on Ctrl-C. After you stop this launch, check for leftovers:

    ps aux | grep -E "[g]z sim"

The ground plane in the worlds is written into the file. The course generator used a
download from Fuel, which fails with no network.
"""

import os

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    worlds = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'worlds')

    turtlebot3_gazebo = FindPackageShare('turtlebot3_gazebo')
    ros_gz_sim = FindPackageShare('ros_gz_sim')

    arguments = [
        DeclareLaunchArgument(
            'world', default_value='lab3_maze_small',
            description='World file name in worlds/, without .world'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='Start the Gazebo window. Use false for a run with no window'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='Spawn x'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Spawn y'),
    ]

    world = PathJoinSubstitution([worlds, [LaunchConfiguration('world'), '.world']])

    # The Burger model needs its meshes from turtlebot3_gazebo/models.
    resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', PathJoinSubstitution([turtlebot3_gazebo, 'models']))

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim, 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': '-g -v2 '}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py'])),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [turtlebot3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'])),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
        }.items(),
    )

    # There is no second /clock bridge here on purpose. spawn_turtlebot3.launch.py already
    # bridges /clock. Two publishers on /clock make simulation time jump back, TF clears
    # its buffer, and the SLAM map shows doubled walls.

    return LaunchDescription(
        arguments + [resources, server, gui, robot_state_publisher, spawn])
