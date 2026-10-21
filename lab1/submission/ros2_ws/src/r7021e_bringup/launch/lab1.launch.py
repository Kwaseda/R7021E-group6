"""One launch file for all four tasks of Lab 1.

    ros2 launch r7021e_bringup lab1.launch.py                      task 1, terminal setpoints
    ros2 launch r7021e_bringup lab1.launch.py trajectory:=true     tasks 2 and 3, the eight
    ros2 launch r7021e_bringup lab1.launch.py mode:=wall_following task 4

Arguments:

    mode          position_control | wall_following. Which node drives /cmd_vel.
    trajectory    start the figure of eight generator. Ignored in wall_following mode.
    domain_id     ROS_DOMAIN_ID for every node started here. The lab dictates 3<robot
                  number>, so turtle2 is 32.
    use_sim_time  true in Gazebo, false on the robot.
    rviz          start RViz2 with the saved configuration.
    sim           include the Gazebo world. Off by default.
    world_launch  which world. Defaults to turtlebot3_dqn_stage1.launch.py -- see the
                  note below.
    config_dir    where the parameter files are. Defaults to this package's share.

mode is an argument rather than two launch files because controller_node and
wall_follower_node both publish to /cmd_vel, and two publishers on one topic
interleave rather than erroring.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'r7021e_bringup'


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)

    arguments = [
        DeclareLaunchArgument(
            'mode', default_value='position_control',
            choices=['position_control', 'wall_following'],
            description='which node drives /cmd_vel'),
        DeclareLaunchArgument(
            'trajectory', default_value='false',
            choices=['true', 'false'],
            description='publish the figure of eight on /new_position'),
        DeclareLaunchArgument(
            'domain_id', default_value='32',
            description='ROS_DOMAIN_ID, the lab uses 3<robot number>. turtle2 is 32'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            choices=['true', 'false'],
            description='true in Gazebo, false on the robot'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            choices=['true', 'false'],
            description='start RViz2 with the saved Lab 1 configuration'),
        DeclareLaunchArgument(
            'sim', default_value='false',
            choices=['true', 'false'],
            description='include the Gazebo world named by world_launch'),
        DeclareLaunchArgument(
            'world_launch', default_value='turtlebot3_dqn_stage1.launch.py',
            description='launch file inside turtlebot3_gazebo that starts the world. '
                        'See the note further down this file'),
        DeclareLaunchArgument(
            'config_dir', default_value=os.path.join(share, 'config'),
            description='directory holding robot.yaml and the per-node parameter files'),
    ]

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_dir = LaunchConfiguration('config_dir')

    # Set for the processes this file starts only -- a second terminal needs its own
    # export for "ros2 topic list" etc. to see the same domain.
    domain = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id'))

    def parameters(node_yaml):
        """robot.yaml first, then the node's own file, so one definition of the
        physical limits serves every node."""
        return [
            PathJoinSubstitution([config_dir, 'robot.yaml']),
            PathJoinSubstitution([config_dir, node_yaml]),
            {'use_sim_time': use_sim_time},
        ]

    driving = IfCondition(PythonExpression(['"', mode, '" == "position_control"']))
    following = IfCondition(PythonExpression(['"', mode, '" == "wall_following"']))
    eight = IfCondition(PythonExpression([
        '"', mode, '" == "position_control" and "',
        LaunchConfiguration('trajectory'), '" == "true"',
    ]))

    nodes = [
        Node(
            package='r7021e_control', executable='controller_node',
            name='controller_node', output='screen',
            parameters=parameters('controller.yaml'), condition=driving),
        Node(
            package='r7021e_control', executable='trajectory_node',
            name='trajectory_node', output='screen',
            parameters=parameters('trajectory.yaml'), condition=eight),
        Node(
            package='r7021e_control', executable='wall_follower_node',
            name='wall_follower_node', output='screen',
            parameters=parameters('wall_follower.yaml'), condition=following),
        # Runs in both modes: task 3 wants the closest wall point during the eight,
        # and the same plot needs task 4's segments marked on it too.
        Node(
            package='r7021e_control', executable='scan_monitor_node',
            name='scan_monitor_node', output='screen',
            parameters=parameters('scan_monitor.yaml')),
        # Redraws /new_position as a Marker for RViz. Runs unconditionally --
        # harmless in wall_following mode, since nothing publishes /new_position then.
        Node(
            package='r7021e_control', executable='goal_marker_node',
            name='goal_marker_node', output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ]

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(share, 'rviz', 'lab1.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # The simulator. The tutorial names box.launch.py, which is not in the upstream
    # turtlebot3_simulations jazzy branch and was never supplied for this course.
    # turtlebot3_dqn_stage1 is the stand-in: a ground plane plus turtlebot3_dqn_world,
    # a 5 m square of walls with inner faces at +/- 2.35 m, robot spawned at the
    # origin. world_launch stays an argument so a different world is a flag, not an
    # edit to this file.
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            # FindPackageShare, not get_package_share_directory: resolved only when
            # this action runs, so a machine without the simulator can still launch
            # the nodes when sim:=false.
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch',
                LaunchConfiguration('world_launch'),
            ])
        ),
        condition=IfCondition(LaunchConfiguration('sim')),
    )

    return LaunchDescription(arguments + [domain] + nodes + [rviz, world])
