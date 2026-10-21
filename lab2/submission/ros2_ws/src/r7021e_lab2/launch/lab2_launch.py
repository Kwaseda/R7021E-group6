"""Lab 2, all four tasks.

    ros2 launch r7021e_lab2 lab2_launch.py task:=1 sim:=true
    ros2 launch r7021e_lab2 lab2_launch.py task:=4 sim:=true
    ros2 launch r7021e_lab2 lab2_launch.py task:=1 domain_id:=32
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('r7021e_lab2')
    sim = LaunchConfiguration('sim')
    use_sim_time = ParameterValue(sim, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('task', default_value='1', choices=['1', '2', '3', '4']),
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('t_step', default_value='0.1'),
        DeclareLaunchArgument('n_horizon', default_value='20'),
        DeclareLaunchArgument('domain_id', default_value='32'),

        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),

        Node(
            package='r7021e_lab2', executable='mpc_node', name='mpc_node', output='screen',
            parameters=[{
                'task': ParameterValue(LaunchConfiguration('task'), value_type=int),
                't_step': ParameterValue(LaunchConfiguration('t_step'), value_type=float),
                'n_horizon': ParameterValue(LaunchConfiguration('n_horizon'), value_type=int),
                'use_sim_time': use_sim_time,
            }]),

        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'lab2.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'), 'launch',
                'turtlebot3_dqn_stage1.launch.py'])),
            condition=IfCondition(sim)),
    ])
