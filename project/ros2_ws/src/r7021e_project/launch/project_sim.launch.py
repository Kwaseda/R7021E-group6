"""Gazebo for the R7021E project: the simulator's equivalent of turtlebot3_bringup (decision Q12).

Starts Gazebo with an empty world, the Burger (with or without the plank), the scenario's
obstacles and walls, robot_state_publisher, and the bridges. The MPC node is NOT started
here: run it in its own terminal so you can read what it prints.

    ros2 launch r7021e_project project_sim.launch.py scenario:=s2_030
    ros2 launch r7021e_project project_sim.launch.py scenario:=s1a gui:=false
    ros2 launch r7021e_project project_sim.launch.py scenario:=lab2_t3 plank:=false

The obstacles come from SCENARIOS in mpc_node.py, the same table the node reads, so typing
the same scenario name in both terminals is all it takes to keep them in agreement (Q13).

Topics you get, on top of the course bridge (/odom /tf /scan /cmd_vel /clock ...):
    /ground_truth    geometry_msgs/PoseStamped, Gazebo's exact robot pose. Only the
                     evaluation reads it; the controller only gets /odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from r7021e_project.mpc_node import SCENARIOS


OBSTACLE_HEIGHT = 0.40   # m, taller than the plank (0.21 m) so the plank can really hit it
WALL_THICKNESS = 0.05    # m
ROBOT_NAME = 'burger'    # Gazebo entity name; the ground-truth topic is built from it


def cylinder_sdf(name, radius):
    return ('<sdf version="1.8"><model name="%s"><static>true</static><link name="link">'
            '<pose>0 0 %.3f 0 0 0</pose>'
            '<collision name="c"><geometry><cylinder><radius>%.3f</radius><length>%.3f</length>'
            '</cylinder></geometry></collision>'
            '<visual name="v"><geometry><cylinder><radius>%.3f</radius><length>%.3f</length>'
            '</cylinder></geometry><material><ambient>0.5 0.5 0.5 1</ambient>'
            '<diffuse>0.5 0.5 0.5 1</diffuse></material></visual>'
            '</link></model></sdf>'
            % (name, OBSTACLE_HEIGHT / 2, radius, OBSTACLE_HEIGHT, radius, OBSTACLE_HEIGHT))


def box_sdf(name, size_x, size_y):
    size = '%.3f %.3f %.3f' % (size_x, size_y, OBSTACLE_HEIGHT)
    return ('<sdf version="1.8"><model name="%s"><static>true</static><link name="link">'
            '<pose>0 0 %.3f 0 0 0</pose>'
            '<collision name="c"><geometry><box><size>%s</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>%s</size></box></geometry>'
            '<material><ambient>0.6 0.6 0.6 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse>'
            '</material></visual></link></model></sdf>'
            % (name, OBSTACLE_HEIGHT / 2, size, size))


def spawn(name, x, y, sdf_string=None, sdf_file=None):
    """One ros_gz_sim create call: put a model into the running world."""
    source = ['-string', sdf_string] if sdf_string else ['-file', sdf_file]

    return Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-name', name, '-x', '%.3f' % x, '-y', '%.3f' % y, '-z', '0.01'] + source)


def setup(context):
    scenario = LaunchConfiguration('scenario').perform(context)
    plank = LaunchConfiguration('plank').perform(context) == 'true'
    gui = LaunchConfiguration('gui').perform(context) == 'true'

    if scenario not in SCENARIOS:
        raise RuntimeError('scenario must be one of %s' % sorted(SCENARIOS))
    cfg = SCENARIOS[scenario]

    share = get_package_share_directory('r7021e_project')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    actions = []

    # Gazebo server, and the window only if asked for
    world = os.path.join(tb3, 'worlds', 'empty_world.world')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -s -v2 ' + world, 'on_exit_shutdown': 'true'}.items()))

    if gui:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-g -v2 '}.items()))

    # The robot, with the plank or without
    robot_file = os.path.join(share, 'worlds', 'burger_plank.sdf' if plank else 'burger_bare.sdf')
    actions.append(spawn(ROBOT_NAME, 0.0, 0.0, sdf_file=robot_file))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()))

    # The course bridge: /odom /tf /scan /cmd_vel /clock /imu /joint_states
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='course_bridge', output='screen',
        arguments=['--ros-args', '-p',
                   'config_file:=' + os.path.join(tb3, 'params', 'turtlebot3_burger_bridge.yaml')]))

    # Ground truth, for the evaluation only
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='truth_bridge', output='screen',
        arguments=['/model/%s/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose' % ROBOT_NAME],
        remappings=[('/model/%s/pose' % ROBOT_NAME, '/ground_truth')]))

    # The scenario's obstacles, from the same table the controller reads
    for i, (ox, oy, radius) in enumerate(cfg['obstacles'], start=1):
        actions.append(spawn('obstacle_%d' % i, ox, oy, sdf_string=cylinder_sdf('obstacle_%d' % i, radius)))

    # Scenario 2 corridor walls: inner faces at |y| = walls, along the box's x range
    if cfg['walls']:
        x_min, x_max = cfg['box'][0], cfg['box'][1]
        length = x_max - x_min
        centre_x = (x_min + x_max) / 2
        offset = cfg['walls'] + WALL_THICKNESS / 2

        for name, y in (('wall_left', offset), ('wall_right', -offset)):
            actions.append(spawn(name, centre_x, y, sdf_string=box_sdf(name, length, WALL_THICKNESS)))

    return actions


def generate_launch_description():
    tb3 = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='s1a',
                              description='a key of SCENARIOS in mpc_node.py'),
        DeclareLaunchArgument('plank', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),

        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),

        OpaqueFunction(function=setup),
    ])
