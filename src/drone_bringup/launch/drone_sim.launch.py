"""Bring up the quadcopter world in Gazebo Sim and bridge it to ROS 2.

    ros2 launch drone_bringup drone_sim.launch.py                # GUI
    ros2 launch drone_bringup drone_sim.launch.py headless:=true # no GUI
    ros2 launch drone_bringup drone_sim.launch.py takeoff:=true  # fly on start
    ros2 launch drone_bringup drone_sim.launch.py mission:=true  # fly A -> B

Keyboard control is not launched from here: launch owns the stdin of everything
it starts, and the teleop node needs a terminal of its own. Run it beside the
sim instead:

    ros2 run drone_bringup teleop_key
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ros_gz_bridge.actions import RosGzBridge


def _rendering_env():
    """Point gz-rendering at its ogre2 plugin and media directories.

    The RoboStack conda build bakes in an install prefix that does not survive
    relocation, so the GUI reports "Failed to load plugin [gz-rendering-ogre2]"
    and then cannot find its shader media. Both live under CONDA_PREFIX at
    predictable paths; set them only when they are not already set.
    """
    prefix = os.environ.get('CONDA_PREFIX')
    if not prefix:
        return []

    wanted = {
        'GZ_RENDERING_PLUGIN_PATH':
            os.path.join(prefix, 'lib', 'gz-rendering-8', 'engine-plugins'),
        'GZ_RENDERING_RESOURCE_PATH':
            os.path.join(prefix, 'share', 'gz', 'gz-rendering8'),
    }
    return [
        SetEnvironmentVariable(name, path)
        for name, path in wanted.items()
        if name not in os.environ and os.path.isdir(path)
    ]


def generate_launch_description():
    pkg = get_package_share_directory('drone_bringup')
    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')

    world_path = PathJoinSubstitution([pkg, 'worlds', world])

    # The world pulls the drone in with `model://quadcopter`, so Gazebo needs
    # the installed models directory on its resource path.
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models'))

    def gz(args, condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={'gz_args': args, 'on_exit_shutdown': 'true'}.items(),
            condition=condition,
        )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='drone_world.sdf',
                              description='World file inside drone_bringup/worlds.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the server only, without the Gazebo GUI.'),
        DeclareLaunchArgument('takeoff', default_value='false',
                              description='Arm and climb to `altitude` once the sim is up.'),
        DeclareLaunchArgument('altitude', default_value='3.0',
                              description='Target altitude in metres, above the launch point.'),
        DeclareLaunchArgument('mission', default_value='false',
                              description='Take off at A, cruise to B and land there.'),
        DeclareLaunchArgument('a_x', default_value='0.0',
                              description='Point A x in metres, for mission:=true.'),
        DeclareLaunchArgument('a_y', default_value='0.0',
                              description='Point A y in metres, for mission:=true.'),
        DeclareLaunchArgument('b_x', default_value='5.0',
                              description='Point B x in metres, for mission:=true.'),
        DeclareLaunchArgument('b_y', default_value='4.0',
                              description='Point B y in metres, for mission:=true.'),

        resource_path,
        *_rendering_env(),

        gz(['-r -v2 ', world_path], UnlessCondition(headless)),
        gz(['-s -r -v2 ', world_path], IfCondition(headless)),

        RosGzBridge(
            bridge_name='ros_gz_bridge',
            config_file=os.path.join(pkg, 'config', 'bridge.yaml'),
        ),

        Node(
            package='drone_bringup',
            executable='takeoff',
            name='takeoff',
            output='screen',
            parameters=[{'altitude': ParameterValue(
                LaunchConfiguration('altitude'), value_type=float)}],
            condition=IfCondition(LaunchConfiguration('takeoff')),
        ),

        Node(
            package='drone_bringup',
            executable='mission',
            name='mission',
            output='screen',
            parameters=[{
                **{arg: ParameterValue(LaunchConfiguration(arg), value_type=float)
                   for arg in ('a_x', 'a_y', 'b_x', 'b_y')},
                'cruise_altitude': ParameterValue(
                    LaunchConfiguration('altitude'), value_type=float),
            }],
            condition=IfCondition(LaunchConfiguration('mission')),
        ),
    ])
