"""AWS small warehouse with N TurtleBot3 Waffles, one Nav2 stack each.

Follows nav2_bringup's cloned_multi_tb3_simulation_launch.py, with two
differences: the world is plain SDF (no xacro step), and every robot gets
its own params file whose AMCL initial pose is its spawn pose, so robots
are localized at start without anyone publishing /initialpose. Each robot
also gets a simulated battery (scripts/battery_sim.py) on battery_state,
and its true pose from Gazebo on ground_truth (nav_msgs/Odometry, map
frame) for evaluation. The robot model is urdf/waffle.sdf.xacro.

    ros2 launch warehouse_sim warehouse_multi.launch.py n_robots:=3
    ros2 launch warehouse_sim warehouse_multi.launch.py gui:=True
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, GroupAction, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def robot_params(base_file, robot):
    """Copy of base_file with AMCL starting at the robot's spawn pose."""
    with open(base_file) as f:
        params = yaml.safe_load(f)
    amcl = params['amcl']['ros__parameters']
    amcl['set_initial_pose'] = True
    amcl['initial_pose'] = {'x': robot['x'], 'y': robot['y'], 'z': 0.0, 'yaw': robot['yaw']}
    out = tempfile.NamedTemporaryFile(
        'w', prefix=f"nav2_{robot['name']}_", suffix='.yaml', delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def launch_robots(context):
    bringup = get_package_share_directory('nav2_bringup')
    robot_sdf = os.path.join(get_package_share_directory('warehouse_sim'), 'urdf', 'waffle.sdf.xacro')
    with open(LaunchConfiguration('robots_file').perform(context)) as f:
        robots = yaml.safe_load(f)['robots']
    robots = robots[:int(LaunchConfiguration('n_robots').perform(context))]
    base_params = LaunchConfiguration('params_file').perform(context)

    actions = []
    for r in robots:
        actions.append(GroupAction([
            LogInfo(msg=f"Launching {r['name']} at ({r['x']}, {r['y']}, yaw {r['yaw']})"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup, 'launch', 'tb3_simulation_launch.py')),
                launch_arguments={
                    'namespace': r['name'],
                    'use_namespace': 'True',
                    'map': LaunchConfiguration('map'),
                    'use_sim_time': 'True',
                    'params_file': robot_params(base_params, r),
                    'autostart': 'True',
                    'use_rviz': 'False',
                    'use_simulator': 'False',
                    'headless': 'False',
                    'use_robot_state_pub': 'True',
                    'x_pose': str(r['x']),
                    'y_pose': str(r['y']),
                    'z_pose': '0.01',
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': str(r['yaw']),
                    'robot_name': r['name'],
                    'robot_sdf': robot_sdf,
                }.items()),
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name='ground_truth_bridge', namespace=r['name'], output='screen',
                 arguments=[f"/{r['name']}/ground_truth@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
                 remappings=[(f"/{r['name']}/ground_truth", 'ground_truth')],
                 parameters=[{'use_sim_time': True}]),
            Node(package='warehouse_sim', executable='battery_sim.py',
                 namespace=r['name'], output='screen',
                 parameters=[{'use_sim_time': True,
                              'range_m': float(LaunchConfiguration('battery_range_m').perform(context))}]),
        ]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('warehouse_sim')
    bringup = get_package_share_directory('nav2_bringup')
    tb3 = get_package_share_directory('nav2_minimal_tb3_sim')

    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='3'),
        DeclareLaunchArgument(
            'robots_file', default_value=os.path.join(share, 'config', 'robots.yaml')),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(share, 'worlds', 'warehouse.sdf')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(share, 'maps', 'warehouse_sim.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('battery_range_m', default_value='200.0',
                              description='Distance a full battery lasts'),

        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(tb3)),

        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', LaunchConfiguration('world')],
                       output='screen'),
        ExecuteProcess(cmd=['gz', 'sim', '-g'], output='screen',
                       condition=IfCondition(LaunchConfiguration('gui'))),
        OpaqueFunction(function=launch_robots),
    ])
