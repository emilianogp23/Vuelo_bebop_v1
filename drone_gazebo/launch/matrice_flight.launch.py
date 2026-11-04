"""
Matrice 100 en drone_world con el sistema de vuelo de control_bebop.

Mismos nodos y controles que la simulación del Bebop (flight_manager,
trajectory_planner, control_PID, geocerca en sim_odometry, mando PS5, video),
escalados al Matrice en config/matrice.yaml: órbita de r 20 m a 6 m de altura
alrededor de la estación de bomberos. sim.launch.py sigue siendo la
teleoperación directa con drone_teleop.

Funciona en Jazzy + Harmonic y en Humble + Fortress (laptop): en Humble usa una
copia del mundo y del modelo con los nombres de sistemas de Fortress.

Uso:
    ros2 launch drone_gazebo matrice_flight.launch.py
    ros2 launch drone_gazebo matrice_flight.launch.py headless:=true record_bag:=false
"""

import getpass
import os
import re
import tempfile
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from control_bebop.paths import data_dir
from control_bebop.sim_common import SIM_BAG_TOPICS, gz_args, is_true


def _fortress_copy(src):
    """Copia del SDF para Fortress: sistemas ignition-gazebo-*, SDF 1.9, mallas por model://."""
    with open(src, encoding='utf-8') as f:
        sdf = f.read()
    sdf = re.sub(r'filename=([\'"])gz-sim-', r'filename=\1ignition-gazebo-', sdf)
    sdf = re.sub(r'<sdf version=([\'"])1\.1\d', r'<sdf version=\g<1>1.9', sdf)
    # La copia vive en /tmp: las rutas relativas del Matrice ya no resolverían
    sdf = sdf.replace('<uri>meshes/', '<uri>model://matrice_100/meshes/')
    out_dir = os.path.join(tempfile.gettempdir(), f'drone_gazebo_{getpass.getuser()}')
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, os.path.basename(src))
    with open(out, 'w', encoding='utf-8') as f:
        f.write(sdf)
    return out


def _build(context):
    pkg = get_package_share_directory('drone_gazebo')
    bebop_cfg = os.path.join(get_package_share_directory('control_bebop'), 'config')
    cfg = [os.path.join(bebop_cfg, 'real_drone.yaml'),
           os.path.join(bebop_cfg, 'sim.yaml'),
           os.path.join(pkg, 'config', 'matrice.yaml')]
    use_joy = IfCondition(LaunchConfiguration('use_joystick'))
    model_root = os.path.join(pkg, 'models')
    world = os.path.join(pkg, 'worlds', 'drone_world.sdf')
    model = os.path.join(model_root, 'matrice_100', 'model.sdf')
    if os.environ.get('ROS_DISTRO') == 'humble':
        world, model = _fortress_copy(world), _fortress_copy(model)

    actions = [
        SetEnvironmentVariable(var, os.pathsep.join(
            filter(None, [model_root, os.environ.get(var, '')])))
        for var in ('GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH')
    ] + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args(world, context)}.items()),
        # Centrado en la placa de colisión de la plataforma (x 2 - 0.08); pose_relay usa este origen
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', model,
                        '-name', 'matrice_100', '-x', '1.92', '-y', '-10', '-z', '0.7']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'bridge_control.yaml')}]),
        Node(package='drone_gazebo', executable='pose_relay', name='pose_relay', output='screen'),

        Node(package='control_bebop', executable='flight_manager',
             name='flight_manager', output='screen', parameters=cfg,
             remappings=[('cmd_takeoff', '/bridge/takeoff'), ('cmd_land', '/bridge/land')]),
        Node(package='joy', executable='joy_node', name='joy_node',
             output='screen', condition=use_joy,
             parameters=[os.path.join(pkg, 'config', 'joy_params.yaml')]),
        Node(package='control_bebop', executable='joystick_interface',
             name='joystick_interface', output='screen', condition=use_joy,
             parameters=[os.path.join(bebop_cfg, 'ps5_dualsense.yaml'),
                         {'max_linear_speed': 2.0, 'max_vertical_speed': 1.0}]),
        Node(package='control_bebop', executable='control_PID',
             name='control_PID', output='screen', parameters=cfg),
        Node(package='control_bebop', executable='trajectory_planner',
             name='trajectory_planner', output='screen', parameters=cfg),
        Node(package='control_bebop', executable='nodo_video',
             name='nodo_video', output='screen', parameters=cfg),
        # Geocerca y watchdog de /bridge/cmd_vel_raw → /bebop1/cmd_vel; publica /bebop1/pose
        Node(package='control_bebop', executable='sim_odometry',
             name='sim_odometry', output='screen', parameters=cfg),
        Node(package='control_bebop', executable='odom_viewer',
             name='odom_viewer', output='log', parameters=[{'log_rate': 2.0}]),
    ]
    if is_true(context, 'launch_gui'):
        actions.append(Node(package='control_bebop', executable='gui_odometria',
                            name='gui_odometria', output='log', parameters=cfg))
    if is_true(context, 'launch_camera_view'):
        actions.append(Node(package='rqt_image_view', executable='rqt_image_view',
                            arguments=['/camera/image_raw'], output='log'))
    if is_true(context, 'record_bag'):
        bag = os.path.join(data_dir('bags'),
                           f'vuelo_matrice_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        actions += [
            LogInfo(msg=f'Grabando bag en {bag}'),
            ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', bag]
                           + SIM_BAG_TOPICS, output='log'),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_joystick', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo sin ventana'),
        DeclareLaunchArgument('record_bag', default_value='true',
                              description='Grabar un bag en control_bebop/bags'),
        DeclareLaunchArgument('launch_gui', default_value='true',
                              description='Ventana de odometría'),
        DeclareLaunchArgument('launch_camera_view', default_value='true',
                              description='rqt_image_view con la cámara del Matrice'),
        OpaqueFunction(function=_build),
    ])
