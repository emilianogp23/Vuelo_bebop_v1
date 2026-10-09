"""
Launch para vuelo con el Bebop 2 REAL.

Cadena de comandos:
    joy_node → /joy → joystick_interface → /joy_commands, /joy_cmd, /deadman_active
    flight_manager → /bridge/takeoff, /bridge/land, /bridge/emergency, /bridge/flattrim
    control_PID   → /bridge/cmd_vel_raw
    real_drone_bridge → /bebop/takeoff, /bebop/land, /bebop/reset, /bebop/flattrim, /bebop/cmd_vel
    real_drone_bridge ← /bebop/odom   → /bebop1/pose (marco de despegue), /odom_local

Uso:
    # Driver corriendo en otra terminal (recomendado):
    ros2 launch ros2_bebop_driver bebop_node_launch.xml ip:=192.168.42.1
    ros2 launch control_bebop real_flight.launch.py

    # O lanzar el driver aquí mismo:
    ros2 launch control_bebop real_flight.launch.py launch_driver:=true

    # Usar el YAML de calibración generado por analisis_deriva:
    ros2 launch control_bebop real_flight.launch.py use_calibration:=true
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context):
    pkg_dir = get_package_share_directory('control_bebop')
    base_cfg = os.path.join(pkg_dir, 'config', 'real_drone.yaml')
    joy_cfg = os.path.join(pkg_dir, 'config', 'ps5_dualsense.yaml')

    # Calibración: se busca en el src (donde la escribe analisis_deriva)
    calib_src = os.path.expanduser(
        '~/ros2_ws/src/control_bebop/config/real_drone_calibrated.yaml')
    use_calib = LaunchConfiguration('use_calibration').perform(context).lower() == 'true'

    params = [base_cfg]
    if use_calib and os.path.exists(calib_src):
        params.append(calib_src)
        print(f'[real_flight] Usando calibración: {calib_src}')
    elif use_calib:
        print('[real_flight] use_calibration:=true pero no existe el YAML, se ignora.')

    # Forzado explícito: NUNCA depender de que el YAML lo traiga
    real = {'use_real_drone': True}

    bag_dir = os.path.expanduser('~/ros2_ws/src/control_bebop/bags')
    bag_path = os.path.join(bag_dir, f'vuelo_real_{int(time.time())}')

    return [
        Node(
            package='ros2_bebop_driver',
            executable='bebop_driver',
            name='bebop_driver',
            namespace='bebop',
            output='screen',
            parameters=[{'bebop_ip': LaunchConfiguration('ip').perform(context)}],
            condition=IfCondition(LaunchConfiguration('launch_driver')),
        ),
        Node(
            package='control_bebop',
            executable='real_drone_bridge',
            name='real_drone_bridge',
            parameters=params + [real],
            output='screen',
        ),
        Node(
            package='control_bebop',
            executable='flight_manager',
            name='flight_manager',
            parameters=params + [real, {
                'takeoff_height': 1.0,
                'arena_half_size': 2.0,
                'max_altitude': 2.0,
            }],
            output='screen',
            remappings=[
                ('cmd_takeoff', '/bridge/takeoff'),
                ('cmd_land', '/bridge/land'),
            ],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
        ),
        Node(
            package='control_bebop',
            executable='joystick_interface',
            name='joystick_interface',
            parameters=[joy_cfg],
            output='screen',
        ),
        Node(
            package='control_bebop',
            executable='control_PID',
            name='control_PID',
            parameters=params + [real, {
                # El tópico es absoluto dentro del nodo: se fija por parámetro
                'cmd_vel_topic': '/bridge/cmd_vel_raw',
                'arena_limit': 2.0,
            }],
            output='screen',
        ),
        Node(
            package='control_bebop',
            executable='trajectory_planner',
            name='trajectory_controller',
            parameters=params,
            output='screen',
        ),
        Node(
            package='control_bebop',
            executable='nodo_video',
            name='nodo_video',
            parameters=params,
            output='screen',
        ),
        Node(
            package='control_bebop',
            executable='gui_odometria',
            name='gui_odometria',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_gui')),
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path,
                 '/bebop/odom', '/bebop/cmd_vel', '/bridge/cmd_vel_raw',
                 '/bebop1/pose', '/odom_local', '/joy', '/joy_cmd',
                 '/flight_state', '/goal'],
            output='log',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_driver', default_value='false',
                              description='Lanzar ros2_bebop_driver desde aquí'),
        DeclareLaunchArgument('launch_gui', default_value='true',
                              description='Abrir ventana interactiva de odometría'),
        DeclareLaunchArgument('ip', default_value='192.168.42.1',
                              description='IP del Bebop 2'),
        DeclareLaunchArgument('use_calibration', default_value='false',
                              description='Cargar config/real_drone_calibrated.yaml'),
        OpaqueFunction(function=_build_nodes),
    ])
