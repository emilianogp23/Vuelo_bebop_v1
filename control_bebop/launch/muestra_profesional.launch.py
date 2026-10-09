"""
Launch file para la Muestra Profesional del Parrot Bebop 2 en ROS 2.

Ejecuta:
  1. ros2_bebop_driver (opcional si launch_driver:=true)
  2. real_drone_bridge (transformación al marco de despegue y publicación de odometría calibrada)
  3. joy_node + joystick_interface (Mando PS5 DualSense)
  4. demo_safety_manager (Garantiza modo muestra: motores activos en ralentí sin aceleración brusca)
  5. muestra_odometria_gui (Suite gráfica PyQt5 con mapa espacial, horizonte artificial, telemetría y cámara YOLO)
  6. ros2 bag record (opcional para registrar la sesión)

Uso típico:
  # Con driver corriendo en otra terminal:
  ros2 launch control_bebop muestra_profesional.launch.py

  # Lanzando el driver y conectando al dron IP por defecto (192.168.42.1):
  ros2 launch control_bebop muestra_profesional.launch.py launch_driver:=true

  # Modo presentación en proyector (sin mando físico si solo se usa la GUI):
  ros2 launch control_bebop muestra_profesional.launch.py use_joystick:=false
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

    calib_src = os.path.expanduser('~/ros2_ws/src/control_bebop/config/real_drone_calibrated.yaml')
    use_calib = LaunchConfiguration('use_calibration').perform(context).lower() == 'true'

    params = [base_cfg]
    if use_calib and os.path.exists(calib_src):
        params.append(calib_src)
        print(f'[muestra_profesional] Usando calibración: {calib_src}')

    real = {'use_real_drone': True}

    bag_dir = os.path.expanduser('~/ros2_ws/src/control_bebop/bags')
    bag_path = os.path.join(bag_dir, f'muestra_profesional_{int(time.time())}')

    nodes = [
        # 1. Driver del Bebop (condicional)
        Node(
            package='ros2_bebop_driver',
            executable='bebop_driver',
            name='bebop_driver',
            namespace='bebop',
            output='screen',
            parameters=[{'bebop_ip': LaunchConfiguration('ip').perform(context)}],
            condition=IfCondition(LaunchConfiguration('launch_driver')),
        ),

        # 2. Puente de odometría y marco de despegue
        Node(
            package='control_bebop',
            executable='real_drone_bridge',
            name='real_drone_bridge',
            parameters=params + [real],
            output='screen',
        ),

        # 3. Guardián de seguridad en modo muestra (Neutralizador de PID y gestor de arranque seguro)
        Node(
            package='control_bebop',
            executable='demo_safety_manager',
            name='demo_safety_manager',
            output='screen',
        ),

        # 4. Joy node (Lectura de mando PS5)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 20.0}],
            condition=IfCondition(LaunchConfiguration('use_joystick')),
        ),

        # 5. Interfaz de mapeo del mando PS5
        Node(
            package='control_bebop',
            executable='joystick_interface',
            name='joystick_interface',
            parameters=[joy_cfg],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_joystick')),
        ),

        # 6. Suite Gráfica Profesional (PyQt5) — Controla visualización y orientación sostenida de cámara
        Node(
            package='control_bebop',
            executable='muestra_odometria_gui',
            name='muestra_odometria_gui',
            output='screen',
        ),

        # 7. Grabación opcional de bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path,
                 '/bebop/odom', '/bebop1/pose', '/odom_local',
                 '/joy', '/joy_cmd', '/joy_commands',
                 '/demo/status', '/demo/command'],
            output='log',
            condition=IfCondition(LaunchConfiguration('record_bag')),
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_driver', default_value='false',
            description='Lanzar ros2_bebop_driver directamente'
        ),
        DeclareLaunchArgument(
            'ip', default_value='192.168.42.1',
            description='Dirección IP del Parrot Bebop 2'
        ),
        DeclareLaunchArgument(
            'use_joystick', default_value='true',
            description='Habilitar conexión con mando PS5 DualSense'
        ),
        DeclareLaunchArgument(
            'use_calibration', default_value='false',
            description='Cargar calibración de real_drone_calibrated.yaml'
        ),
        DeclareLaunchArgument(
            'record_bag', default_value='false',
            description='Registrar sesión en rosbag'
        ),
        OpaqueFunction(function=_build_nodes),
    ])

