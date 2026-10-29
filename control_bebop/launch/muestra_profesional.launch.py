"""
Launch file para la Muestra Profesional del Parrot Bebop 2 en ROS 2.

Ejecuta:
  1. ros2_bebop_driver (opcional si launch_driver:=true)
  2. real_drone_bridge (transformación al marco de despegue y publicación de odometría calibrada)
  3. joy_node + joystick_interface (Mando PS5 DualSense)
  4. demo_safety_manager (L2+× despega y el dron queda en hover nativo a ~1 m; mismos botones que el vuelo real)
  5. muestra_odometria_gui (Suite gráfica PyQt5 con mapa 2D/3D, horizonte artificial, telemetría y cámara YOLO)
  6. ros2 bag record (opcional para registrar la sesión)

Mando PS5 (L2 es el deadman: sin él se ignora todo movimiento):
  L2 + ×          despegar (sube a ~1 m y queda en hover)
  L2 sostenido +  stick izquierdo = avance/lateral, cruceta = altura/giro (manual:=true)
  soltar L2       hover nativo
  ○               aterrizar          △  aterrizaje inmediato
  L1 + R1 + △     corte total de motores (el dron cae)
  Share           FlatTrim (en tierra)   Options  reset desde emergencia
  El stick derecho no se usa. La inclinación de la cámara se maneja desde la GUI.

Uso típico:
  # Con driver corriendo en otra terminal:
  ros2 launch control_bebop muestra_profesional.launch.py

  # Lanzando el driver y conectando al dron IP por defecto (192.168.42.1):
  ros2 launch control_bebop muestra_profesional.launch.py launch_driver:=true

  # Modo presentación en proyector (sin mando físico si solo se usa la GUI):
  ros2 launch control_bebop muestra_profesional.launch.py use_joystick:=false

  # Solo hover (sin control manual) o con otra velocidad máxima:
  ros2 launch control_bebop muestra_profesional.launch.py manual:=false
  ros2 launch control_bebop muestra_profesional.launch.py manual_speed:=0.15

  # Vista inicial de la GUI: map | camera | split, y mapa 2d | 3d:
  ros2 launch control_bebop muestra_profesional.launch.py gui_view:=split gui_map:=3d
"""

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from control_bebop.paths import config_file, data_dir


def _build_nodes(context):
    pkg_dir = get_package_share_directory('control_bebop')
    base_cfg = os.path.join(pkg_dir, 'config', 'real_drone.yaml')
    joy_cfg = os.path.join(pkg_dir, 'config', 'ps5_dualsense.yaml')

    calib_src = config_file('real_drone_calibrated.yaml')
    use_calib = LaunchConfiguration('use_calibration').perform(context).lower() == 'true'

    params = [base_cfg]
    if use_calib and calib_src:
        params.append(calib_src)
        print(f'[muestra_profesional] Usando calibración: {calib_src}')

    real = {'use_real_drone': True}

    manual = LaunchConfiguration('manual').perform(context).lower() == 'true'
    manual_speed = float(LaunchConfiguration('manual_speed').perform(context))
    # Tope absoluto: el bridge satura en 0.4 (max_cmd_xy); más no tiene efecto
    manual_speed = max(0.0, min(manual_speed, 0.4))
    print(f'[muestra_profesional] Manual {"ON" if manual else "OFF"} (≤ {manual_speed:.2f})')

    # control_bebop/bags, relativo al proyecto (ver control_bebop/paths.py)
    bag_dir = data_dir('bags')
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

        # 3. Despegue, hover nativo y aterrizaje por el bridge
        Node(
            package='control_bebop',
            executable='demo_safety_manager',
            name='demo_safety_manager',
            parameters=[{
                'allow_manual_nudge': manual,
                'max_nudge_speed': manual_speed,
            }],
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

        # 6. GUI (PyQt5): mapa 2D/3D, cámara con YOLO y orientación sostenida de cámara.
        #    base_cfg aporta las dimensiones del corral (arena_x_max, max_z, safe_zone_radius).
        Node(
            package='control_bebop',
            executable='muestra_odometria_gui',
            name='muestra_odometria_gui',
            parameters=[base_cfg, {
                'gui_view': LaunchConfiguration('gui_view').perform(context),
                'gui_map': LaunchConfiguration('gui_map').perform(context),
            }],
            output='screen',
        ),

        # 7. Grabación opcional de bag
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', bag_path,
                 '/bebop/odom', '/bebop1/pose', '/odom_local',
                 '/joy', '/joy_cmd', '/joy_commands',
                 '/demo/status', '/demo/command', '/bridge/landing',
                 '/bebop/altitude', '/bebop/cmd_vel'],
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
        DeclareLaunchArgument(
            'manual', default_value='true',
            description='Control manual con L2 sostenido (false: solo hover)'
        ),
        DeclareLaunchArgument(
            'manual_speed', default_value='0.25',
            description='Comando máximo del control manual, 0.0-0.4 (el bridge satura en 0.4)'
        ),
        DeclareLaunchArgument(
            'gui_view', default_value='map',
            description='Vista inicial de la GUI: map | camera | split'
        ),
        DeclareLaunchArgument(
            'gui_map', default_value='2d',
            description='Mapa inicial de la GUI: 2d | 3d'
        ),
        OpaqueFunction(function=_build_nodes),
    ])
