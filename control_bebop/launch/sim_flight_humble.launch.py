"""
Launch file para simulación de Bebop 2 adaptado para ROS 2 Humble + Gazebo Fortress.
Mantiene el main_flight.launch.py original intacto para la otra computadora.

Uso:
  ros2 launch control_bebop sim_flight_humble.launch.py
  # Sin joystick:
  ros2 launch control_bebop sim_flight_humble.launch.py use_joystick:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_bebop_demo = get_package_share_directory('bebop_demo')
    pkg_control = get_package_share_directory('control_bebop')
    
    try:
        pkg_bebop_gz = get_package_share_directory('bebop_gz')
    except Exception:
        pkg_bebop_gz = os.path.expanduser('~/ros2_ws/src/Vuelo_bebop_v1/bebop_ros/bebop_gz')

    # Configurar rutas de recursos para Gazebo Fortress / Ignition
    mesh_candidates = [
        os.path.join(pkg_control, 'resource', 'meshes'),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'resource', 'meshes')),
        os.path.expanduser('~/ros2_ws/src/Vuelo_bebop_v1/control_bebop/resource/meshes'),
    ]
    mesh_path = next((p for p in mesh_candidates if os.path.isdir(p)), '')
    models_path = os.path.join(pkg_bebop_gz, 'models')
    worlds_path = os.path.join(pkg_bebop_gz, 'worlds')
    world_file = os.path.join(worlds_path, 'bebop1.sdf')

    extra_paths = ':'.join([p for p in [mesh_path, models_path, worlds_path] if p and os.path.exists(p)])
    for var in ['IGN_GAZEBO_RESOURCE_PATH', 'GZ_SIM_RESOURCE_PATH']:
        current = os.environ.get(var, '')
        os.environ[var] = f"{extra_paths}:{current}" if current else extra_paths

    # Ruta guardado bag
    bag_base_dir = os.path.expanduser('~/ros2_ws/src/control_bebop/bags')
    os.makedirs(bag_base_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(bag_base_dir, f'vuelo_sim_{timestamp}')

    # Argumentos de launch
    use_joy_arg = DeclareLaunchArgument(
        'use_joystick', default_value='true',
        description='Habilitar control PS5 DualSense'
    )

    # 1. Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Bridge ROS <-> Gazebo (Nodo estándar compatible con Humble)
    bridge_config = os.path.join(pkg_bebop_demo, 'config', 'bebop1.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config}],
    )

    # 3. Pose inicial
    set_pose = Node(
        package='bebop_demo',
        executable='set_pose',
        name='set_pose',
        output='screen',
        parameters=[
            {'robot_names': '["bebop1"]'},
            {'initial_conditions': '[[0.0, 0.0, 0.0, 0.0]]'}
        ]
    )

    # 4. Flight Manager
    flight_manager = Node(
        package='control_bebop',
        executable='flight_manager',
        name='flight_manager',
        output='screen',
        parameters=[
            {'takeoff_height': 1.0},
            {'arena_half_size': 1.8},
            {'safety_margin': 0.15},
            {'auto_land_after_orbit': True},
            {'auto_land_on_home': True},
            {'deadman_timeout': 0.5},
            {'landing_rate': 0.3},
            {'position_tolerance': 0.15},
            {'touchdown_z': 0.08},
            {'frequency': 20.0},
            {'home_x': 0.0},
            {'home_y': 0.0},
        ],
    )

    # 5a. Joy node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'device_id': 0},
            {'deadzone': 0.05},
            {'autorepeat_rate': 20.0},
        ],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    # 5b. Interfaz del joystick PS5
    joystick_interface = Node(
        package='control_bebop',
        executable='joystick_interface',
        name='joystick_interface',
        output='screen',
        parameters=[
            os.path.join(pkg_control, 'config', 'ps5_dualsense.yaml'),
        ],
        condition=IfCondition(LaunchConfiguration('use_joystick')),
    )

    # 6. Controlador PID
    controller = Node(
        package='control_bebop',
        executable='control_PID',
        name='control_PID',
        output='screen',
        parameters=[
            {'takeoff_height': 1.0},
            {'arena_limit': 1.8},
            {'max_vel_xy': 0.5},
            {'max_vel_z': 1.0},
            {'max_vel_yaw': 0.5},
            {'cmd_vel_topic': '/bebop1/cmd_vel'},
        ],
    )

    # 7. Planificador de trayectoria
    trajectory = Node(
        package='control_bebop',
        executable='trajectory_planner',
        name='trajectory_planner',
        output='screen',
        parameters=[
            {'target_x': 1.0},
            {'target_y': 0.0},
            {'circle_radius': 0.15},
            {'circle_height': 1.0},
            {'circle_speed': 0.15},
            {'arena_half_size': 1.8},
            {'safety_margin': 0.15},
            {'lookat_x': 1.0},
            {'lookat_y': 0.0},
            {'lookat_z': 1.025},
        ]
    )

    # 8. Visor de odometría
    gui_odometria = Node(
        package='control_bebop',
        executable='gui_odometria',
        name='gui_odometria',
        output='screen',
        parameters=[{'frequency': 20.0}],
    )

    # 9. Nodo de video
    video_node = Node(
        package='control_bebop',
        executable='nodo_video',
        name='nodo_video',
        output='screen',
        parameters=[{'camera_fps': 15.0}],
    )

    # Grabación bag
    bag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path, '/bebop1/odom', '/goal', '/flight_state', '/bebop1/cmd_vel'],
        output='log'
    )

    return LaunchDescription([
        use_joy_arg,
        gz_sim,
        ros_gz_bridge,
        set_pose,
        flight_manager,
        joy_node,
        joystick_interface,
        controller,
        trajectory,
        gui_odometria,
        video_node,
        bag_record,
    ])

