import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    sdf_path = os.path.join(pkg_drone_gazebo, 'models', 'matrice_100', 'model.sdf')
    world_file = os.path.join(pkg_drone_gazebo, 'worlds', 'drone_world.sdf')
    model_root = os.path.join(pkg_drone_gazebo, 'models')

    env_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join(filter(None, [model_root, env_path]))

    # --- Gazebo Sim ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # --- Spawn del dron sobre la plataforma de aterrizaje ---
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_path, '-name', 'matrice_100',
                   '-x', '1.92', '-y', '-10', '-z', '0.7'],
        output='screen'
    )

    # --- Bridge ROS ↔ Gazebo ---
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_drone_gazebo, 'config', 'bridge.yaml')
        }],
        output='screen'
    )

    # --- Joy node (lee el control PS5) ---
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[os.path.join(pkg_drone_gazebo, 'config', 'joy_params.yaml')],
        output='screen'
    )

    # --- Teleop node (traduce Joy → cmd_vel + enable) ---
    teleop_node = Node(
        package='drone_gazebo',
        executable='drone_teleop',
        name='drone_teleop',
        output='screen'
    )

    # --- Visor de cámara ---
    image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/camera/image_raw'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        gazebo_launch,
        spawn_model,
        bridge,
        joy_node,
        teleop_node,
        image_view,
    ])