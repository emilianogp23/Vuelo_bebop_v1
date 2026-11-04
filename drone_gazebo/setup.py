from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_gazebo'


def get_model_data_files():
    """Recursively collect every file under models/ so all models are installed."""
    data_files = []
    models_dir = 'models'
    for dirpath, _, filenames in os.walk(models_dir):
        if filenames:
            # Build install destination mirroring source tree:
            #   models/DronePlatformX1/meshes  →  share/drone_gazebo/models/DronePlatformX1/meshes
            install_dir = os.path.join('share', package_name, dirpath)
            files = [os.path.join(dirpath, f) for f in filenames]
            data_files.append((install_dir, files))
    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ] + get_model_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emiliano',
    maintainer_email='emiliano@todo.todo',
    description='Simulacion DJI Matrice 100 con Gazebo y ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_teleop = drone_gazebo.drone_teleop:main',
            'pose_relay = drone_gazebo.pose_relay:main',
        ],
    },
)