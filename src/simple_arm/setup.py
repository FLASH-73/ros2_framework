from setuptools import setup
import os
from glob import glob

package_name = 'simple_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=['simple_arm'],  # Your Python code directory
    data_files=[
        # Marker and package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),

        # URDF files (now from description dir)
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*'))),

        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 control for the Mark II Arm (Python)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_driver = simple_arm.arm_driver_node:main',  # New: For your driver node
            'trajectory_action_bridge = simple_arm.trajectory_action_bridge:main',  # New: For the action bridge
        ],
    },
)