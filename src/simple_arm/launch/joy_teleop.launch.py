# simple_arm/launch/joy_teleop.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config_package_path = get_package_share_directory('mark_ii_moveit_new')
    simple_arm_package_path = get_package_share_directory('simple_arm')
    
    xacro_file = os.path.join(get_package_share_directory("mark_ii_moveit_new"), "config", "MarkII_urdf.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="mark_ii_moveit_new")
        .robot_description(file_path=xacro_file)
        .robot_description_kinematics(file_path=os.path.join(moveit_config_package_path, "config", "kinematics.yaml"))
        .robot_description_semantic(file_path=os.path.join(moveit_config_package_path, "config", "MarkII_urdf.srdf"))
        .to_moveit_configs()
    )

    # Joy node (reads PS4 controller)
    joy_node = Node(
        package='joy',
        executable='joy_node',  # <-- Now matches the package's entry point
        name='joy',
        parameters=[{'dev': '/dev/input/js0'}],
    )

    # Teleop Twist Joy (converts joy to Twist for servo)
    teleop_config = os.path.join(moveit_config_package_path, 'config', 'ps4_teleop.config.yaml')
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        parameters=[teleop_config],
        remappings=[('/cmd_vel', '/servo_node/delta_twist_cmds')],
    )

    # MoveIt Servo node (computes joint commands from Twist)
    servo_config = os.path.join(moveit_config_package_path, 'config', 'servo.yaml')
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',  # Updated
        name='servo_node_main',  # Match log
        parameters=[
            servo_config,
            robot_description,  # URDF
            moveit_config.robot_description_semantic,  # SRDF
            moveit_config.robot_description_kinematics,  # Kinematics
        ],
        output='screen',
    )

    # Bridge node (converts servo's JointTrajectory to your action)
    bridge_node = Node(
        package='simple_arm',
        executable='trajectory_action_bridge',
        name='trajectory_action_bridge',
        output='screen',
    )
    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'frame_id': 'gripper'}],
        remappings=[
            ('/in', '/servo_node/delta_twist_cmds'),
            ('/out', '/servo_node/delta_twist_stamped_cmds'),
        ],
    )
    return LaunchDescription([
        joy_node,
        teleop_node,
        servo_node,
        bridge_node,
        twist_stamper,
    ])