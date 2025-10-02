import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
import xacro

def generate_launch_description():
    """
    Generates the launch description for starting the Mark II arm with MoveIt 2.

    This launch file performs the following steps:
    1.  Processes the robot's URDF/XACRO file to generate the robot_description.
    2.  Loads the MoveIt 2 configuration based on that robot_description.
    3.  Starts the Robot State Publisher.
    4.  Starts the ros2_control node, which loads your Python hardware interface.
    5.  Spawns the necessary controllers (for the arm and gripper).
    6.  Starts the MoveIt 2 move_group node for motion planning.
    7.  Starts RViz with the MoveIt 2 visualization configuration.
    """

    # --- Step 1: Get Package Paths ---
    # This makes the launch file portable by finding files relative to the package.
    simple_arm_package_path = get_package_share_directory("simple_arm")
    # NOTE: You may need to update 'mark_ii_moveit_new' to your actual MoveIt config package name
    moveit_config_package_path = get_package_share_directory("mark_ii_moveit_new")

    # --- Step 2: Process URDF and Load MoveIt Config ---
    # This is the crucial step to ensure the <ros2_control> tags from your xacro
    # are included in the robot_description that all nodes receive.
    xacro_file = os.path.join(get_package_share_directory("mark_ii_moveit_new"), "config", "MarkII_urdf.urdf.xacro")
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # The MoveItConfigsBuilder uses the robot_description to generate the rest of
    # the MoveIt configuration.
    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="mark_ii_moveit_new")
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path=os.path.join(moveit_config_package_path, "config", "MarkII_urdf.srdf"))
        .trajectory_execution(file_path=os.path.join(moveit_config_package_path, "config", "moveit_controllers.yaml"))
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
        )
        .to_moveit_configs()
    )

    # --- Step 3: Define Nodes ---

    # Robot State Publisher: Publishes the robot's state (joint positions) to TF2.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ROS2 Control Node: The heart of the hardware interface.
    # It loads the robot_description, finds the <ros2_control> tag, and loads the
    # plugin specified there (your Python ArmHardware class).
    arm_driver = Node(
        package='simple_arm',
        executable='arm_driver',
        name='arm_driver',
        output='screen',
    )

    # MoveGroup Node: The main MoveIt 2 planning interface.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"}],
    )

    # RViz Node: For visualization.
    rviz_config = os.path.join(moveit_config_package_path, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # --- Step 4: Define Controller Spawners ---
    # These nodes wait for the controller_manager to be active and then load
    # the controllers specified in your ros2_controllers.yaml file.

   

    # --- Step 5: Assemble and Return Launch Description ---
    return LaunchDescription(
        [
            robot_state_publisher,
            arm_driver,
            move_group_node,
            rviz_node,
           
        ]
    )
