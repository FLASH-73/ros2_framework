import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    """
    Generates a streamlined ROS2 launch description for basic MoveIt2 functionality.

    This launch file starts the essential nodes for planning and executing trajectories
    for the Mark II robotic arm and its gripper. It excludes advanced features like
    moveit_servo, joystick teleoperation, and interactive marker control.
    """

    # --- Step 1: Process URDF with xacro ---
    # The 'no ros2_control tag' error is fixed by explicitly processing the URDF
    # with xacro first. This ensures the <ros2_control> tags from your .xacro
    # files are included in the robot_description that all nodes receive.
    urdf_xacro_path = os.path.join(
        get_package_share_directory("simple_arm"),
        "description",
        "MarkII_urdf.urdf"  # This file must include the .ros2_control.xacro
    )
    robot_description_content = xacro.process_file(urdf_xacro_path).toxml()
    robot_description = {"robot_description": robot_description_content}

    # --- Step 2: Build MoveIt Configuration ---
    # This builder now uses the pre-processed robot_description.
    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="mark_ii_moveit_new")
        .robot_description_semantic()
        .robot_description_kinematics()
        .planning_pipelines(pipelines=["ompl"])  # Trimmed to only OMPL for simplicity
        .trajectory_execution()
        .to_moveit_configs()
    )

    # --- Step 3: Launch Essential Nodes ---

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],  # Use the processed description
    )

    # ROS2 Control Node (Controller Manager)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("mark_ii_moveit_new"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager", # Explicitly naming the node
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Controller Spawners (delayed to ensure controller_manager is ready)
    controller_spawners = []
    controllers_to_spawn = ["joint_state_broadcaster", "arm_controller", "gripper_controller"]
    for i, controller in enumerate(controllers_to_spawn):
        controller_spawners.append(
            TimerAction(
                period=float(i * 3.0 + 3.0), # Staggered start
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[controller, "-c", "/controller_manager"],
                        output="screen",
                    )
                ],
            )
        )

    # MoveGroup Node
    # Combine the MoveIt configuration with the processed robot description
    move_group_params = moveit_config.to_dict()
    move_group_params.update(robot_description)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params],
    )

    # RViz for Visualization (optional, but kept for basic trajectory visualization)
    rviz_config = os.path.join(
        get_package_share_directory("mark_ii_moveit_new"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,  # Use the processed description
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Assemble and return the full launch description
    nodes_to_launch = [
        robot_state_publisher,
        ros2_control_node,
        move_group_node,
        rviz_node,
    ] + controller_spawners

    return LaunchDescription(nodes_to_launch)