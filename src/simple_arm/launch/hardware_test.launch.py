import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
    """
    This function is executed at launch time and resolves all substitutions.
    """
    use_real_hardware = LaunchConfiguration("use_real_hardware")

    # Generate the MoveIt configuration parameters as a dictionary
    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="mark_ii_moveit_new")
        .robot_description(
            mappings={"use_real_hardware": use_real_hardware.perform(context)}
        )
        .to_dict()  # CRITICAL FIX: This returns a standard dictionary
    )

    # --- Nodes ---

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mark_ii_moveit_new"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[moveit_config],
    )

    # ros2_control node
    ros2_controllers_path = PathJoinSubstitution(
        [FindPackageShare("mark_ii_moveit_new"), "config", "ros2_controllers.yaml"]
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config, ros2_controllers_path], # Now passing a dict and a path
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config],
    )

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )
    
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # --- Event Handlers for Sequential Startup ---
    
    delay_arm_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    delay_gripper_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return [
        rviz_node,
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_spawner,
        delay_gripper_controller_spawner,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_real_hardware",
            default_value="true",
            description="Flag to use real hardware interface. If false, a fake system is used.",
        )
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )