import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # Declare the launch argument for switching between real and simulated hardware
    use_real_hardware_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='true',
        description='Set to "true" to use real hardware, "false" for simulation.'
    )

    # Get the package share directories
    moveit_config_pkg = get_package_share_directory('mark_ii_moveit_new')

    # Use MoveItConfigsBuilder for standard config loading
    moveit_config = (
        MoveItConfigsBuilder("MarkII_urdf", package_name="mark_ii_moveit_new")
        .robot_description(
            file_path=os.path.join(moveit_config_pkg, "config", "MarkII_urdf.urdf.xacro"),
            mappings={"use_real_hardware": LaunchConfiguration('use_real_hardware')}
        )
        .robot_description_semantic(file_path=os.path.join(moveit_config_pkg, "config", "MarkII_urdf.srdf"))
        .trajectory_execution(file_path=os.path.join(moveit_config_pkg, "config", "controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Flatten controllers.yaml for top-level params
    controllers_path = os.path.join(moveit_config_pkg, "config", "controllers.yaml")
    with open(controllers_path, 'r') as f:
        full_controllers = yaml.safe_load(f)
        controllers_params = full_controllers['move_group']['ros__parameters']

    # TOPP params (nested under 'ompl' for adapter)
    topp_params = {
        'ompl': {
            'resample_dt': 0.01,       # Force 10ms steps for increasing timestamps
            'path_tolerance': 0.1,     # Velocity tolerance
            'min_angle_change': 0.001, # Min change to trigger resampling
        }
    }

    # Merge all params into one dict (ensures overrides like 'ompl.resample_dt' apply)
    all_params = moveit_config.to_dict()
    all_params.update(controllers_params)
    all_params.update(topp_params)
    all_params.update({
        'trajectory_execution.allowed_start_tolerance': 0.05,
        'moveit_manage_controllers': False,
    })

    # --- ROS2 Controllers Configuration ---
    ros2_controllers_path = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    # --- Controller Manager ---
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output='screen',
    )

    # --- Spawn Controllers (delayed) ---
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner_delayed = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    gripper_controller_spawner_delayed = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )
    
    # --- Robot State Publisher ---
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[moveit_config.robot_description]
    )

    # --- MoveGroup Node (with merged params) ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[all_params],  # Single merged dict for proper overrides
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ]
    )

    # --- Foxglove Bridge ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    return LaunchDescription([
        use_real_hardware_arg,
        controller_manager,
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
        foxglove_bridge,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner_delayed,
        gripper_controller_spawner_delayed,
    ])