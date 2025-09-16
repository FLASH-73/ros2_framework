import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def generate_launch_description():
    
    # Declare the launch argument for switching between real and simulated hardware
    use_real_hardware_arg = DeclareLaunchArgument(
        'use_real_hardware',
        default_value='true',
        description='Set to "true" to use real hardware, "false" for simulation.'
    )

    # Get the package share directories
    simple_arm_pkg = get_package_share_directory('simple_arm')
    moveit_config_pkg = get_package_share_directory('mark_ii_moveit_new')

    # --- Robot Description (URDF) ---
    # Load the URDF from a xacro file and pass the 'use_real_hardware' argument to it
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            os.path.join(moveit_config_pkg, 'config', 'MarkII_urdf.urdf.xacro'),
            ' use_real_hardware:=', LaunchConfiguration('use_real_hardware')
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # --- Robot Description (SRDF) ---
    srdf_path = os.path.join(moveit_config_pkg, 'config', 'MarkII_urdf.srdf')
    with open(srdf_path, 'r') as srdf_file:
        robot_description_semantic_content = srdf_file.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # --- ROS2 Controllers Configuration ---
    ros2_controllers_path = os.path.join(
        moveit_config_pkg, # Corrected to use moveit_config_pkg as per your structure
        'config',
        'ros2_controllers.yaml'
    )
    controllers_path = os.path.join(
        moveit_config_pkg,
        'config',
        'controllers.yaml'
    )
    kinematics_path = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)

    # --- Joint Limits Configuration (new addition) ---
    joint_limits_path = os.path.join(moveit_config_pkg, 'config', 'joint_limits.yaml')
    with open(joint_limits_path, 'r') as f:
        joint_limits = yaml.safe_load(f)

    # --- MoveIt 2 ---
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[robot_description, robot_description_semantic, controllers_path, {'robot_description_kinematics': robot_description_kinematics},{'trajectory_execution.allowed_start_tolerance': 0.05},{'trajectory_execution.execution_velocity_scaling': 1.0}, 
        {'trajectory_execution.execution_duration_monitoring': False}, {'planning_plugin': 'ompl_interface/OMPLPlanner'},  # Switch from CHOMP
        {'request_adapters': [
            'default_planner_request_adapters/AddTimeOptimalParameterization',  # Adds timestamps/velocities
            'default_planner_request_adapters/FixWorkspaceBounds',
            'default_planner_request_adapters/FixStartStateBounds',
            'default_planner_request_adapters/FixStartStateCollision',
            'default_planner_request_adapters/FixStartStatePathConstraints'
        ]},
        {'ompl.algorithm': 'RRTConnectkConfigDefault'}, {'robot_description_planning.joint_limits': joint_limits}]
    )
    
    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')],
        parameters=[robot_description, robot_description_semantic]
    )

    # --- Foxglove Bridge (Optional) ---
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen'
    )

    # --- Controller Manager (ros2_control) ---
    # This node loads your C++ hardware interface plugin and the controllers.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # --- Spawn Controllers ---
    # The spawner nodes load the controllers into the running controller_manager.
    # We delay them to ensure the controller_manager is fully initialized.
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner_delayed = TimerAction(
        period=3.0, # Increased delay for robustness
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen',
        )]
    )

    gripper_controller_spawner_delayed = TimerAction(
        period=5.0, # Increased delay for robustness
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
        parameters=[robot_description]
    )

    # The order of nodes in the return list does not guarantee execution order.
    # Use TimerAction or event handlers for dependencies.
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