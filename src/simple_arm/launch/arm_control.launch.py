import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    servo_params_dict = {
    'move_group_name': 'arm',
    'planning_group': 'arm',
    'move_group': 'arm',
    'planning_frame': 'base_link',
    'ee_frame_name': 'gripper_ee',
    'robot_link_command_frame': 'gripper_ee',
    'incoming_command_timeout': 0.1,
    'command_in_type': 'unitless',
    'command_out_topic': '/joint_trajectory_controller/joint_trajectory',
    'command_out_type': 'trajectory_msgs/JointTrajectory',
    'joint_topic': '/joint_states',
    'linear_scale': 10.0,  # Further increase for visibility
    'angular_scale': 10.0,
    'joint_scale': 1.0,
    'singularity_threshold': 0.5,  # Relax further
    'hard_stop_singularity_threshold': 0.7,
    'collision_velocity_scale': 0.2,
    'use_gazebo': False,
    'publish_period': 0.005,  # 200Hz for faster response
    'joint_limit_margin': 0.1,  # Relax joint limits
    'low_pass_filter_coeff': 1.0,  # Minimal filtering
    'publish_joint_positions': True,
    'publish_joint_velocities': True,
    'check_collisions': False,
    'collision_check_rate': 100,
    'collision_check_type': 'STOP_ON_COLLISION',
    'allowed_planning_time': 0.5,
    'print_servo_debug': True  # Custom param for debug logging
}
    #print("Servo parameters:", servo_params)
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
            'resample_dt': 0.05,       # Force 10ms steps for increasing timestamps
            'path_tolerance': 0.05,     # Velocity tolerance
            'min_angle_change': 0.005, # Min change to trigger resampling
        }
    }

    # Merge all params into one dict (ensures overrides like 'ompl.resample_dt' apply)
    all_params = moveit_config.to_dict()
    all_params.update(controllers_params)
    all_params.update(topp_params)
    all_params.update({
        'planning_pipelines': ['ompl'],
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'default_planning_pipeline': 'ompl',
        'use_sim_time': False,
        'allow_integration_in_goal_trajectories': True,
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
        'trajectory_execution.allowed_start_tolerance': 0.2,  # Relax tolerance
        'moveit_manage_controllers': False,
        'move_group/planning_plugin': 'ompl_interface/OMPLPlanner',
        'move_group/default_planning_pipeline': 'ompl',  # Additional override
        'move_group/available_plugins': ['ompl_interface/OMPLPlanner']  # Restrict to OMPL
    })
    # --- ROS2 Controllers Configuration ---
    ros2_controllers_path = os.path.join(moveit_config_pkg, "config", "ros2_controllers.yaml")

    servo_params_path = os.path.join(moveit_config_pkg, "config", "servo.yaml")
    

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
    #print("Servo params path:", servo_params_path)  # Add before servo_node
    """
    with open(servo_params_path, 'r') as f:
        servo_params_dict = yaml.safe_load(f)['servo_node']['ros__parameters']  # Flatten to dict

    servo_params_dict.update({
        'move_group_name': 'arm',
        'planning_group': 'arm'
    })
    """
    
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params_dict,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_intra_process_comms': True}
        ],
        #extra_arguments=[{'use_intra_process_comms': True}],
    )
    """
    servo_container = ComposableNodeContainer(
        name='servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
    )
    servo_composable = LoadComposableNodes(
        target_container='servo_container',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoNode',  # Use plugin class
                name='servo_node',
                parameters=[
                    servo_params_dict,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    {'use_intra_process_comms': True},  # Enables intra-process
                    {'move_group_name': 'arm'},  # Force override
                ],
            )
        ],
    )"""
    # Joy node for PS4 controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',  # Adjust if your PS4 is js1, etc. (check dmesg or ls /dev/input)
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # Joy to Servo bridge
    joy_to_servo_node = Node(
        package='simple_arm',
        executable='joy_to_servo.py',
        name='joy_to_servo',
        output='screen'
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
    marker_bridge = Node(
        package='simple_arm',
        executable='marker_to_servo',
        output='screen'
    )

    return LaunchDescription([
        use_real_hardware_arg,
        controller_manager,
        robot_state_publisher_node,
        move_group_node,
        joy_node,
        joy_to_servo_node,
        rviz_node,
        foxglove_bridge,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner_delayed,
        gripper_controller_spawner_delayed,
        servo_node,
        marker_bridge,
    ])