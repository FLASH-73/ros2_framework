import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial_d405 = DeclareLaunchArgument('serial_d405', default_value='1234567890')  # Replace with your D405 serial (rs-enumerate-devices)
    serial_d435 = DeclareLaunchArgument('serial_d435', default_value='0987654321')  # D435 serial

    d405_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d405_gripper_cam',
        namespace='d405',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_d405'),
            'rgb_camera.profile': '640x480x30',  # Adjust resolution/FPS
            'depth_module.profile': '640x480x30',
            'enable_infra1': True,  # For depth/IR
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,  # Align depth to RGB
        }],
        output='screen'
    )

    d435_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435_stationary_cam',
        namespace='d435',
        parameters=[{
            'serial_no': LaunchConfiguration('serial_d435'),
            'rgb_camera.profile': '1280x720x30',  # Higher for overview
            'depth_module.profile': '1280x720x30',
            'enable_infra1': True,
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
        }],
        output='screen'
    )

    return launch.LaunchDescription([
        serial_d405,
        serial_d435,
        d405_node,
        d435_node
    ])