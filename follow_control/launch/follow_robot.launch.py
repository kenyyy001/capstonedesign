from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_source', default_value='/dev/video0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.05'),
        DeclareLaunchArgument('camera_focal_px', default_value='600.0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('desired_distance_m', default_value='1.0'),

        # 1. Vision: deteksi marker ArUco -> /vision/target_locked + /vision/target_position
        Node(
            package='vision_lock',
            executable='aruco_lock_node',
            name='aruco_lock_node',
            output='screen',
            parameters=[{
                'camera_source': ParameterValue(LaunchConfiguration('camera_source'), value_type=str),
                'marker_size_m': ParameterValue(LaunchConfiguration('marker_size_m'), value_type=float),
                'camera_focal_px': ParameterValue(LaunchConfiguration('camera_focal_px'), value_type=float),
            }],
        ),

        # 2. Controller: data vision -> /cmd_vel
        Node(
            package='follow_control',
            executable='follow_controller_node',
            name='follow_controller_node',
            output='screen',
            parameters=[{
                'desired_distance_m': ParameterValue(LaunchConfiguration('desired_distance_m'), value_type=float),
            }],
        ),

        # 3. Bridge: /cmd_vel -> serial -> ESP32
        Node(
            package='follow_control',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            parameters=[{
                'serial_port': ParameterValue(LaunchConfiguration('serial_port'), value_type=str),
            }],
        ),
    ])
