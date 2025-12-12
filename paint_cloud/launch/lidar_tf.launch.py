from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Existing transform: world -> link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=['0', '0', '2.0', '0', '1.57', '0', 'world', 'link']
        ),
        
        # New transform: base_link -> base_link1 (180 deg around Z)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_rot_tf',
            # Arguments: x y z yaw pitch roll parent_frame child_frame
            # 180 degrees = 3.14159 radians
            arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'base_link1']
        )
    ])