from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    laserscan_multi_merger = Node(
        package='ira_laser_tools',
        namespace='',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        output='screen',
        parameters=[{
            'destination_frame': 'base_link',
            'cloud_destination_topic': '/merged_cloud',
            'scan_destination_topic': '/scan_multi',
            'laserscan_topics': '/scansx /scandx',
            # LIST OF THE LASER SCAN TOPICS TO SUBSCRIBE
            'angle_min': -1.57,
            'angle_max': 1.57,
            'angle_increment': 0.00437,
            'scan_time': 0.0,
            'range_min': 0.0,
            'range_max': 3.0,
        }],
    )

    description = LaunchDescription()
    description.add_action(laserscan_multi_merger)
    return description
