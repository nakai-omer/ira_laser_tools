from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tf_node_1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name='ira_static_broadcaster_d',
        arguments=[
            '--frame-id', 'base_link', '--child-frame-id', 'scandx',
            '--x', '-0.3', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ],
    )

    tf_node_2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name='ira_static_broadcaster_s',
        arguments=[
            '--frame-id', 'base_link', '--child-frame-id', 'scansx',
            '--x', '0.3', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ],
    )

    laserscan_virtualizer = Node(
        package='ira_laser_tools',
        namespace='',
        executable='laserscan_virtualizer',
        name='laserscan_virtualizer',
        output='screen',
        parameters=[{
            # INPUT POINT CLOUD
            "cloud_topic": "/merged_cloud",
            # REFERENCE FRAME WHICH LASER(s) ARE RELATED
            "base_frame": "base_link",
            # VIRTUAL LASER OUTPUT TOPIC, LEAVE VALUE EMPTY TO PUBLISH ON THE VIRTUAL LASER NAMES (param: output_laser_scan)
            "output_laser_topic": "/scan",
            # LIST OF THE VIRTUAL LASER SCANS. YOU MUST PROVIDE THE STATIC TRANSFORMS TO TF, SEE ABOVE
            "virtual_laser_scan": "scansx scandx",
            'angle_min': -1.57,
            'angle_max': 1.57,
            'angle_increment': 0.00437,
            'scan_time': 0.0,
            'range_min': 0.0,
            'range_max': 3.0,
        }],
    )

    description = LaunchDescription()
    description.add_action(tf_node_1)
    description.add_action(tf_node_2)
    description.add_action(laserscan_virtualizer)
    return description
