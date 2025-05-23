from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
# ...

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.join(
            get_package_share_directory('bumperbot_mapping'),
            'maps/basketball_court/map.yaml'),
                'topic_name': 'map',
                'frame_id': 'map'
            }]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{
                'node_names': ['map_server'],
                'autostart': True,
            }]
        )
    ])