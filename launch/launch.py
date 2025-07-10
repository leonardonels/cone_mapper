from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('cone_mapper'),
        'config',
        'params.yaml'
    )
    return LaunchDescription([
        Node(
            package='cone_mapper',
            executable='cone_mapper_node',
            name='cone_mapper_node',
            output='screen',
            parameters=[params_file]
        )
    ])
