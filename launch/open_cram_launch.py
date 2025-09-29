from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='open_cram',
            executable='open_cram_node',
            name='open_cram_node',
            output='screen',
            parameters=[{
                'openai_model': 'gpt-4o-mini',   # example placeholder
                'openwebui_url': 'http://192.168.200.10:3000',
                'timeout_s': 30.0
            }]
        )
    ])