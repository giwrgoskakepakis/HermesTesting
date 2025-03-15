import launch
import launch_ros.actions
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
    LifecycleNode(package='lifecycle_node', executable='sensor',
                      name='lc_talker', namespace='', output='screen'),
        Node(package='lifecycle_handler', executable='handler', output='screen'),
        Node(package='lifecycle', executable='lifecycle_service_client', output='screen')
    ])
