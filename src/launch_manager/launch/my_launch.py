from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='main_package',
            executable='main_node',
            name='main_node'
        ),
        Node(
            package='sensor_handler_package',
            executable='sensor_handler_node',
            name='sensor_handler_node',
            parameters=[
                {"test_param": "hello"}
            ]
        ),
        Node(
            package='sensor_package',
            executable='sensor1_script',
            name='sensor1_script',
            output='screen'
        ),
        Node(
            package='sensor_package',
            executable='sensor2_script',
            name='sensor2_script',
            output='screen'
        )
    ])
