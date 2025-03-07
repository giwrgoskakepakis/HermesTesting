from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sensor1_name = "sensor1_data"
    sensor2_name = "sensor2_data"

    sensor_names = [sensor1_name, sensor2_name]

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
                {"sensor_names": sensor_names}
            ]
        ),
        Node(
            package='sensor_package',
            executable='sensor1_script',
            name='sensor1_script',
            output='screen',
            parameters=[
                {"sensor_name": sensor1_name}
            ]
        ),
        Node(
            package='sensor_package',
            executable='sensor2_script',
            name='sensor2_script',
            output='screen',
            parameters=[
                {"sensor_name": sensor2_name}
            ]
        )
    ])
