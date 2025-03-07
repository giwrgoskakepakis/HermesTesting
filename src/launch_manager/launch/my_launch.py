from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sensor1_name = "gyroscope_sensor"
    sensor2_name = "temperature_sensor"

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
            executable='gyroscope_sensor',
            name='gyroscope_sensor',
            output='screen',
            parameters=[
                {"sensor_name": sensor1_name}
            ]
        ),
        Node(
            package='sensor_package',
            executable='temperature_sensor',
            name='temperature_sensor',
            output='screen',
            parameters=[
                {"sensor_name": sensor2_name}
            ]
        )
    ])
