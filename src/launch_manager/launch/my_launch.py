from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sensor_names = [
        "gyroscope_sensor",
        "temperature_sensor",
        "voltage_sensor",
        "sole_pressure_sensor"
    ]

    return LaunchDescription([
        Node(
            package='main_package',
            executable='main_node',
            name='main_node'
        ),
        Node(
            package='sensor_package',
            executable=sensor_names[0],
            name=sensor_names[0],
            output='screen',
            parameters=[
                {"sensor_name": sensor_names[0]}
            ]
        ),
        Node(
            package='sensor_package',
            executable=sensor_names[1],
            name=sensor_names[1],
            output='screen',
            parameters=[
                {"sensor_name": sensor_names[1]}
            ]
        ),
        Node(
            package='sensor_package',
            executable=sensor_names[2],
            name=sensor_names[2],
            output='screen',
            parameters=[
                {"sensor_name": sensor_names[2]}
            ]
        ),
        Node(
            package='sensor_package',
            executable=sensor_names[3],
            name=sensor_names[3],
            output='screen',
            parameters=[
                {"sensor_name": sensor_names[3]}
            ]
        ),
        Node(
            package='sensor_handler_package',
            executable='sensor_handler_node',
            name='sensor_handler_node',
            parameters=[
                {"sensor_names": sensor_names}
            ]
        ),
    ])
