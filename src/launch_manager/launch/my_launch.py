from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node

def generate_launch_description():
    sensor_node = LifecycleNode(
        package='lifecycle_node',
        executable='sensor',
        name='sensor_node',
        namespace='',
        output='screen'
    )

    handler_node = LifecycleNode(
        package='lifecycle_handler',
        executable='handler',
        name='handler_node',
        namespace='',
        output='screen'
    )

    main_node = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='main_node',
        namespace='',
        output='screen'
    )

    return LaunchDescription([
        sensor_node,
        handler_node,
        main_node
    ])