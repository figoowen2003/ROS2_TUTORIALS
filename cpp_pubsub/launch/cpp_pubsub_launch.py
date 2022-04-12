from http.server import executable
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='cpp_pubsub',
                namespace='ns1',
                executable='my_publiser',
                name='publiser_demo'
            ),
            Node(
                package='cpp_pubsub',
                namespace='ns1',
                executable='my_subscriber',
                name='subscriber_demo'
            )
        ]
    )