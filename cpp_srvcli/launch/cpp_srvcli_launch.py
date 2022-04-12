import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    arg1 = LaunchConfiguration('arg1', default='1')
    arg2 = LaunchConfiguration('arg2', default='2')
    # for custom srv test
    arg3 = LaunchConfiguration('arg3', default='3')

    server = launch_ros.actions.Node(
        package='cpp_srvcli', executable='server', output='screen')
    # client = launch_ros.actions.Node(
    #     package='cpp_srvcli', executable='client', output='screen', arguments=[arg1, arg2])
    # for custom srv test
    client = launch_ros.actions.Node(
        package='cpp_srvcli', executable='client', output='screen', arguments=[arg1, arg2, arg3]
    )
    return launch.LaunchDescription([
        server,
        client,
        # Shutdown launch when client exits.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=client,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])