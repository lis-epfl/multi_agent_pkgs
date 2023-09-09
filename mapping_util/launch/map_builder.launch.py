from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get config file
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    # create node
    agent_node = Node(
        package='mapping_util',
        executable='map_builder_node',
        name='map_builder_node',
        parameters=[config],
        # prefix=['xterm -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(agent_node)
    return ld
