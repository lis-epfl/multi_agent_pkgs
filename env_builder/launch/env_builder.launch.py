from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    # Create the NatNet client node
    config = os.path.join(
        get_package_share_directory('env_builder'),
        'config',
        # 'env_default_config.yaml'
        'env_long_config.yaml'
        # 'env_loop_config.yaml'
        # 'env_new_config.yaml'
        # 'env_crazyflie_config.yaml'
    )
    params_sub = [{'publish_period': 0.1}]
    env_builder_node = Node(
        package='env_builder',
        executable='env_builder_node',
        name='env_builder_node',
        parameters=[config] + params_sub,
        # prefix=['xterm -fa default -fs 10 -e gdb -ex run --args'],
        output='screen',
        emulate_tty=True
    )

    ld.add_action(env_builder_node)
    return ld
