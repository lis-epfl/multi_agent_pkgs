from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    # set params
    n_rob = 2 

    # create bridge nodes
    for i in range(n_rob):
        params_ = [{'id': i},
                   {'test_bool': True}]
        node_bridge = Node(
            package='planner_crazyswarm_bridge',
            executable='bridge_node',
            name='bridge_node_{}'.format(i),
            parameters= params_,
            # prefix=['xterm -fa default -fs 5 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            # prefix=prefix_tmp,
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node_bridge)


    return ld
