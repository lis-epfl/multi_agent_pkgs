from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get config file
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('multi_agent_planner'),
        'config',
        'agent_default_config.yaml'
    )

    # define params
    n_rob = 5
    dist_between_rob = 0.6
    y_pos = 0
    z_pos = 0 
    dist_start_goal = 10

    # calculate equidistant start and goal positions on the same line
    start_positions = []
    goal_positions = []
    start_positions.append((0.0,  y_pos, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    goal_positions.append((0.0, y_pos + dist_start_goal, z_pos))
    for i in range(n_rob-1):
        x = (-1)**i * ((i // 2) + 1)*dist_between_rob
        start_positions.append((x,  y_pos, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_positions.append((x, y_pos + dist_start_goal, z_pos))

    # create nodes
    for i in range(n_rob):
        params_sub = [{'state_ini': list(start_positions[i])},
                      {'n_rob': n_rob},
                      {'id': i},
                      {'goal': list(goal_positions[i])}]
        node = Node(
            package='multi_agent_planner',
            executable='agent_node',
            name='agent_node_{}'.format(i),
            parameters=[config] + params_sub,
            # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            prefix=['xterm -fa default -fs 10 -hold -e'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node)

    return ld
