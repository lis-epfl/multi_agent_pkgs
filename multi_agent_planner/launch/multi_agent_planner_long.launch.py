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
        # 'agent_default_config.yaml'
        'agent_agile_config.yaml'
    )

    # define params
    n_rob = 10
    dist_between_rob = 2
    x_pos = 0
    z_pos = 0 
    dist_start_goal = 96 

    # calculate equidistant start and goal positions on the same line
    start_positions = []
    goal_positions = []
    start_positions.append((x_pos,  5.0, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    goal_positions.append((x_pos + dist_start_goal, 5.0, z_pos))
    for i in range(n_rob-1):
        y = start_positions[0][1] + (i+1)*dist_between_rob
        start_positions.append((x_pos,  y, z_pos, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        goal_positions.append((x_pos + dist_start_goal, y, z_pos))

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
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node)

    return ld
