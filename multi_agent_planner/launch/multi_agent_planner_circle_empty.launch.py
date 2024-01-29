from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # get config file
    ld = LaunchDescription()
    config_planner = os.path.join(
        get_package_share_directory('multi_agent_planner'),
        'config',
        'agent_default_config.yaml'
        # 'agent_agile_config.yaml'
        # 'agent_crazyflie_config.yaml'
    )

    config_mapper = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    # define params
    radius = 10 
    center_x = 20.00
    center_y = 20.00
    n_rob = 10 
    use_mapping_util = True
    # use_mapping_util = False
    save_stats = True

    # calculate equidistant start and goal positions on the circle
    start_positions = []
    goal_positions = []
    for i in range(n_rob):
        angle = 2 * math.pi * i / n_rob
        start_positions.append(
            (center_x + radius * math.cos(angle), center_y + radius * math.sin(angle), 1.5, 0, 0, 0, 0, 0, 0))

    for i in range(n_rob):
        goal = start_positions[(i + n_rob // 2) % n_rob]
        goal_positions.append((goal[0], goal[1], goal[2]))

        # create mapping nodes
    if use_mapping_util:
        for i in range(n_rob):
            params_sub = [{'id': i}]
            node_mapper = Node(
                package='mapping_util',
                executable='map_builder_node',
                name='map_builder_node_{}'.format(i),
                parameters=[config_mapper] + params_sub,
                # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
                # prefix=['xterm -fa default -fs 10 -hold -e'],
                output='screen',
                emulate_tty=True,
            )
            ld.add_action(node_mapper)

    # create planner nodes
    for i in range(n_rob):
        params_sub = [{'state_ini': list(start_positions[i])},
                      {'n_rob': n_rob},
                      {'id': i},
                      {'goal': list(goal_positions[i])},
                      {'use_mapping_util': use_mapping_util},
                      {'voxel_grid_update_period': 10.0},
                      {'save_stats': save_stats}]
        node_planner = Node(
            package='multi_agent_planner',
            executable='agent_node',
            name='agent_node_{}'.format(i),
            parameters=[config_planner] + params_sub,
            # prefix=['xterm -fa default -fs 5 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node_planner)


    return ld
