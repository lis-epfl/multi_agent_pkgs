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
        'agent_crazyflie_config.yaml'
    )

    config_mapper = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    config_bridge = os.path.join(
        get_package_share_directory('planner_crazyswarm_bridge'),
        'config',
        'bridge_default_config.yaml'
    )

    # define params
    voxel_grid_range = [12.0, 12.0, 5.0]
    # height of the start and the goal
    z_plane = 1.0 
    goal_offset = (5.7, 0, 0)
    use_mapping_util = True
    free_grid = True
    save_stats = True

    # set start and goal positions manually
    start_positions = [(-3.09, 1.32, z_plane),
                       (-3.107, 0.42, z_plane),
                       (-3.13, -0.47, z_plane),
                       (-3.14, -1.37, z_plane),
                       (-2.36, 0.88, z_plane),
                       (-2.38, -0.05, z_plane),
                       (-2.38, -0.92, z_plane)]

    goal_positions = []
    for x, y, z in start_positions:
        goal_x = x + goal_offset[0]
        goal_y = y + goal_offset[1]
        goal_z = z + goal_offset[2]
        goal_positions.append((goal_x, goal_y, goal_z))

    n_rob = len(start_positions)

    # create mapping nodes
    if use_mapping_util:
        for i in range(n_rob):
            params_sub = [{'id': i},
                          {'voxel_grid_range': voxel_grid_range},
                          {'free_grid': free_grid}]
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
        # prefix_tmp = []
        # if i == 6:
        #     prefix_tmp = ['xterm -fa default -fs 5 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args']
        params_sub = [{'state_ini': list(start_positions[i])},
                      {'n_rob': n_rob},
                      {'id': i},
                      {'goal': list(goal_positions[i])},
                      {'use_mapping_util': use_mapping_util},
                      {'voxel_grid_update_period': 10.0},
                      {'voxel_grid_range': voxel_grid_range},
                      {'save_stats': save_stats}]
        # if i == 8:
        #     params_sub = params_sub + [{'planner_verbose': True}]
        node_planner = Node(
            package='multi_agent_planner',
            executable='agent_node',
            name='agent_node_{}'.format(i),
            parameters=[config_planner] + params_sub,
            # prefix=['xterm -fa default -fs 5 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            # prefix=prefix_tmp,
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node_planner)

    # create bridge nodes
    for i in range(n_rob):
        params_sub = [{'id': i}]
        node_bridge = Node(
            package='planner_crazyswarm_bridge',
            executable='bridge_node',
            name='bridge_node_{}'.format(i),
            parameters=[config_bridge] + params_sub,
            # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            # prefix=prefix_tmp,
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node_bridge)

    return ld
