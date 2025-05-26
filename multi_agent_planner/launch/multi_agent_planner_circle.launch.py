from launch import LaunchDescription
from launch_ros.actions import Node
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    packet_loss_arg = DeclareLaunchArgument(
        'packet_loss_percentage', default_value='0.0',
        description='Percentage of packet loss (0.0 to 1.0)'
    )
    comm_delay_arg = DeclareLaunchArgument(
        'communication_delay', default_value='0.0',
        description='Communication delay in seconds'
    )
    # get config file
    ld = LaunchDescription()
    ld.add_action(packet_loss_arg)
    ld.add_action(comm_delay_arg)
    config_planner = os.path.join(
        get_package_share_directory('multi_agent_planner'),
        'config',
        # 'agent_default_config.yaml'
        'agent_agile_config.yaml'
    )

    config_mapper = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    # define params
    n_rob = 6
    radius = 22  
    center_x = 18 
    center_y = 15
    voxel_grid_range = [20.0, 20.0, 6.0]
    use_mapping_util = True
    free_grid = True
    save_stats = True

    packet_loss_percentage = LaunchConfiguration('packet_loss_percentage')
    communication_delay = LaunchConfiguration('communication_delay')

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
                      {'save_stats': save_stats},
                      {'packet_loss_percentage': packet_loss_percentage},
                      {'communication_delay': communication_delay}]
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

    return ld
