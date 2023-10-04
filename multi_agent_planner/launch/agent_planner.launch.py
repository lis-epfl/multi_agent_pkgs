from launch import LaunchDescription
from launch_ros.actions import Node
import os
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

    config_mapper = os.path.join(
        get_package_share_directory('mapping_util'),
        'config',
        'map_builder_default_config.yaml'
    )

    # params
    use_mapping_util = True
    voxel_grid_range = [20.0, 20.0, 6.0]
    n_it_decomp = 42
    # state_ini = [0.0, 20.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    # goal = [130.0, 20.0, 1.5]

    if use_mapping_util:
        params_sub = [{'id': 0},
                      {'voxel_grid_range': voxel_grid_range}]
        node_mapper = Node(
            package='mapping_util',
            executable='map_builder_node',
            name='map_builder_node_0',
            parameters=[config_mapper] + params_sub,
            # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
            # prefix=['xterm -fa default -fs 10 -hold -e'],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(node_mapper)

    # create node
    params_sub = [{'use_mapping_util': use_mapping_util},
                  {'planner_verbose': False},
                  {'voxel_grid_range': voxel_grid_range},
                  {'n_it_decomp': n_it_decomp}]
                  # {'state_ini': state_ini},
                  # {'goal': goal}]
    agent_node = Node(
        package='multi_agent_planner',
        executable='agent_node',
        name='agent_node_0',
        parameters=[config] + params_sub,
        # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
        # prefix=["sudo \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  \"GUROBI_HOME=$GUROBI_HOME\" \"GRB_LICENSE_FILE=$GRB_LICENSE_FILE\" -u toumieh bash -c "],
        # shell=True
        output='screen',
        emulate_tty=True
    )

    ld.add_action(agent_node)
    return ld
