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
        'agent_default_config.yaml'
    )

    # create node
    params_sub = [{'use_mapping_util': False}]
    agent_node = Node(
        package='multi_agent_planner',
        executable='agent_node',
        name='agent_node',
        parameters=[config] + params_sub,
        # prefix=['xterm -fa default -fs 10 -xrm "XTerm*selectToClipboard: true" -e gdb -ex run --args'],
        # prefix=["sudo \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  \"GUROBI_HOME=$GUROBI_HOME\" \"GRB_LICENSE_FILE=$GRB_LICENSE_FILE\" -u toumieh bash -c "],
        # shell=True
        output='screen',
        emulate_tty=True
    )

    ld.add_action(agent_node)
    return ld
