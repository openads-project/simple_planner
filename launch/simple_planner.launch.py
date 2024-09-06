from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value=PathJoinSubstitution([
        get_package_share_directory("simple_planner"), "config", "params.yml"])
    )

    node_name_arg = DeclareLaunchArgument('node_name', default_value='simple_planner')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='')

    ego_data_topic_arg = DeclareLaunchArgument('ego_data_topic', default_value='~/ego_data')
    route_topic_arg = DeclareLaunchArgument('route_topic', default_value='~/route')
    trajectory_topic_arg = DeclareLaunchArgument('trajectory_topic', default_value='~/trajectory')
  
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    node = Node(
        name=LaunchConfiguration('node_name'),
        namespace=LaunchConfiguration('namespace'),
        package='simple_planner',
        executable='simple_planner_node',
        parameters=[LaunchConfiguration('params')],
        remappings=[('~/ego_data', LaunchConfiguration('ego_data_topic')),
                    ('~/route', LaunchConfiguration('route_topic')),
                    ('~/trajectory', LaunchConfiguration('trajectory_topic'))
        ],
        output='screen',
        emulate_tty=True)

    return LaunchDescription([
        params_arg,
        node_name_arg,
        namespace_arg,
        ego_data_topic_arg,
        route_topic_arg,
        trajectory_topic_arg,
        use_sim_time_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        node
    ])
