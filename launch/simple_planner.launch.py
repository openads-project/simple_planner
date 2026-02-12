#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from tracetools_launch.action import Trace


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("ego_data_topic", default_value="~/ego_data"),
        DeclareLaunchArgument("route_topic", default_value="~/route"),
        DeclareLaunchArgument("trajectory_topic", default_value="~/trajectory"),
        DeclareLaunchArgument("left_turn_indicator_service", default_value="~/enable_left_turn_indicator"),
        DeclareLaunchArgument("right_turn_indicator_service", default_value="~/enable_right_turn_indicator"),
        DeclareLaunchArgument("hazard_lights_service", default_value="~/enable_hazard_lights"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="simple_planner", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("simple_planner"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        DeclareLaunchArgument("trace", default_value="false", description="Enable tracing"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="simple_planner",
            executable="simple_planner_node",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        ),
        Trace(
            session_name='trace',
            dual_session=True,
            condition=IfCondition(LaunchConfiguration("trace")),
        ),
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])