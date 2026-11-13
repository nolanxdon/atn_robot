#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    clock = {"use_sim_time": use_sim_time}

    return LaunchDescription([

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true"
        ),

        # Launch RViz
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"),
                "launch",
                "rviz.launch.py"
            ]),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("section5"),
                    "rviz",
                    "default.rviz"
                ]),
                "use_sim_time": use_sim_time,
            }.items(),
        ),

        # Publish robot state
        Node(
            package="asl_tb3_lib",
            executable="state_publisher.py",
            parameters=[clock],
        ),

        # Start navigator
        Node(
            package="section5",
            executable="navigator.py",
            parameters=[clock],
            output="screen",
        ),

        # Start frontier explorer
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="autonomy_repo",
                    executable="frontier_explorer.py",
                    output="screen",
                    parameters=[{"use_sim_time": True}],
                ),
            ],
        ),
    ])
