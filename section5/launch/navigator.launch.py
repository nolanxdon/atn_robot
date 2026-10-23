from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    clock = {"use_sim_time": use_sim_time}
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="asl_tb3_lib", executable="rviz_goal_relay.py",
             parameters=[clock, {"output_channel": "/cmd_nav"}]),
        Node(package="asl_tb3_lib", executable="state_publisher.py", parameters=[clock]),
        Node(package="section5", executable="navigator.py", parameters=[clock],
             output="screen"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("asl_tb3_sim"), "launch", "rviz.launch.py"])),
            launch_arguments={
                "config": PathJoinSubstitution([
                    FindPackageShare("section5"), "rviz", "default.rviz"]),
                "use_sim_time": use_sim_time,
            }.items()),
    ])
