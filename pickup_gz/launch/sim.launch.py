"""Bring up the farm pickup in Gazebo Harmonic.

    ros2 launch pickup_gz sim.launch.py
    ros2 launch pickup_gz sim.launch.py gui:=false
    ros2 launch pickup_gz sim.launch.py world:=citrus_farm_60.sdf

A wrapper, on purpose. All of the work is in agro_gz_bringup, which is shared by
every vehicle; this file exists so the pickup can be launched by its own
package name. Its spawn height is not here but in config/spawn.yaml, where
every launch that spawns it (convoy included) reads it.

To put the pickup at the head of a line of tractors instead of running it on
its own, see agro_gz_bringup/launch/convoy.launch.py.

An included launch does not inherit the command line, so each argument is
declared here and forwarded explicitly. The list is short and it is the price
of `ros2 launch pickup_gz sim.launch.py world:=...` working the way
anyone would expect it to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

FORWARDED = ("world", "gui", "name", "namespace", "x", "y", "z", "yaw")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty.sdf",
                              description=("World file, relative to agro_gz_worlds/worlds: "
                                           "empty.sdf, or citrus_farm_90/_60/_30.sdf.")),
        DeclareLaunchArgument("gui", default_value="true",
                              description="false runs the server without the Gazebo window."),
        DeclareLaunchArgument("name", default_value="pickup",
                              description="Model name in Gazebo, and the scope of its topics."),
        DeclareLaunchArgument("namespace", default_value="",
                              description="ROS namespace for every node and topic."),
        DeclareLaunchArgument("x", default_value="0.0", description="Spawn X, metres."),
        DeclareLaunchArgument("y", default_value="0.0", description="Spawn Y, metres."),
        DeclareLaunchArgument("z", default_value="",
                              description="Spawn Z, metres. Empty: config/spawn.yaml."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Spawn heading, radians."),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("agro_gz_bringup"), "launch", "sim.launch.py",
            ])),
            launch_arguments={
                "robot": "pickup",
                **{arg: LaunchConfiguration(arg) for arg in FORWARDED},
            }.items(),
        ),
    ])
