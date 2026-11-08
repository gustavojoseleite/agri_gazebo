"""A line of machines on the carreador: one leader in front, followers behind.

    ros2 launch agro_gz_bringup convoy.launch.py
    ros2 launch agro_gz_bringup convoy.launch.py spacing:=15.0 followers:=2
    ros2 launch agro_gz_bringup convoy.launch.py world:=citrus_farm_60.sdf gui:=false
    ros2 launch agro_gz_bringup convoy.launch.py leader:=autonomous_tractor follower:=pickup

Any vehicle can lead and any vehicle can follow: leader:= and follower:= name
a vehicle type, which is found by the same <robot>_gz convention as
sim.launch.py, and each one is dropped from its own spawn height. Nothing here
knows what a tractor or a pickup is. This file only lines machines up; what
they do next (follow the leader, drive a route) belongs to the project that
launched them.

By default the leader is a **pickup** and the followers are **autonomous
tractors**, four of them, nose to tail down the main carreador at ten metres.
The leader sits on the world origin facing east, which is where every world in
agro_gz_worlds puts a vehicle — level ground, on the road, pointing along it — and
each follower is one spacing further back:

    pickup       ( 0, 0)      /pickup/cmd_vel
    tractor_1    (-10, 0)     /tractor_1/cmd_vel
    tractor_2    (-20, 0)     ...
    tractor_3    (-30, 0)
    tractor_4    (-40, 0)

Ten metres between base_links, which on a 3.6 m tractor is 6.4 m of daylight
between one and the next — close enough to read as a convoy, far enough that
nobody rear-ends anybody while the physics settles.

**Each machine is its own ROS namespace**, and that is not tidiness: the
plugins scope their gz topics under /model/<name>/, so five vehicles would
otherwise all arrive on /cmd_vel and /odom and fight. Driving the third
tractor is `ros2 run vehicle_control teleop_key --ros-args -r __ns:=/tractor_3`.
Followers are named <follower_prefix>_<n>, tractor_1 and on by default.

**Only the leader carries a camera.** A camera is the most expensive thing in
any of these worlds -- measured at 16.8 of the 62.1 wall-seconds it took to
simulate 20 s of citrus_farm_60 -- so the followers are spawned with
camera:=false. They keep GNSS and IMU, which are cheap and what a follower
localises with. followers_camera:=true gives them cameras too, at the price
above; followers_sensors:=false takes GNSS and IMU away as well.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from agro_gz_bringup.vehicles import vehicle_nodes


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    world = arg("world")
    gui = arg("gui").lower() == "true"
    spacing = float(arg("spacing"))
    followers = int(arg("followers"))
    x0, y0, yaw = float(arg("x")), float(arg("y")), float(arg("yaw"))
    follower_sensors = arg("followers_sensors").lower() == "true"
    follower_camera = arg("followers_camera").lower() == "true"
    leader, follower = arg("leader"), arg("follower")
    leader_name = arg("leader_name") or leader
    prefix = arg("follower_prefix")

    world_path = Path(get_package_share_directory("agro_gz_worlds")) / "worlds" / world
    if not world_path.exists():
        raise FileNotFoundError(f"{world_path} does not exist")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
        )),
        launch_arguments={
            "gz_args": f"{world_path} -r -v 3" + ("" if gui else " -s"),
        }.items(),
    )

    # The line runs backwards along the heading, whatever the heading is: turn
    # the convoy with yaw:= and it stays a convoy instead of becoming a row.
    back_x, back_y = -math.cos(yaw), -math.sin(yaw)

    # No spawn height here: each vehicle brings its own (spawn.yaml in its
    # <robot>_gz package), which is what lets any type take either role.
    line = [(leader, leader_name)]
    line += [(follower, f"{prefix}_{n + 1}") for n in range(followers)]

    nodes = [gz_sim]
    for n, (robot, name) in enumerate(line):
        nodes += vehicle_nodes(
            robot, name, f"/{name}",
            (x0 + back_x * spacing * n, y0 + back_y * spacing * n, None, yaw),
            sensors=(n == 0 or follower_sensors),
            camera=(n == 0 or follower_camera),
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="citrus_farm.sdf",
            description="World file, relative to agro_gz_worlds/worlds.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="false runs the server without the Gazebo window.",
        ),
        DeclareLaunchArgument(
            "leader", default_value="pickup",
            description="Vehicle type in front. A package <leader>_gz must exist.",
        ),
        DeclareLaunchArgument(
            "leader_name", default_value="",
            description="Model name and ROS namespace of the leader. Empty means leader:=.",
        ),
        DeclareLaunchArgument(
            "follower", default_value="autonomous_tractor",
            description="Vehicle type of every follower. A package <follower>_gz must exist.",
        ),
        DeclareLaunchArgument(
            "follower_prefix", default_value="tractor",
            description="Followers are named <prefix>_1, <prefix>_2, ...",
        ),
        DeclareLaunchArgument(
            "followers", default_value="4",
            description="How many followers fall in behind the leader.",
        ),
        DeclareLaunchArgument(
            "spacing", default_value="10.0",
            description="Metres between one machine's base_link and the next.",
        ),
        DeclareLaunchArgument(
            "followers_sensors", default_value="true",
            description="false spawns the followers with no GNSS, IMU or camera at all.",
        ),
        DeclareLaunchArgument(
            "followers_camera", default_value="false",
            description=("true gives every follower a camera as well. "
                         "Costly: a camera is the most expensive thing in these worlds."),
        ),
        DeclareLaunchArgument("x", default_value="0.0",
                              description="Where the leader stands. 0 is the world centre."),
        DeclareLaunchArgument("y", default_value="0.0",
                              description="Where the leader stands. 0 is the world centre."),
        DeclareLaunchArgument("yaw", default_value="0.0",
                              description=("Heading of the whole line, radians. 0 points "
                                           "along the carreador through the origin.")),

        OpaqueFunction(function=launch_setup),
    ])
