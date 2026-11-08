"""Bring up any vehicle of this workspace in Gazebo Harmonic.

    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor gui:=false
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
        world:=citrus_farm_60.sdf x:=-40.0 y:=6.0 yaw:=1.57

One launch file for every vehicle, instead of a copy per vehicle that slowly
drifts. What makes that possible is convention: a package named <robot>_gz is
expected to ship

    urdf/<robot>.gz.xacro     the model, with its Gazebo tags and plugins
    config/bridge.yaml        the topic list, as a template (see below)

and nothing else is looked up. Each <robot>_gz still keeps a three-line
sim.launch.py of its own so that `ros2 launch autonomous_tractor_gz
sim.launch.py` works and tab-completes; it forwards to this one.

Four processes come up together:

  gz sim                  the simulator, running the world
  robot_state_publisher   URDF + /joint_states -> the TF tree
  create                  spawns the robot from the robot_description topic
  parameter_bridge        connects the gz topics to ROS

Two names, two different jobs:

  name       the model's name inside Gazebo. Gazebo scopes a plugin's relative
             topics under /model/<name>/, so this is what keeps two machines in
             one world from both publishing on /odom. bridge.yaml is a template
             precisely because it has to name those topics, and it cannot know
             the name until launch time — {name} in it is substituted here.
  namespace  the ROS-side prefix. Every node below runs in it, and the relative
             topic names in bridge.yaml resolve under it. Empty by default, so
             a single-vehicle session gets plain /cmd_vel and /odom.

Nothing here starts rviz. Run it yourself from another shell when you want it.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# What it takes to bring one vehicle up lives in the vehicles module of this
# package, because convoy.launch.py needs exactly the same thing five times over.
from agri_gz_bringup.vehicles import vehicle_nodes


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    namespace = LaunchConfiguration("namespace").perform(context)

    # Defaulting here rather than in the DeclareLaunchArgument: the default is
    # another argument's value, which a declaration cannot express.
    name = LaunchConfiguration("name").perform(context) or robot

    world_path = Path(get_package_share_directory("agri_gz_worlds")) / "worlds" / world
    if not world_path.exists():
        raise FileNotFoundError(f"{world_path} does not exist")

    # -r starts unpaused; -s runs the server only, which is how the same launch
    # file serves both an interactive session and a headless CI run.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("ros_gz_sim")) / "launch" / "gz_sim.launch.py"
        )),
        launch_arguments={
            "gz_args": f"{world_path} -r -v 3" + ("" if gui else " -s"),
        }.items(),
    )

    pose = tuple(LaunchConfiguration(k).perform(context)
                 for k in ("x", "y", "z", "yaw"))
    return [gz_sim, *vehicle_nodes(robot, name, namespace, pose)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot",
            default_value="autonomous_tractor",
            description=("Vehicle to bring up. A package named <robot>_gz must exist "
                         "and ship urdf/<robot>.gz.xacro and config/bridge.yaml."),
        ),
        DeclareLaunchArgument(
            "world",
            default_value="empty.sdf",
            description=("World file, relative to agri_gz_worlds/worlds: empty.sdf for the "
                         "smoke test, or one of citrus_farm_90/_60/_30.sdf."),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="false runs the simulation server without the Gazebo window.",
        ),
        DeclareLaunchArgument(
            "name",
            default_value="",
            description="Model name in Gazebo. Empty means the value of robot:=.",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace for every node and topic. Empty means none.",
        ),
        # The default pose is the centre of the world, and that is a decision the
        # worlds cooperate with rather than a placeholder. Every citrus world is
        # levelled between x = +/-25 m and carries a carreador through the origin
        # running along +x — check_network() refuses to write one whose origin
        # road is more than 3 degrees off — so 0 0 with yaw 0 puts a vehicle in
        # the middle of the property, on the road, pointing down it, in any of
        # them. Measured after settling, in all five: (0, 0, 0) and level to
        # 0.00 degrees in roll, pitch and yaw.
        DeclareLaunchArgument(
            "x", default_value="0.0",
            description="Spawn X, metres. 0 is the centre of the world.",
        ),
        DeclareLaunchArgument(
            "y", default_value="0.0",
            description="Spawn Y, metres. 0 is the centre of the world.",
        ),
        DeclareLaunchArgument(
            "z",
            default_value="",
            description=("Spawn Z, metres. Empty means the vehicle's own height, "
                         "from <robot>_gz/config/spawn.yaml."),
        ),
        DeclareLaunchArgument(
            "yaw", default_value="0.0",
            description="Spawn heading, radians. 0 points along the origin carreador.",
        ),

        OpaqueFunction(function=launch_setup),
    ])
