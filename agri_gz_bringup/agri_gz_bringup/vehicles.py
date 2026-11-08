"""What bringing one vehicle up consists of, in one place.

Imported by both launch files in this package — sim.launch.py, which brings up
one machine, and convoy.launch.py, which brings up several — and by any
project that composes its own scenario, because a vehicle is three nodes and a
rule about topic names, and having that written twice is having two
definitions that will disagree.

It is an installed Python module (ament_python_install_package in
CMakeLists.txt), so every caller imports it the ordinary way:

    from agri_gz_bringup.vehicles import vehicle_nodes
"""

import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Where a bridge config stops being plumbing and starts being sensors. Every
# <robot>_gz/config/bridge.yaml carries this line, and it is why a follower
# spawned without sensors also arrives without their topics.
SENSOR_MARKER = "# \u2500\u2500 Sensors "
# And where the camera starts, below it: the camera has its own switch.
CAMERA_MARKER = "# \u2500\u2500 Camera "


def vehicle_files(robot):
    """Where a vehicle's model and bridge template live, checked.

    The convention is the whole of the lookup: a package named <robot>_gz
    ships urdf/<robot>.gz.xacro and config/bridge.yaml, and nothing else is
    looked for. A missing file is raised here rather than left to fail later
    as an empty robot_description, which is a much harder thing to read.
    """
    share = Path(get_package_share_directory(f"{robot}_gz"))
    model = share / "urdf" / f"{robot}.gz.xacro"
    bridge = share / "config" / "bridge.yaml"
    for path in (model, bridge):
        if not path.exists():
            raise FileNotFoundError(f"{path} does not exist")
    return model, bridge


def spawn_height(robot):
    """How high above the ground this vehicle is dropped, in metres.

    A property of the vehicle, not of the scenario that spawns it: a machine
    on 1.5 m wheels needs more clearance than a pickup does. Each <robot>_gz
    declares it in config/spawn.yaml (`z: 0.4`), so a new vehicle works in
    every launch, convoy included, without anyone else knowing its size.
    """
    path = Path(get_package_share_directory(f"{robot}_gz")) / "config" / "spawn.yaml"
    if not path.exists():
        raise FileNotFoundError(f"{path} does not exist")
    return float(yaml.safe_load(path.read_text())["z"])


def vehicle_nodes(robot, name, namespace, pose, sensors=True, camera=None):
    """The three nodes one machine needs, and the bridge config they share.

    robot      which vehicle type: autonomous_tractor, pickup
    name       its name in Gazebo, and the scope of its gz topics
    namespace  the ROS namespace everything of its own lands under
    pose       (x, y, z, yaw), as strings or numbers; z empty or None means
               the vehicle's own spawn height (spawn_height above)
    sensors    false leaves the camera, GNSS and IMU off this one
    camera     false leaves the camera off and keeps GNSS and IMU; None
               means the same as sensors. A convoy's followers localise
               with GNSS and IMU and do not need the expensive camera.

    The model name reaches the xacro because sensors take an absolute gz topic
    — a relative one would be scoped under the world name as well, which the
    bridge would then have to track — so the xacro has to build it.
    """
    model, bridge = vehicle_files(robot)
    camera = sensors if camera is None else (camera and sensors)
    x, y, z, yaw = pose
    if z in (None, ""):
        z = spawn_height(robot)
    x, y, z, yaw = (str(v) for v in (x, y, z, yaw))

    # Namespaced vehicles keep their TF tree inside the namespace. Every
    # vehicle uses the same frame names (base_link, camera_front_link…), so on
    # one global /tf two machines would overwrite each other's transforms. The
    # bridge already sends the gz odom -> base_footprint transform to the
    # relative "tf"; robot_state_publisher is pointed at the same place here,
    # and any node reading this vehicle's TF takes the same two remaps.
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")] if namespace else []

    robot_description = ParameterValue(
        Command([
            "xacro ", str(model),
            " name:=", name,
            " sensors:=", "true" if sensors else "false",
            " camera:=", "true" if camera else "false",
        ]),
        value_type=str,
    )

    # parameter_bridge takes a config file and no substitutions, so the file it
    # gets is written here with {name} filled in. delete=False because the
    # bridge opens it after the launch description has been built; it is a few
    # hundred bytes in the system temp directory and the OS reclaims it.
    config = bridge.read_text()
    if not sensors:
        # A bridge entry whose gz topic has no publisher still creates the ROS
        # topic, and a topic that exists and never carries anything is worse
        # than no topic: it is a thing to subscribe to and wait on forever. So
        # the sensor half of the file goes with the sensors.
        config = config.split(SENSOR_MARKER)[0]
    elif not camera:
        config = config.split(CAMERA_MARKER)[0]
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{name}_bridge.yaml", delete=False,
    ) as handle:
        handle.write(config.replace("{name}", name))
        bridge_config = handle.name

    return [
        # use_sim_time everywhere: with the simulator driving the clock, a node
        # left on wall time produces TF extrapolation errors that look like a
        # broken robot rather than a clock problem.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
            remappings=tf_remaps,
            output="screen",
        ),

        # Spawns from the robot_description topic rather than a file path, so
        # the simulator gets exactly the URDF robot_state_publisher is using.
        # The topic is relative, and the namespace above is what it resolves
        # under.
        Node(
            package="ros_gz_sim",
            executable="create",
            namespace=namespace,
            name=f"spawn_{name}",
            arguments=[
                "-topic", "robot_description",
                "-name", name,
                "-x", x, "-y", y, "-z", z, "-Y", yaw,
            ],
            output="screen",
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=namespace,
            name=f"bridge_{name}",
            parameters=[{
                "config_file": bridge_config,
                "use_sim_time": True,
            }],
            output="screen",
        ),
    ]
