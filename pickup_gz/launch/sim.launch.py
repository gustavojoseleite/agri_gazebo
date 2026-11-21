"""Sobe a pickup da fazenda no Gazebo Harmonic.

    ros2 launch pickup_gz sim.launch.py
    ros2 launch pickup_gz sim.launch.py gui:=false
    ros2 launch pickup_gz sim.launch.py world:=citrus_farm_60.sdf

Um invólucro, de propósito. Todo o trabalho está no agri_gz_bringup, que é
compartilhado por todos os veículos; este arquivo existe para que a pickup possa
ser lançada pelo nome do próprio pacote. A altura de criação dela não está aqui,
mas em config/spawn.yaml, de onde todo `launch` que a cria (comboio incluído) a lê.

Para pôr a pickup à frente de uma fila de tratores em vez de executá-la
sozinha, veja agri_gz_bringup/launch/convoy.launch.py.

Um `launch` incluído não herda a linha de comando, então cada argumento é
declarado aqui e repassado explicitamente. A lista é curta e é o preço de
`ros2 launch pickup_gz sim.launch.py world:=...` funcionar como
qualquer um esperaria.
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
                              description=("World file, relative to agri_gz_worlds/worlds: "
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
                FindPackageShare("agri_gz_bringup"), "launch", "sim.launch.py",
            ])),
            launch_arguments={
                "robot": "pickup",
                **{arg: LaunchConfiguration(arg) for arg in FORWARDED},
            }.items(),
        ),
    ])
