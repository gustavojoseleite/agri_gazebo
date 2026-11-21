"""Uma fila de máquinas no carreador: um líder à frente, seguidores atrás.

    ros2 launch agri_gz_bringup convoy.launch.py
    ros2 launch agri_gz_bringup convoy.launch.py spacing:=15.0 followers:=2
    ros2 launch agri_gz_bringup convoy.launch.py world:=citrus_farm_60.sdf gui:=false
    ros2 launch agri_gz_bringup convoy.launch.py leader:=autonomous_tractor follower:=pickup

Qualquer veículo pode liderar e qualquer veículo pode seguir: leader:= e
follower:= nomeiam um tipo de veículo, achado pela mesma convenção <robot>_gz do
sim.launch.py, e cada um é solto na própria altura de criação. Nada aqui sabe o
que é um trator ou uma pickup. Este arquivo só alinha as máquinas; o que elas
fazem depois (seguir o líder, percorrer uma rota) cabe ao projeto que as lançou.

Por padrão o líder é uma **pickup** e os seguidores são **tratores autônomos**,
quatro, em fila pelo carreador principal a dez metros um do outro. O líder fica
na origem do mundo voltado para leste, que é onde todo mundo do agri_gz_worlds
põe um veículo (terreno nivelado, sobre a estrada, apontado ao longo dela), e
cada seguidor fica um espaçamento mais atrás:

    pickup       ( 0, 0)      /pickup/cmd_vel
    tractor_1    (-10, 0)     /tractor_1/cmd_vel
    tractor_2    (-20, 0)     ...
    tractor_3    (-30, 0)
    tractor_4    (-40, 0)

Dez metros entre os `base_link`, o que num trator de 3,6 m dá 6,4 m de vão entre
um e o próximo: perto o bastante para parecer um comboio, longe o bastante para
que ninguém bata na traseira de ninguém enquanto a física assenta.

**Cada máquina é o próprio namespace ROS**, e isso não é capricho: os plugins
põem seus tópicos gz sob /model/<name>/, então cinco veículos chegariam todos em
/cmd_vel e /odom e brigariam. Conduzir o terceiro trator é
`ros2 run vehicle_control teleop_key --ros-args -r __ns:=/tractor_3`.
Os seguidores se chamam <follower_prefix>_<n>, tractor_1 em diante por padrão.

**Só o líder leva câmera.** A câmera é a coisa mais cara em qualquer um destes
mundos (medido: 16,8 dos 62,1 segundos de relógio que levou para simular 20 s
do citrus_farm_60), então os seguidores são criados com camera:=false. Eles
mantêm GNSS e IMU, que são baratos e com os quais um seguidor se localiza.
followers_camera:=true dá câmeras a eles também, pelo preço acima;
followers_sensors:=false tira também o GNSS e a IMU.
"""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from agri_gz_bringup.vehicles import vehicle_nodes


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

    world_path = Path(get_package_share_directory("agri_gz_worlds")) / "worlds" / world
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

    # A fila corre para trás ao longo do rumo, seja qual for o rumo: gire o
    # comboio com yaw:= e ele continua um comboio em vez de virar uma fileira.
    back_x, back_y = -math.cos(yaw), -math.sin(yaw)

    # Sem altura de criação aqui: cada veículo traz a sua (`spawn.yaml` no seu
    # pacote <robot>_gz), o que permite a qualquer tipo assumir qualquer papel.
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
            description="World file, relative to agri_gz_worlds/worlds.",
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
