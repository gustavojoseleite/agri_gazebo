"""Sobe qualquer veículo deste workspace no Gazebo Harmonic.

    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor gui:=false
    ros2 launch agri_gz_bringup sim.launch.py robot:=autonomous_tractor \
        world:=citrus_farm_60.sdf x:=-40.0 y:=6.0 yaw:=1.57

Um só `launch` para todos os veículos, em vez de uma cópia por veículo que vai
divergindo aos poucos. O que torna isso possível é a convenção: espera-se que um
pacote chamado <robot>_gz traga

    urdf/<robot>.gz.xacro     o modelo, com suas tags e plugins do Gazebo
    config/bridge.yaml        a lista de tópicos, como modelo (ver abaixo)

e nada mais é procurado. Cada <robot>_gz ainda mantém um `sim.launch.py` próprio
de três linhas, para que `ros2 launch autonomous_tractor_gz sim.launch.py`
funcione e complete com tab; ele repassa para este.

Estes sobem juntos:

  gz sim                  o simulador, executando o mundo
  robot_state_publisher   URDF + /joint_states -> a árvore de TF
  create                  cria o robô a partir do tópico robot_description
  parameter_bridge        liga os tópicos gz (relógio, sensores) ao ROS
  spawner                 `joint_state_broadcaster`, `ackermann_steering_controller`
                          e `front_drive_controller`, no `controller manager`
                          que o `gz_ros2_control` executa dentro do Gazebo
  cmd_vel_limiter         cmd_vel -> limites -> o controlador de direção

Dois nomes, dois trabalhos diferentes:

  name       o nome do modelo dentro do Gazebo. O Gazebo põe os tópicos relativos
             de um plugin sob /model/<name>/, então é isto que impede duas
             máquinas no mesmo mundo de publicarem ambas em /odom. O bridge.yaml é
             um modelo justamente porque precisa nomear esses tópicos e não pode
             saber o nome até a hora do `launch`: o {name} dele é substituído aqui.
  namespace  o prefixo do lado ROS. Todo nó abaixo roda nele, e os nomes de
             tópico relativos do bridge.yaml se resolvem sob ele. Vazio por
             padrão, então uma sessão de um só veículo tem /cmd_vel e /odom simples.

Nada aqui inicia o `rviz`. Execute-o você mesmo em outro terminal quando quiser.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# O que é preciso para subir um veículo fica no módulo `vehicles` deste pacote,
# porque o convoy.launch.py precisa exatamente da mesma coisa cinco vezes.
from agri_gz_bringup.vehicles import vehicle_nodes


def launch_setup(context, *args, **kwargs):
    robot = LaunchConfiguration("robot").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() == "true"
    namespace = LaunchConfiguration("namespace").perform(context)

    # O padrão é definido aqui e não no `DeclareLaunchArgument`: o padrão é o valor
    # de outro argumento, o que uma declaração não consegue expressar.
    name = LaunchConfiguration("name").perform(context) or robot

    world_path = Path(get_package_share_directory("agri_gz_worlds")) / "worlds" / world
    if not world_path.exists():
        raise FileNotFoundError(f"{world_path} does not exist")

    # -r inicia sem pausa; -s executa só o servidor, e é assim que o mesmo `launch`
    # serve tanto a uma sessão interativa quanto a uma execução de CI sem janela.
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
        # A pose padrão é o centro do mundo, e isso é uma decisão com a qual os
        # mundos cooperam, não um valor provisório. Todo mundo de citros é
        # nivelado entre x = +/-25 m e tem um carreador pela origem seguindo +x
        # (`check_network()` recusa gerar um cuja estrada na origem fique mais de
        # 3 graus torta), então 0 0 com yaw 0 põe o veículo no meio da propriedade,
        # sobre a estrada, apontado ao longo dela, em qualquer um deles. Medido
        # após assentar, nos cinco: (0, 0, 0) e nivelado a 0,00 grau em rolagem,
        # arfagem e guinada.
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
