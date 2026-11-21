"""Tudo o que é preciso para subir um veículo, num só lugar.

Importado pelos dois `launch` deste pacote (`sim.launch.py`, que sobe uma
máquina, e `convoy.launch.py`, que sobe várias) e por qualquer projeto que
monte o próprio cenário, porque um veículo são três nós e uma regra de nomes
de tópicos, e ter isso escrito duas vezes é ter duas definições que vão
divergir.

É um módulo Python instalado (`ament_python_install_package` no
`CMakeLists.txt`), então todo chamador o importa do jeito comum, com
`from agri_gz_bringup.vehicles import vehicle_nodes`.
"""

import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_control.launch_nodes import control_config, control_nodes


# Onde uma configuração de ponte deixa de ser infraestrutura e passa a ser sensores.
# Todo `<robot>_gz/config/bridge.yaml` traz esta linha, e é por isso que um seguidor
# criado sem sensores também chega sem os tópicos deles.
SENSOR_MARKER = "# \u2500\u2500 Sensors "
# E onde a câmera começa, abaixo dela: a câmera tem a própria chave.
CAMERA_MARKER = "# \u2500\u2500 Camera "


def vehicle_files(robot):
    """Onde ficam o modelo e o modelo de ponte de um veículo, já conferidos.

    A convenção é toda a busca: um pacote chamado <robot>_gz traz
    `urdf/<robot>.gz.xacro` e `config/bridge.yaml`, e nada mais é procurado.
    O arquivo ausente gera o erro aqui, em vez de falhar adiante como um
    `robot_description` vazio, que é bem mais difícil de ler.
    """
    share = Path(get_package_share_directory(f"{robot}_gz"))
    model = share / "urdf" / f"{robot}.gz.xacro"
    bridge = share / "config" / "bridge.yaml"
    for path in (model, bridge):
        if not path.exists():
            raise FileNotFoundError(f"{path} does not exist")
    return model, bridge


def spawn_height(robot):
    """A que altura do chão o veículo é solto, em metros.

    É uma propriedade do veículo, não do cenário que o cria: uma máquina com
    rodas de 1,5 m precisa de mais folga que uma pickup. Cada <robot>_gz a
    declara em `config/spawn.yaml` (`z: 0.4`), então um veículo novo funciona
    em todo `launch`, comboio incluído, sem que ninguém mais saiba o tamanho dele.
    """
    path = Path(get_package_share_directory(f"{robot}_gz")) / "config" / "spawn.yaml"
    if not path.exists():
        raise FileNotFoundError(f"{path} does not exist")
    return float(yaml.safe_load(path.read_text())["z"])


def vehicle_nodes(robot, name, namespace, pose, sensors=True, camera=None):
    """Os nós de que uma máquina precisa e a configuração de ponte que eles dividem.

    robot      o tipo do veículo: autonomous_tractor, pickup
    name       o nome dele no Gazebo e o escopo dos seus tópicos gz
    namespace  o namespace ROS sob o qual cai tudo o que é dele
    pose       (x, y, z, yaw), como strings ou números; z vazio ou None
               significa a altura de criação do próprio veículo (`spawn_height`)
    sensors    false deixa câmera, GNSS e IMU fora deste veículo
    camera     false deixa a câmera fora e mantém GNSS e IMU; None
               equivale a `sensors`. Os seguidores de um comboio se localizam
               com GNSS e IMU e não precisam da câmera, que é cara.

    O nome do modelo chega ao xacro porque os sensores usam um tópico gz
    absoluto (um relativo ficaria sob o nome do mundo também, e a ponte teria
    de acompanhá-lo), então o xacro precisa montá-lo.
    """
    model, bridge = vehicle_files(robot)
    camera = sensors if camera is None else (camera and sensors)
    x, y, z, yaw = pose
    if z in (None, ""):
        z = spawn_height(robot)
    x, y, z, yaw = (str(v) for v in (x, y, z, yaw))

    # Veículos com namespace mantêm a árvore de TF dentro dele. Todo veículo usa
    # os mesmos nomes de frame (base_link, camera_front_link…), então num único
    # /tf global duas máquinas sobrescreveriam as transformações uma da outra. A
    # ponte já envia a transformação gz odom -> base_footprint para o "tf"
    # relativo; o `robot_state_publisher` é apontado aqui para o mesmo lugar, e
    # qualquer nó que leia o TF deste veículo usa os mesmos dois remapeamentos.
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")] if namespace else []

    # O veículo é conduzido pelos controladores do `vehicle_control`, atrás de um
    # `controller manager` que o `gz_ros2_control` executa dentro do Gazebo; eles
    # publicam odom, tf e joint_states e leem cmd_vel pelo limitador. A
    # configuração é escrita por namespace: todo `gz_ros2_control` no único
    # processo do Gazebo leria, senão, os mesmos parâmetros. Ver
    # `vehicle_control.launch_nodes`.
    controllers = control_config(robot, namespace)

    robot_description = ParameterValue(
        Command([
            "xacro ", str(model),
            " name:=", name,
            " sensors:=", "true" if sensors else "false",
            " camera:=", "true" if camera else "false",
            " namespace:=", namespace,
            " controllers:=", controllers,
        ]),
        value_type=str,
    )

    # O `parameter_bridge` recebe um arquivo de configuração e nenhuma
    # substituição, então o arquivo que ele recebe é escrito aqui com {name}
    # preenchido. `delete=False` porque a ponte o abre depois que a descrição do
    # `launch` foi montada; são algumas centenas de bytes no diretório temporário
    # do sistema, e o SO os recupera.
    config = bridge.read_text()
    if not sensors:
        # Uma entrada da ponte cujo tópico gz não tem publicador ainda cria o
        # tópico ROS, e um tópico que existe e nunca carrega nada é pior que
        # nenhum: é algo em que se inscrever e esperar para sempre. Por isso a
        # metade de sensores do arquivo sai junto com os sensores.
        config = config.split(SENSOR_MARKER)[0]
    elif not camera:
        config = config.split(CAMERA_MARKER)[0]
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{name}_bridge.yaml", delete=False,
    ) as handle:
        handle.write(config.replace("{name}", name))
        bridge_config = handle.name

    nodes = [
        # `use_sim_time` em todo lugar: com o simulador conduzindo o relógio, um nó
        # deixado no tempo real produz erros de extrapolação de TF que parecem um
        # robô quebrado, e não um problema de relógio.
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

        # Cria a partir do tópico `robot_description` em vez de um caminho de
        # arquivo, de modo que o simulador recebe exatamente o URDF que o
        # `robot_state_publisher` usa. O tópico é relativo, e o namespace acima
        # é sob o qual ele se resolve.
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
    return nodes + control_nodes(namespace, controllers)
