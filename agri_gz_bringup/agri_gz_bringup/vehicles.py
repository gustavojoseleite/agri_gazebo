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

import copy
import os
import tempfile
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from vehicle_control.launch_nodes import control_config, control_nodes

from agri_gz_bringup.tags import tag_texture


# Prefixo dos tópicos ROS da câmera na configuração da ponte. As entradas que começam
# com ele são as da câmera; na montagem dupla viram as das duas câmeras.
CAMERA = "camera_front"
# Os nomes das câmeras de cada montagem.
CAMERAS = {
    "single": [CAMERA],
    "dual": [f"{CAMERA}_left", f"{CAMERA}_right"],
}
# Vendor EGL da NVIDIA: sem ele o EGL escolhe o Mesa e a câmera não renderiza no container.
NVIDIA_EGL = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"


def convoy_line(leader, follower, followers, prefix, leader_name=""):
    """Quem é cada veículo do comboio, do líder ao último seguidor.

    O líder tem a tag 0 e não tem câmera; o seguidor k tem a tag k e câmera.
    Devolve uma lista de dicionários com `robot`, `name`, `tag_id` e `camera`.
    """
    line = [{"robot": leader, "name": leader_name or leader, "tag_id": 0, "camera": False}]
    line += [
        {"robot": follower, "name": f"{prefix}_{k}", "tag_id": k, "camera": True}
        for k in range(1, followers + 1)
    ]
    return line


def bridge_config(text, name, camera_mount, clock=True):
    """As entradas da ponte de um veículo, a partir do `bridge.yaml` dele.

    `{name}` vira o nome do modelo. Com `clock=False` a entrada do `/clock` sai: num comboio
    o relógio vem de uma ponte só, e não de uma por veículo. Na montagem `dual`, as entradas
    da `camera_front` são trocadas pelas das duas câmeras, `camera_front_left` e
    `camera_front_right`.
    """
    entries = yaml.safe_load(text.replace("{name}", name)) or []
    if not clock:
        entries = [e for e in entries if e["ros_topic_name"] != "/clock"]
    if camera_mount != "dual":
        return entries
    result = []
    for entry in entries:
        if not entry["ros_topic_name"].startswith(f"{CAMERA}/"):
            result.append(entry)
            continue
        for camera in CAMERAS["dual"]:
            copy_ = copy.deepcopy(entry)
            copy_["ros_topic_name"] = entry["ros_topic_name"].replace(
                f"{CAMERA}/", f"{camera}/", 1)
            copy_["gz_topic_name"] = entry["gz_topic_name"].replace(
                f"/{CAMERA}/", f"/{camera}/")
            result.append(copy_)
    return result


def has_camera(robot):
    """Se o `bridge.yaml` do tipo de veículo traz tópicos de câmera."""
    _, bridge = vehicle_files(robot)
    return f'"{CAMERA}/' in bridge.read_text()


def gz_args(world_path, gui):
    """Argumentos do `gz sim`: `-s` sem janela e, sem tela, render por EGL."""
    args = f"{world_path} -r -v 3"
    if not gui:
        args += " -s"
        if not os.environ.get("DISPLAY"):
            args += " --headless-rendering"
    return args


def egl_actions(gui):
    """Ações de `launch` que apontam o EGL para a NVIDIA, quando não há tela."""
    if gui or os.environ.get("DISPLAY") or not Path(NVIDIA_EGL).exists():
        return []
    return [SetEnvironmentVariable("__EGL_VENDOR_LIBRARY_FILENAMES", NVIDIA_EGL)]


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


def vehicle_nodes(robot, name, namespace, pose, tag_id=0, tag_family="36h11", tag_size=0.5,
                  camera=False, camera_mount="single", sensor_profile="default", clock=True):
    """Os nós de que uma máquina precisa e a configuração de ponte que eles dividem.

    robot           o tipo do veículo: autonomous_tractor, pickup
    name            o nome dele no Gazebo e o escopo dos seus tópicos gz
    namespace       o namespace ROS sob o qual cai tudo o que é dele
    pose            (x, y, z, yaw), como strings ou números; z vazio ou None
                    significa a altura de criação do próprio veículo (`spawn_height`)
    tag_id          o ID da tag na traseira; `tag_family` e `tag_size` (metros) a descrevem
    camera          false deixa de fora os tópicos da câmera e o nó que a distorce;
                    todo veículo tem GNSS, IMU e tag
    camera_mount    `single` ou `dual`, só vai ao xacro de quem tem câmera
    sensor_profile  `default`, `nominal` ou `ideal`: o perfil de GNSS e IMU
    clock           false tira o `/clock` da ponte do veículo; num comboio ele vem de uma
                    ponte só, e não de uma por veículo

    O nome do modelo chega ao xacro porque os sensores usam um tópico gz
    absoluto (um relativo ficaria sob o nome do mundo também, e a ponte teria
    de acompanhá-lo), então o xacro precisa montá-lo.
    """
    model, bridge = vehicle_files(robot)
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
            " namespace:=", namespace,
            " controllers:=", controllers,
            " tag_family:=", tag_family,
            " tag_id:=", str(tag_id),
            " tag_size:=", str(tag_size),
            " tag_texture:=", tag_texture(tag_family, tag_id),
            " sensor_profile:=", sensor_profile,
        ] + ([" camera_mount:=", camera_mount] if camera else [])),
        value_type=str,
    )

    # O `parameter_bridge` recebe um arquivo de configuração e nenhuma
    # substituição, então o arquivo que ele recebe é escrito aqui com {name}
    # preenchido. `delete=False` porque a ponte o abre depois que a descrição do
    # `launch` foi montada; são algumas centenas de bytes no diretório temporário
    # do sistema, e o SO os recupera.
    entries = bridge_config(bridge.read_text(), name, camera_mount, clock)
    if not camera:
        # Uma entrada da ponte cujo tópico gz não tem publicador ainda cria o
        # tópico ROS, e um tópico que existe e nunca carrega nada é pior que
        # nenhum: é algo em que se inscrever e esperar para sempre. Por isso as
        # entradas da câmera saem junto com ela.
        entries = [e for e in entries
                   if not e["ros_topic_name"].startswith(f"{CAMERA}")]
    with tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{name}_bridge.yaml", delete=False,
    ) as handle:
        yaml.safe_dump(entries, handle)
        bridge_config_file = handle.name

    # A pose real do veículo, que os nós de degradação usam no gatilho por zona.
    ground_truth = f"/ground_truth/{name}/pose"
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
                "config_file": bridge_config_file,
                "use_sim_time": True,
            }],
            output="screen",
        ),

        # GNSS, IMU e odometria chegam pelos tópicos `_ideal`; este processo publica os
        # normais, degradados conforme os parâmetros `loss.*` de cada nó.
        Node(
            package="agri_sensor_degradation",
            executable="sensors",
            namespace=namespace,
            parameters=[{"use_sim_time": True, "ground_truth_topic": ground_truth}],
            output="screen",
        ),
    ]
    # A imagem sai do Gazebo sem distorção (`image_ideal`); este nó aplica a de Brown do
    # `camera_info` e publica `image_raw`, uma vez por câmera.
    if camera:
        nodes += [
            Node(
                package="agri_gz_bringup",
                executable="camera_distortion",
                namespace=namespace,
                name=f"distortion_{camera_name}",
                parameters=[{
                    "camera": camera_name,
                    "use_sim_time": True,
                    "ground_truth_topic": ground_truth,
                }],
                output="screen",
            )
            for camera_name in CAMERAS[camera_mount]
        ]
    return nodes + control_nodes(namespace, controllers, odom_topic="odom_ideal")
