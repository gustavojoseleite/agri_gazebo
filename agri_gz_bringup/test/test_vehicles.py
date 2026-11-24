"""Testes da composição do comboio e da configuração da bridge (F1.2, R1, R4 e R6).

`convoy_line` diz quem é cada veículo: o líder tem a tag 0 e não tem câmera; o seguidor k tem
a tag k e câmera. `bridge_config` monta as entradas da bridge de um veículo a partir do
`bridge.yaml` dele: troca `{name}` pelo nome e, na montagem dupla, troca as entradas da
`camera_front` pelas duas câmeras. O `convoy.launch.py` não tem mais chaves que tiram sensores
dos seguidores e recebe a família, o tamanho da tag, a montagem de câmera e o perfil de sensor.
"""
import importlib.util
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory

from agri_gz_bringup.vehicles import bridge_config, convoy_line, gz_args

BRIDGE = """
- ros_topic_name: "imu/data"
  gz_topic_name: "/model/{name}/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
- ros_topic_name: "_sim/camera_front/image"
  gz_topic_name: "/model/{name}/camera_front/image"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS
- ros_topic_name: "camera_front/camera_info"
  gz_topic_name: "/model/{name}/camera_front/camera_info"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
- ros_topic_name: "/ground_truth/{name}/pose"
  gz_topic_name: "/model/{name}/pose"
  ros_type_name: "geometry_msgs/msg/PoseStamped"
  gz_type_name: "gz.msgs.Pose"
  direction: GZ_TO_ROS
"""


def test_convoy_line():
    line = convoy_line('pickup', 'autonomous_tractor', 2, 'tractor')
    assert line == [
        {'robot': 'pickup', 'name': 'pickup', 'tag_id': 0, 'camera': False},
        {'robot': 'autonomous_tractor', 'name': 'tractor_1', 'tag_id': 1, 'camera': True},
        {'robot': 'autonomous_tractor', 'name': 'tractor_2', 'tag_id': 2, 'camera': True},
    ]


def test_convoy_line_leader_name_and_no_followers():
    assert convoy_line('pickup', 'autonomous_tractor', 0, 'tractor', leader_name='lider') == [
        {'robot': 'pickup', 'name': 'lider', 'tag_id': 0, 'camera': False}]


def test_bridge_single_camera():
    entries = bridge_config(BRIDGE, 'tractor_1', 'single')
    assert yaml.safe_load(BRIDGE.replace('{name}', 'tractor_1')) == entries


def test_bridge_dual_camera():
    entries = bridge_config(BRIDGE, 'tractor_1', 'dual')
    ros = sorted(e['ros_topic_name'] for e in entries)
    assert ros == sorted(['imu/data', '/ground_truth/tractor_1/pose',
                          '_sim/camera_front_left/image', 'camera_front_left/camera_info',
                          '_sim/camera_front_right/image', 'camera_front_right/camera_info'])
    gz = {e['ros_topic_name']: e['gz_topic_name'] for e in entries}
    assert gz['_sim/camera_front_left/image'] == '/model/tractor_1/camera_front_left/image'
    assert gz['camera_front_right/camera_info'] == \
        '/model/tractor_1/camera_front_right/camera_info'
    assert not any('{name}' in str(e) for e in entries)


def test_bridge_without_camera_entries_is_unchanged_by_the_mount():
    text = BRIDGE.split('- ros_topic_name: "_sim/camera_front/image"')[0]
    assert bridge_config(text, 'pickup', 'dual') == bridge_config(text, 'pickup', 'single')


def _convoy_arguments():
    path = (Path(get_package_share_directory('agri_gz_bringup')) / 'launch'
            / 'convoy.launch.py')
    spec = importlib.util.spec_from_file_location('convoy_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    return {a.name: a.default_value for a in description.get_launch_arguments()}


def test_convoy_arguments():
    names = set(_convoy_arguments())
    assert {'tag_family', 'tag_size', 'camera_mount', 'sensor_profile', 'seed'} <= names
    assert 'followers_sensors' not in names
    assert 'followers_camera' not in names


def test_convoy_has_the_ground_height():
    # O relevo varia ao longo de X: a fila nasce sobre o chão do ponto de partida.
    assert _convoy_arguments()['z'] is not None


def test_gz_args_seed():
    # Semente maior que zero vai para o --seed do gz sim; zero deixa o Gazebo sortear.
    assert gz_args("w.sdf", True, 7).endswith(" --seed 7")
    assert "--seed" not in gz_args("w.sdf", True, 0)
    assert "--seed" not in gz_args("w.sdf", True)


def test_sequential_spawn():
    # Cada veículo nasce depois do anterior, para a semente do Gazebo dar o mesmo ruído ao
    # mesmo veículo em toda execução.
    from launch.actions import RegisterEventHandler
    from launch_ros.actions import Node
    path = (Path(get_package_share_directory('agri_gz_bringup')) / 'launch'
            / 'convoy.launch.py')
    spec = importlib.util.spec_from_file_location('convoy_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    first = [Node(package='p', executable='x'), Node(package='ros_gz_sim', executable='create')]
    second = [Node(package='p', executable='y'), Node(package='ros_gz_sim', executable='create')]
    assert module.sequential_spawn(first, None) == first
    previous = module.spawn_node(first)
    assert previous is first[1]
    chained = module.sequential_spawn(second, previous)
    assert second[1] not in chained
    assert second[0] in chained
    assert isinstance(chained[-1], RegisterEventHandler)
