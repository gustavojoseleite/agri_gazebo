"""Teste de simulação dos sensores do comboio (F1.2, R1 a R4 e R6). Marca `simulation`.

Sobe o `convoy.launch.py` sem GUI, com a pickup e um trator. Com `spacing:=11.425`, a câmera do
trator fica a 7,5 m da tag da pickup (7,5 + 1,13 + 2,795), a folga alvo de 10,8 km/h. O perfil é
o `ideal`: as taxas nominais, sem ruído. O `image_proc` retifica a imagem do trator e o
`apriltag_ros` procura a tag nela. Confere:
- as taxas medidas pelos carimbos de tempo, com ±5% (GNSS 1 Hz, IMU 20 Hz, câmera 10 Hz);
- o `camera_info`: o modelo *plumb_bob* e os coeficientes padrão;
- o fix sem ruído na posição da antena dada pela verdade de solo;
- a verdade de solo dos dois veículos;
- a detecção da tag 0 (a da pickup) pelo trator.
"""
import math
import os
import random
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix

pytestmark = pytest.mark.simulation

SPACING = 11.425
SIM_SECONDS = 15.0
WALL_TIMEOUT = 300.0
ANTENNA = {'pickup': (1.545, 0.0, 1.845), 'tractor_1': (1.075, 0.0, 2.685)}
RATES = {'/pickup/gnss/fix': 1.0, '/tractor_1/gnss/fix': 1.0, '/pickup/imu/data': 20.0,
         '/tractor_1/imu/data': 20.0, '/tractor_1/camera_front/image_raw': 10.0}
TYPES = {'gnss/fix': NavSatFix, 'imu/data': Imu, 'camera_front/image_raw': Image,
         'camera_front/camera_info': CameraInfo}


def stamp(msg):
    """O carimbo de tempo da mensagem, em s."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def world_origin():
    """Latitude, longitude e altitude da origem do `citrus_farm.sdf`."""
    path = (Path(get_package_share_directory('agri_gz_worlds')) / 'worlds'
            / 'citrus_farm.sdf')
    coords = ET.parse(path).getroot().find('world/spherical_coordinates')
    return (float(coords.find('latitude_deg').text), float(coords.find('longitude_deg').text),
            float(coords.find('elevation').text))


def to_local(fix, origin):
    """(leste, norte, cima) em m a partir da origem do mundo, no elipsoide WGS84."""
    lat0, lon0, h0 = origin
    a, e2 = 6378137.0, 6.69437999014e-3
    phi = math.radians(lat0)
    w = 1 - e2 * math.sin(phi) ** 2
    meridian = a * (1 - e2) / w ** 1.5
    normal = a / math.sqrt(w)
    return np.array([math.radians(fix.longitude - lon0) * normal * math.cos(phi),
                     math.radians(fix.latitude - lat0) * meridian,
                     fix.altitude - h0])


def rotate(q, v):
    """O vetor `v` girado pelo quatérnio `q` (geometry_msgs)."""
    u = np.array([q.x, q.y, q.z])
    return v + 2 * np.cross(u, np.cross(u, v) + q.w * v)


@pytest.fixture(scope='module')
def sim():
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)))
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    cam = '/tractor_1/camera_front'
    commands = [
        ['ros2', 'launch', 'agri_gz_bringup', 'convoy.launch.py', 'gui:=false',
         'followers:=1', f'spacing:={SPACING}', 'sensor_profile:=ideal'],
        ['ros2', 'run', 'image_proc', 'rectify_node', '--ros-args',
         '-r', f'image:={cam}/image_raw', '-r', f'camera_info:={cam}/camera_info',
         '-r', f'image_rect:={cam}/image_rect'],
        ['ros2', 'run', 'apriltag_ros', 'apriltag_node', '--ros-args',
         '-r', f'image_rect:={cam}/image_rect', '-r', f'camera_info:={cam}/camera_info',
         '-r', 'detections:=/tractor_1/detections',
         '-p', 'family:=36h11', '-p', 'size:=0.5'],
    ]
    processes = [subprocess.Popen(c, env=env, start_new_session=True,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                 for c in commands]
    rclpy.init()
    node = rclpy.create_node('test_sim_sensors')
    data = {}

    def keep(topic):
        return lambda msg: data.setdefault(topic, []).append(msg)

    for vehicle in ANTENNA:
        for suffix, kind in TYPES.items():
            topic = f'/{vehicle}/{suffix}'
            node.create_subscription(kind, topic, keep(topic), qos_profile_sensor_data)
        topic = f'/ground_truth/{vehicle}/pose'
        node.create_subscription(PoseStamped, topic, keep(topic), qos_profile_sensor_data)
    node.create_subscription(AprilTagDetectionArray, '/tractor_1/detections',
                             keep('/tractor_1/detections'), qos_profile_sensor_data)

    def enough():
        return all(len(data.get(t, [])) > 2
                   and stamp(data[t][-1]) - stamp(data[t][0]) >= SIM_SECONDS for t in RATES)

    start = time.monotonic()
    while not enough() and time.monotonic() - start < WALL_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.1)
    yield data
    node.destroy_node()
    rclpy.shutdown()
    for process in processes:
        os.killpg(process.pid, signal.SIGINT)
    for process in processes:
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


@pytest.mark.parametrize('topic', sorted(RATES))
def test_rates(sim, topic):
    messages = sim.get(topic, [])
    assert len(messages) > 2, f'{topic}: {len(messages)} mensagens'
    stamps = sorted(stamp(m) for m in messages)
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    assert rate == pytest.approx(RATES[topic], rel=0.05)


def test_camera_info(sim):
    info = sim['/tractor_1/camera_front/camera_info'][-1]
    assert info.distortion_model == 'plumb_bob'
    assert list(info.d) == pytest.approx([-0.10, 0.01, 0.0, 0.0, 0.0], abs=1e-9)
    assert (info.width, info.height) == (1280, 720)
    focal = 640 / math.tan(math.radians(120) / 2)
    assert info.k[0] == pytest.approx(focal, abs=0.5)


@pytest.mark.parametrize('vehicle', sorted(ANTENNA))
def test_fix_is_the_antenna(sim, vehicle):
    fix = sim[f'/{vehicle}/gnss/fix'][-1]
    truth = sim[f'/ground_truth/{vehicle}/pose'][-1].pose
    position = np.array([truth.position.x, truth.position.y, truth.position.z])
    antenna = position + rotate(truth.orientation, np.array(ANTENNA[vehicle]))
    local = to_local(fix, world_origin())
    assert np.linalg.norm(local[:2] - antenna[:2]) < 0.02
    assert abs(local[2] - antenna[2]) < 0.05


def test_ground_truth_of_every_vehicle(sim):
    for vehicle, x in (('pickup', 0.0), ('tractor_1', -SPACING)):
        poses = sim.get(f'/ground_truth/{vehicle}/pose', [])
        assert poses, vehicle
        position = poses[-1].pose.position
        assert (position.x, position.y) == pytest.approx((x, 0.0), abs=0.3)


def test_tractor_sees_the_tag_of_the_pickup(sim):
    ids = {d.id for message in sim.get('/tractor_1/detections', []) for d in message.detections}
    assert 0 in ids
