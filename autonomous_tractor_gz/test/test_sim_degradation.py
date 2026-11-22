"""Teste de simulação da degradação no comboio (F1.3, R2, R4 e R5). Marca `simulation`.

Sobe o comboio sem GUI com a pickup e um trator. A cadeia é Gazebo → ponte (tópico ideal) → nó do
sensor (tópico normal). Confere:
- o GNSS da pickup e a odometria do trator chegam aos tópicos normais;
- os nós `gnss`, `imu` e `odometry` de cada veículo existem;
- ligar a perda do GNSS da pickup pelo serviço de parâmetros corta o `gnss/fix`, e desligar a faz
  voltar.
"""
import os
import random
import signal
import subprocess
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

pytestmark = pytest.mark.simulation

WALL_TIMEOUT = 240.0


@pytest.fixture(scope='module')
def sim():
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)))
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    process = subprocess.Popen(['ros2', 'launch', 'agri_gz_bringup', 'convoy.launch.py',
                                'gui:=false', 'followers:=1'],
                               env=env, start_new_session=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('test_sim_degradation')
    data = {'fix': [], 'odom': []}
    node.create_subscription(NavSatFix, '/pickup/gnss/fix', data['fix'].append,
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/tractor_1/odom', data['odom'].append,
                             qos_profile_sensor_data)
    start = time.monotonic()
    while not (data['fix'] and data['odom']) and time.monotonic() - start < WALL_TIMEOUT:
        rclpy.spin_once(node, timeout_sec=0.2)
    yield node, data
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)


def spin(node, seconds):
    """Gira o nó por `seconds` de relógio."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def test_signals_reach_the_normal_topics(sim):
    node, data = sim
    assert data['fix'], 'sem /pickup/gnss/fix'
    assert data['odom'], 'sem /tractor_1/odom'
    names = {f'{ns.rstrip("/")}/{n}' for n, ns in node.get_node_names_and_namespaces()}
    for vehicle in ('pickup', 'tractor_1'):
        for sensor in ('gnss', 'imu', 'odometry'):
            assert f'/{vehicle}/{sensor}' in names


def test_gnss_loss_by_parameter_service(sim):
    node, data = sim
    client = node.create_client(SetParameters, '/pickup/gnss/set_parameters')
    assert client.wait_for_service(timeout_sec=10.0)

    def set_loss(value):
        request = SetParameters.Request()
        request.parameters = [Parameter('loss.enabled', value=value).to_parameter_msg()]
        future = client.call_async(request)
        end = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert future.done() and future.result().results[0].successful

    set_loss(True)
    spin(node, 2.0)
    data['fix'].clear()
    spin(node, 6.0)
    assert data['fix'] == []
    set_loss(False)
    spin(node, 6.0)
    assert data['fix']
