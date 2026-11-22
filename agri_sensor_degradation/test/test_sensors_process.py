"""Teste do executável `sensors` (F1.3, R2 e R5).

`ros2 run agri_sensor_degradation sensors --ros-args -r __ns:=/v` sobe, num só processo, os nós
`gnss`, `imu` e `odometry` do veículo. Cada um publica no tópico normal o que recebe no ideal; com
a degradação desligada, o sinal passa igual.
"""
import os
import random
import signal
import subprocess
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


@pytest.fixture(scope='module')
def process():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    child = subprocess.Popen(['ros2', 'run', 'agri_sensor_degradation', 'sensors',
                              '--ros-args', '-r', '__ns:=/v'],
                             start_new_session=True, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('probe_sensors')
    yield node
    node.destroy_node()
    rclpy.shutdown()
    os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)


def wait_for(node, condition, seconds=20.0):
    """Gira o nó até a condição valer ou o prazo acabar."""
    end = time.monotonic() + seconds
    while not condition() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return condition()


def test_three_nodes_in_one_process(process):
    def names():
        return {f'{ns.rstrip("/")}/{n}' for n, ns in process.get_node_names_and_namespaces()}
    assert wait_for(process, lambda: {'/v/gnss', '/v/imu', '/v/odometry'} <= names()), names()


@pytest.mark.parametrize('kind, ideal, normal', [
    (Imu, '/v/imu/data_ideal', '/v/imu/data'),
    (Odometry, '/v/odom_ideal', '/v/odom'),
])
def test_passes_the_signal(process, kind, ideal, normal):
    received = []
    process.create_subscription(kind, normal, received.append, 10)
    publisher = process.create_publisher(kind, ideal, 10)
    assert wait_for(process, lambda: process.count_subscribers(ideal) > 0)
    message = kind()
    message.header.stamp = Time(sec=7)
    message.header.frame_id = 'base_link'
    assert wait_for(process, lambda: received or publisher.publish(message), 10.0)
    assert received[0].header.stamp.sec == 7
    assert received[0].header.frame_id == 'base_link'
