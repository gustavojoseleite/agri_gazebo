"""Teste da perda na câmera simulada (F1.3, R2, R3 e R5), com `rclpy`, sem o Gazebo.

O `camera_distortion` é o nó que gera a imagem da câmera, então é ele que degrada: tem os mesmos
parâmetros `loss.*` do `SensorNode` e publica o estado em `degradation/state`. Com a perda
desligada, a imagem passa; ligada (gatilho sempre), não sai imagem nenhuma.
"""
import os
import random
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from agri_gz_bringup.camera_distortion import CameraDistortion


@pytest.fixture
def rig():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/v'])
    executor = SingleThreadedExecutor()
    camera = CameraDistortion()
    probe = rclpy.create_node('probe')
    received, states = [], []
    probe.create_subscription(Image, 'camera_front/image_raw', received.append, 10)
    probe.create_subscription(DiagnosticArray, 'degradation/state', states.append, 10)
    info = probe.create_publisher(CameraInfo, 'camera_front/camera_info', 10)
    ideal = probe.create_publisher(Image, '_sim/camera_front/image', 10)
    for node in (camera, probe):
        executor.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)

    def send(sec):
        """Publica o `camera_info` sem distorção e uma imagem ideal pequena."""
        calibration = CameraInfo(width=4, height=2, k=[1.0, 0.0, 2.0, 0.0, 1.0, 1.0, 0.0, 0.0,
                                                       1.0], d=[0.0] * 5)
        info.publish(calibration)
        spin(0.2)
        image = Image(width=4, height=2, encoding='mono8', step=4, data=bytes(range(8)))
        image.header.stamp = Time(sec=sec)
        ideal.publish(image)
        spin(0.3)

    spin(1.0)
    yield camera, received, states, send, spin
    executor.shutdown()
    camera.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def test_image_passes_by_default(rig):
    _, received, _, send, _ = rig
    send(1)
    assert [m.header.stamp.sec for m in received] == [1]
    assert bytes(received[0].data) == bytes(range(8))


def test_loss_drops_the_image(rig):
    camera, received, _, send, _ = rig
    camera.set_parameters([Parameter('loss.enabled', value=True),
                           Parameter('loss.trigger', value='always')])
    send(1)
    camera.set_parameters([Parameter('loss.enabled', value=False)])
    send(2)
    assert [m.header.stamp.sec for m in received] == [2]


def test_camera_state_is_published(rig):
    camera, _, states, send, spin = rig
    camera.set_parameters([Parameter('loss.enabled', value=True)])
    send(1)
    spin(1.5)
    entries = [s for message in states for s in message.status if 'camera' in s.name]
    assert entries
    assert {v.key: v.value for v in entries[-1].values}['dropped'] == '1'
