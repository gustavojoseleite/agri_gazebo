"""Testes dos efeitos da câmera no nó que gera a imagem (F1.4c, R3 e R4), com `rclpy`.

O `camera_distortion` declara, além do `loss.*`, os efeitos da `CameraSensor`, cada um com
`<e>.enabled`, `<e>.trigger`, `<e>.window_start`, `<e>.window_end`, `<e>.zone` e os valores com
faixa: `occlusion.fraction` (0 a 1), `dust.contrast` (0 a 1), `dust.blur` (0 a 10 px),
`brightness.gain` (0 a 3, padrão 1) e `brightness.offset` (−128 a 128). Os efeitos de valor dos
outros sensores (`noise`, `scale`...) não existem na câmera. O `profile` escolhe um perfil da
câmera; a aplicação em execução leva alguns ciclos.
"""
import os
import random
import time

import cv2
import numpy as np
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

from agri_gz_bringup.camera_distortion import CameraDistortion
from agri_sensor_degradation.image import MUD

W, H, SIDE, X, Y = 640, 480, 160, 200, 120


def tag_image():
    # Fundo cinza, e não branco, para o ganho do brilho ter o que clarear.
    gray = np.full((H, W), 160, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gray[Y:Y + SIDE, X:X + SIDE] = cv2.aruco.drawMarker(dictionary, 1, SIDE, borderBits=1)
    return np.dstack([gray] * 3)


@pytest.fixture
def rig():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init(args=['--ros-args', '-r', '__ns:=/v'])
    executor = SingleThreadedExecutor()
    camera = CameraDistortion()
    probe = rclpy.create_node('probe')
    received = []
    probe.create_subscription(Image, 'camera_front/image_raw', received.append, 10)
    info = probe.create_publisher(CameraInfo, 'camera_front/camera_info', 10)
    ideal = probe.create_publisher(Image, 'camera_front/image_ideal', 10)
    for node in (camera, probe):
        executor.add_node(node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)

    def send():
        """Publica o `camera_info` sem distorção e a imagem com a tag; devolve a recebida."""
        info.publish(CameraInfo(width=W, height=H, d=[0.0] * 5,
                                k=[300.0, 0.0, W / 2, 0.0, 300.0, H / 2, 0.0, 0.0, 1.0]))
        spin(0.2)
        image = tag_image()
        ideal.publish(Image(width=W, height=H, encoding='rgb8', step=W * 3,
                            data=image.tobytes()))
        received.clear()
        end = time.monotonic() + 3.0
        while not received and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
        assert received, 'o nó não publicou a imagem'
        return np.frombuffer(bytes(received[-1].data), np.uint8).reshape(H, W, 3), image

    spin(1.0)
    yield camera, send, spin
    executor.shutdown()
    camera.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def floating_range(node, name):
    (r,) = node.describe_parameter(name).floating_point_range
    return r.from_value, r.to_value


def test_camera_effect_parameters(rig):
    camera = rig[0]
    for name in ('occlusion', 'dust', 'brightness'):
        assert camera.get_parameter(f'{name}.enabled').value is False
        assert camera.get_parameter(f'{name}.trigger').value == 'always'
    assert floating_range(camera, 'occlusion.fraction') == pytest.approx((0.0, 1.0))
    assert floating_range(camera, 'dust.contrast') == pytest.approx((0.0, 1.0))
    assert floating_range(camera, 'dust.blur') == pytest.approx((0.0, 10.0))
    assert floating_range(camera, 'brightness.gain') == pytest.approx((0.0, 3.0))
    assert camera.get_parameter('brightness.gain').value == pytest.approx(1.0)
    assert floating_range(camera, 'brightness.offset') == pytest.approx((-128.0, 128.0))
    assert camera.get_parameter('profile').value == ''
    assert not camera.has_parameter('noise.enabled')
    assert not camera.has_parameter('scale.enabled')


def test_image_passes_unchanged_by_default(rig):
    _, send, _ = rig
    out, image = send()
    assert np.array_equal(out, image)


def test_occlusion_profile_covers_half_of_the_tag(rig):
    camera, send, spin = rig
    assert camera.set_parameters([Parameter('profile', value='occlusion_50')])[0].successful
    spin(0.5)
    assert camera.get_parameter('occlusion.enabled').value is True
    out, _ = send()
    mud = np.all(out == MUD, axis=2)
    assert mud[Y:Y + SIDE, X:X + SIDE].mean() == pytest.approx(0.5, abs=0.06)


def test_glare_then_back_to_ideal(rig):
    camera, send, spin = rig
    camera.set_parameters([Parameter('profile', value='glare')])
    spin(0.5)
    out, image = send()
    assert out.mean() > image.mean() + 5.0
    camera.set_parameters([Parameter('profile', value='')])
    spin(0.5)
    out, image = send()
    assert np.array_equal(out, image)


def test_unknown_camera_profile_is_rejected(rig):
    camera = rig[0]
    assert not camera.set_parameters([Parameter('profile', value='rtk_float')])[0].successful
