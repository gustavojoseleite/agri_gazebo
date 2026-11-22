"""Testes dos parâmetros novos e do perfil por nome no nó do sensor (F1.4b, R5), com `rclpy`.

- `markov.<grandeza>` (0 ao fundo de escala) e `markov.tau` (1 a 3600 s, padrão 60);
- `jumps.min_fraction` (0 a 1, padrão 1) e `jumps.duration_min` (0 a 30 s, padrão 0);
- na odometria, `slip.rate` (0 a 1), `slip.duration` (0 a 30) e `slip.fraction` (0 a 1).

`profile` (padrão vazio): escolher um perfil, no launch ou em execução, volta os efeitos aos
padrões e grava os valores do perfil nos parâmetros. Vazio desliga os efeitos. Um nome
desconhecido é recusado. O `loss`, a semente e o `vertical_factor` não mudam. A aplicação em
execução pode levar alguns ciclos, por isso o teste espera um pouco.
"""
import math
import os
import random
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from agri_sensor_degradation import profiles
from agri_sensor_degradation.node import SensorNode
from agri_sensor_degradation.sensors import GnssSensor, ImuSensor, OdometrySensor


@pytest.fixture
def ros():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init()
    made = []

    def make(sensor, *overrides):
        node = SensorNode(sensor, namespace='/v', parameter_overrides=list(overrides))
        made.append(node)
        return node

    yield make
    for node in made:
        node.destroy_node()
    rclpy.shutdown()


def settle(node, condition, seconds=3.0):
    """Gira o nó até a condição valer ou o tempo acabar; devolve a condição."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    end = time.monotonic() + seconds
    while time.monotonic() < end and not condition():
        executor.spin_once(timeout_sec=0.05)
    executor.remove_node(node)
    return condition()


def value(node, name):
    return node.get_parameter(name).value


def floating_range(node, name):
    (r,) = node.describe_parameter(name).floating_point_range
    return r.from_value, r.to_value


def test_new_parameters(ros):
    gnss = ros(GnssSensor(seed=0))
    assert floating_range(gnss, 'markov.position') == pytest.approx((0.0, 5.0))
    assert floating_range(gnss, 'markov.tau') == pytest.approx((1.0, 3600.0))
    assert value(gnss, 'markov.tau') == pytest.approx(60.0)
    assert floating_range(gnss, 'jumps.min_fraction') == pytest.approx((0.0, 1.0))
    assert value(gnss, 'jumps.min_fraction') == pytest.approx(1.0)
    assert floating_range(gnss, 'jumps.duration_min') == pytest.approx((0.0, 30.0))
    assert value(gnss, 'profile') == ''
    assert not gnss.has_parameter('slip.rate')
    odometry = ros(OdometrySensor(seed=0))
    assert value(odometry, 'slip.enabled') is False
    for name, high in (('slip.rate', 1.0), ('slip.duration', 30.0), ('slip.fraction', 1.0)):
        assert floating_range(odometry, name) == pytest.approx((0.0, high))


def test_profile_at_runtime_and_back_to_ideal(ros):
    gnss = ros(GnssSensor(seed=0))
    assert gnss.set_parameters([Parameter('profile', value='rtk_float')])[0].successful
    assert settle(gnss, lambda: value(gnss, 'markov.enabled') is True)
    assert value(gnss, 'markov.position') == pytest.approx(0.3 / math.sqrt(2), rel=1e-3)
    assert value(gnss, 'jumps.duration_min') == pytest.approx(1.0)
    assert gnss.sensor.effects['markov'].enabled
    assert gnss.set_parameters([Parameter('profile', value='')])[0].successful
    assert settle(gnss, lambda: value(gnss, 'markov.enabled') is False)
    assert value(gnss, 'markov.position') == pytest.approx(0.0)
    assert value(gnss, 'jumps.enabled') is False
    assert value(gnss, 'jumps.duration_min') == pytest.approx(0.0)
    assert not gnss.sensor.effects['jumps'].enabled


def test_switching_profiles_resets_first(ros):
    gnss = ros(GnssSensor(seed=0))
    gnss.set_parameters([Parameter('noise.position', value=1.0),
                         Parameter('noise.enabled', value=True)])
    gnss.set_parameters([Parameter('profile', value='dgnss')])
    assert settle(gnss, lambda: value(gnss, 'markov.enabled') is True)
    assert value(gnss, 'markov.position') == pytest.approx(0.7 / math.sqrt(2), rel=1e-3)
    assert value(gnss, 'noise.enabled') is False
    assert value(gnss, 'noise.position') == pytest.approx(0.0)


def test_unknown_profile_is_rejected(ros):
    gnss = ros(GnssSensor(seed=0))
    assert not gnss.set_parameters([Parameter('profile', value='survey_grade')])[0].successful
    assert value(gnss, 'profile') == ''


def test_profile_at_launch(ros):
    imu = ros(ImuSensor(seed=0), Parameter('profile', value='mems'))
    assert settle(imu, lambda: value(imu, 'markov.enabled') is True)
    assert value(imu, 'noise.gyro') == pytest.approx(0.005)
    assert imu.sensor.effects['noise'].enabled


def test_profile_leaves_loss_seed_and_vertical_factor(ros):
    gnss = ros(GnssSensor(seed=0))
    gnss.set_parameters([Parameter('loss.enabled', value=True),
                         Parameter('vertical_factor', value=3.0),
                         Parameter('seed', value=7)])
    gnss.set_parameters([Parameter('profile', value='rtk_float')])
    assert settle(gnss, lambda: value(gnss, 'markov.enabled') is True)
    assert value(gnss, 'loss.enabled') is True
    assert value(gnss, 'vertical_factor') == pytest.approx(3.0)
    assert value(gnss, 'seed') == 7


@pytest.mark.parametrize('cls', [GnssSensor, ImuSensor, OdometrySensor])
def test_every_profile_value_is_a_parameter_of_its_node(ros, cls):
    node = ros(cls(seed=0))
    for name in profiles.names(cls.name):
        for key in profiles.load(cls.name, name):
            assert node.has_parameter(key), (name, key)
