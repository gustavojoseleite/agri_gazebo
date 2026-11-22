"""Testes do núcleo da degradação e das classes por sensor (F1.3, R1, R3 e R4).

O núcleo é Python puro, sem `rclpy`:
- `point_in_polygon(x, y, polygon)` diz se o ponto está dentro do polígono (lista de (x, y));
- `Trigger(mode, window, zone)`, com `mode` em 'always', 'window' ou 'zone', diz se está ativo
  no instante `t` (s) e na posição real (x, y) do veículo;
- `Loss(enabled, fraction, trigger)` decide se a mensagem se perde: com o gatilho ativo, perde
  sempre com `fraction` 1,0 e, abaixo disso, ao acaso, pelo gerador da semente;
- `SensorBase(seed)` aplica a perda: `apply(msg, t, position)` devolve a própria mensagem ou
  `None`, conta as recebidas e as perdidas e descreve o estado em `state()`.

As classes por sensor dizem o tipo da mensagem e os tópicos padrão: o ideal, que o simulador
publica, e o normal, que os outros nós consomem.
"""
import pytest
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, NavSatFix

from agri_sensor_degradation.core import Loss, point_in_polygon, SensorBase, Trigger
from agri_sensor_degradation.sensors import (CameraSensor, GnssSensor, ImuSensor,
                                             OdometrySensor, TagDetectionSensor)

SQUARE = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
L_SHAPE = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (4.0, 4.0), (4.0, 10.0), (0.0, 10.0)]


@pytest.mark.parametrize('polygon, point, inside', [
    (SQUARE, (5.0, 5.0), True), (SQUARE, (11.0, 5.0), False), (SQUARE, (-0.1, 3.0), False),
    (L_SHAPE, (2.0, 8.0), True), (L_SHAPE, (8.0, 2.0), True), (L_SHAPE, (8.0, 8.0), False),
])
def test_point_in_polygon(polygon, point, inside):
    assert point_in_polygon(point[0], point[1], polygon) == inside


def test_trigger_always():
    assert Trigger('always').active(123.0, None)


def test_trigger_window():
    trigger = Trigger('window', window=(10.0, 20.0))
    assert not trigger.active(9.99, (0.0, 0.0))
    assert trigger.active(10.0, (0.0, 0.0))
    assert trigger.active(19.99, None)
    assert not trigger.active(20.0, (0.0, 0.0))


def test_trigger_zone():
    trigger = Trigger('zone', zone=SQUARE)
    assert trigger.active(0.0, (5.0, 5.0))
    assert not trigger.active(0.0, (15.0, 5.0))
    assert not trigger.active(0.0, None)


def test_unknown_trigger_mode():
    with pytest.raises(ValueError):
        Trigger('sometimes')


def test_disabled_loss_never_drops():
    sensor = SensorBase(seed=1)
    sensor.loss = Loss(enabled=False, fraction=1.0, trigger=Trigger('always'))
    assert all(sensor.apply(object(), float(t), None) is not None for t in range(100))


def test_full_loss_drops_while_triggered():
    sensor = SensorBase(seed=1)
    sensor.loss = Loss(enabled=True, fraction=1.0, trigger=Trigger('window', window=(5.0, 8.0)))
    kept = [t for t in range(12) if sensor.apply(object(), float(t), None) is not None]
    assert kept == [0, 1, 2, 3, 4, 8, 9, 10, 11]
    assert (sensor.received, sensor.dropped) == (12, 3)


def test_apply_returns_the_same_message():
    sensor = SensorBase(seed=1)
    message = NavSatFix()
    assert sensor.apply(message, 0.0, None) is message


def drops(seed, fraction, count=2000):
    """A sequência de perdas de `count` mensagens com o gatilho sempre ativo."""
    sensor = SensorBase(seed=seed)
    sensor.loss = Loss(enabled=True, fraction=fraction, trigger=Trigger('always'))
    return [sensor.apply(object(), float(t), None) is None for t in range(count)]


def test_fraction_with_the_same_seed_gives_the_same_result():
    assert drops(7, 0.3) == drops(7, 0.3)
    assert drops(7, 0.3) != drops(8, 0.3)


@pytest.mark.parametrize('fraction', [0.0, 0.1, 0.5, 0.9])
def test_fraction_is_the_drop_rate(fraction):
    rate = sum(drops(3, fraction, 10000)) / 10000
    assert rate == pytest.approx(fraction, abs=0.02)


def test_reseed_restarts_the_sequence():
    sensor = SensorBase(seed=5)
    sensor.loss = Loss(enabled=True, fraction=0.5, trigger=Trigger('always'))
    first = [sensor.apply(object(), 0.0, None) is None for _ in range(50)]
    sensor.reseed(5)
    assert [sensor.apply(object(), 0.0, None) is None for _ in range(50)] == first


def test_state():
    sensor = SensorBase(seed=1)
    sensor.loss = Loss(enabled=True, fraction=1.0, trigger=Trigger('zone', zone=SQUARE))
    sensor.apply(object(), 0.0, (5.0, 5.0))
    state = sensor.state()
    assert state['loss_enabled'] == 'true'
    assert state['loss_active'] == 'true'
    assert state['trigger'] == 'zone'
    assert state['fraction'] == '1.0'
    assert (state['received'], state['dropped']) == ('1', '1')


@pytest.mark.parametrize('cls, name, msg_type, ideal, normal', [
    (GnssSensor, 'gnss', NavSatFix, '_sim/gnss/fix', 'gnss/fix'),
    (ImuSensor, 'imu', Imu, '_sim/imu/data', 'imu/data'),
    (OdometrySensor, 'odometry', Odometry, '_sim/odom', 'odom'),
    (CameraSensor, 'camera', Image, '_sim/camera_front/image', 'camera_front/image_raw'),
    (TagDetectionSensor, 'tag_detection', AprilTagDetectionArray, '_sim/detections',
     'detections'),
])
def test_sensor_classes(cls, name, msg_type, ideal, normal):
    sensor = cls(seed=0)
    assert isinstance(sensor, SensorBase)
    assert (sensor.name, sensor.msg_type) == (name, msg_type)
    assert (sensor.input_topic, sensor.output_topic) == (ideal, normal)
    message = msg_type()
    assert sensor.apply(message, 0.0, None) is message
