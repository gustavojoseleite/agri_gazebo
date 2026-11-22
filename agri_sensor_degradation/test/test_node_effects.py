"""Testes dos parâmetros dos efeitos no nó do sensor (F1.4a, R4), com `rclpy`, sem o Gazebo.

Para cada efeito `<e>` há `<e>.enabled`, `<e>.trigger`, `<e>.window_start`, `<e>.window_end` e
`<e>.zone`, como no `loss`. Os valores são `<e>.<grandeza>` (ex.: `noise.gyro`), mais
`jumps.rate`, `jumps.duration`, `rate.max_hz`, `delay.seconds` e, no GNSS, `vertical_factor`.
Todo valor tem faixa declarada, que vira slider no `rqt_reconfigure`. As faixas saem do fundo de
escala de cada grandeza: ruído de 0 ao fundo de escala; escala de 0,5 a 1,5.
"""
import os
import random
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from agri_sensor_degradation.core import EFFECTS
from agri_sensor_degradation.node import SensorNode
from agri_sensor_degradation.sensors import GnssSensor, ImuSensor


@pytest.fixture
def rig():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init()
    executor = SingleThreadedExecutor()
    sensor = SensorNode(ImuSensor(seed=0), namespace='/v')
    probe = rclpy.create_node('probe')
    received, states = [], []
    probe.create_subscription(Imu, '/v/imu/data', received.append, 10)
    probe.create_subscription(DiagnosticArray, '/v/degradation/state', states.append, 10)
    ideal = probe.create_publisher(Imu, '/v/_sim/imu/data', 10)
    for node in (sensor, probe):
        executor.add_node(node)

    def spin(seconds=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)

    def send(count=1):
        """Publica `count` mensagens ideais carimbadas com a hora do nó."""
        for _ in range(count):
            msg = Imu()
            msg.header.stamp = sensor.get_clock().now().to_msg()
            msg.orientation.w = 1.0
            msg.angular_velocity.z = 0.2
            ideal.publish(msg)
            spin(0.05)

    spin(1.0)
    yield sensor, received, states, send, spin
    executor.shutdown()
    sensor.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def floating_range(node, name):
    ranges = node.describe_parameter(name).floating_point_range
    assert len(ranges) == 1, name
    return ranges[0].from_value, ranges[0].to_value


def test_effect_parameters_are_declared(rig):
    sensor = rig[0]
    for name in EFFECTS:
        assert sensor.get_parameter(f'{name}.enabled').value is False
        assert sensor.get_parameter(f'{name}.trigger').value == 'always'
        assert sensor.has_parameter(f'{name}.window_start')
        assert sensor.has_parameter(f'{name}.window_end')
        assert sensor.has_parameter(f'{name}.zone')
    assert floating_range(sensor, 'noise.gyro') == pytest.approx((0.0, 0.1))
    assert floating_range(sensor, 'noise.accel') == pytest.approx((0.0, 1.0))
    assert floating_range(sensor, 'scale.gyro') == pytest.approx((0.5, 1.5))
    assert sensor.get_parameter('scale.gyro').value == pytest.approx(1.0)
    for name in ('bias.gyro', 'drift.accel', 'jumps.gyro', 'jumps.rate', 'jumps.duration',
                 'rate.max_hz', 'delay.seconds'):
        low, high = floating_range(sensor, name)
        assert high > low, name
    assert not sensor.has_parameter('vertical_factor')
    assert not sensor.has_parameter('noise.position')


def test_noise_switched_on_and_off_at_runtime(rig):
    sensor, received, _, send, spin = rig
    send(5)
    assert received and all(m.angular_velocity.z == 0.2 for m in received)
    assert sensor.set_parameters([Parameter('noise.gyro', value=0.05),
                                  Parameter('noise.enabled', value=True)])[-1].successful
    received.clear()
    send(10)
    assert len({m.angular_velocity.z for m in received}) > 5
    assert received[-1].angular_velocity_covariance[0] == pytest.approx(0.0025)
    sensor.set_parameters([Parameter('noise.enabled', value=False)])
    spin(0.2)
    received.clear()
    send(5)
    assert received and all(m.angular_velocity.z == 0.2 for m in received)


def test_out_of_range_and_bad_trigger_are_rejected(rig):
    sensor = rig[0]
    assert not sensor.set_parameters([Parameter('noise.gyro', value=5.0)])[0].successful
    assert not sensor.set_parameters([Parameter('noise.trigger', value='sometimes')])[0].successful
    assert sensor.get_parameter('noise.trigger').value == 'always'


def test_delay_releases_the_message_later(rig):
    sensor, received, _, send, spin = rig
    sensor.set_parameters([Parameter('delay.seconds', value=0.6),
                           Parameter('delay.enabled', value=True)])
    spin(0.2)
    received.clear()
    send(1)
    spin(0.2)
    assert received == []
    spin(1.0)
    assert len(received) == 1


def test_state_reports_the_effects(rig):
    sensor, _, states, send, spin = rig
    sensor.set_parameters([Parameter('bias.gyro', value=0.01),
                           Parameter('bias.enabled', value=True)])
    send(2)
    spin(1.5)
    entries = [s for message in states for s in message.status if 'imu' in s.name]
    assert entries
    values = {v.key: v.value for v in entries[-1].values}
    assert (values['bias_enabled'], values['bias_active']) == ('true', 'true')
    assert values['noise_enabled'] == 'false'


def test_gnss_node_has_the_vertical_factor():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init()
    try:
        node = SensorNode(GnssSensor(seed=0), namespace='/v')
        assert node.get_parameter('vertical_factor').value == pytest.approx(2.0)
        assert floating_range(node, 'vertical_factor') == pytest.approx((1.0, 5.0))
        assert floating_range(node, 'noise.position') == pytest.approx((0.0, 5.0))
        node.set_parameters([Parameter('vertical_factor', value=3.0)])
        assert node.sensor.vertical_factor == pytest.approx(3.0)
        node.destroy_node()
    finally:
        rclpy.shutdown()
