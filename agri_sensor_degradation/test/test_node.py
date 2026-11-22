"""Testes do nó gerador de um sensor (F1.3, R2, R4 e R5), com `rclpy`, sem o Gazebo.

`SensorNode(sensor, namespace=...)` é o nó de um sensor: assina o tópico ideal, publica o normal
e só degrada quando os parâmetros mandam. Parâmetros:
- `seed` (int);
- `loss.enabled` (bool);
- `loss.fraction` (double, de 0 a 1; a faixa declarada vira slider no `rqt_reconfigure`);
- `loss.trigger` ('always', 'window' ou 'zone');
- `loss.window_start` e `loss.window_end` (s);
- `loss.zone` (lista plana x0, y0, x1, y1, ...);
- `ground_truth_topic`, de onde vem a posição real do veículo.

O instante de cada mensagem é o carimbo dela. O estado sai em `degradation/state`
(`diagnostic_msgs/DiagnosticArray`), com uma entrada cujo nome tem o do nó.
"""
import os
import random
import time

import pytest
import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix

from agri_sensor_degradation.node import SensorNode
from agri_sensor_degradation.sensors import GnssSensor

ZONE = [0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0]


@pytest.fixture
def rig():
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(30, 99))
    rclpy.init()
    executor = SingleThreadedExecutor()
    sensor = SensorNode(GnssSensor(seed=0), namespace='/v', parameter_overrides=[
        Parameter('ground_truth_topic', value='/ground_truth/v/pose')])
    probe = rclpy.create_node('probe')
    received, states = [], []
    probe.create_subscription(NavSatFix, '/v/gnss/fix', received.append, 10)
    probe.create_subscription(DiagnosticArray, '/v/degradation/state', states.append, 10)
    ideal = probe.create_publisher(NavSatFix, '/v/gnss/fix_ideal', 10)
    truth = probe.create_publisher(PoseStamped, '/ground_truth/v/pose', 10)
    for node in (sensor, probe):
        executor.add_node(node)

    def spin(seconds=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)

    def send(sec, latitude=-20.9, x=None):
        """Publica a posição real (se dada) e um fix ideal com o carimbo `sec`."""
        if x is not None:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = x, 5.0
            truth.publish(pose)
            spin(0.2)
        fix = NavSatFix()
        fix.header.stamp = Time(sec=sec)
        fix.header.frame_id = 'gnss_link'
        fix.latitude, fix.longitude, fix.altitude = latitude, -48.4, 571.5
        ideal.publish(fix)
        spin(0.3)

    spin(1.0)
    yield sensor, probe, received, states, send, spin
    executor.shutdown()
    sensor.destroy_node()
    probe.destroy_node()
    rclpy.shutdown()


def test_passes_the_signal_unchanged_by_default(rig):
    _, _, received, _, send, _ = rig
    send(3, latitude=-20.95)
    assert len(received) == 1
    fix = received[0]
    assert (fix.header.stamp.sec, fix.header.frame_id) == (3, 'gnss_link')
    assert (fix.latitude, fix.longitude, fix.altitude) == (-20.95, -48.4, 571.5)


def test_zone_loss_drops_inside_and_passes_outside(rig):
    sensor, _, received, _, send, _ = rig
    sensor.set_parameters([Parameter('loss.enabled', value=True),
                           Parameter('loss.trigger', value='zone'),
                           Parameter('loss.zone', value=ZONE)])
    send(1, x=5.0)
    assert received == []
    send(2, x=20.0)
    assert [f.header.stamp.sec for f in received] == [2]


def test_window_loss(rig):
    sensor, _, received, _, send, _ = rig
    sensor.set_parameters([Parameter('loss.enabled', value=True),
                           Parameter('loss.trigger', value='window'),
                           Parameter('loss.window_start', value=10.0),
                           Parameter('loss.window_end', value=20.0)])
    for sec in (5, 12, 25):
        send(sec)
    assert [f.header.stamp.sec for f in received] == [5, 25]


def test_parameter_service_switches_the_loss(rig):
    _, probe, received, _, send, spin = rig
    client = probe.create_client(SetParameters, '/v/gnss/set_parameters')
    assert client.wait_for_service(timeout_sec=5.0)

    def set_enabled(value):
        request = SetParameters.Request()
        request.parameters = [Parameter('loss.enabled', value=value).to_parameter_msg()]
        future = client.call_async(request)
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            spin(0.1)
        assert future.done() and future.result().results[0].successful

    set_enabled(True)
    send(1)
    set_enabled(False)
    send(2)
    assert [f.header.stamp.sec for f in received] == [2]


def test_fraction_is_a_slider(rig):
    sensor = rig[0]
    descriptor = sensor.describe_parameter('loss.fraction')
    assert len(descriptor.floating_point_range) == 1
    span = descriptor.floating_point_range[0]
    assert (span.from_value, span.to_value) == (0.0, 1.0)


def test_invalid_fraction_is_rejected(rig):
    sensor = rig[0]
    result = sensor.set_parameters([Parameter('loss.fraction', value=1.5)])[0]
    assert not result.successful


def test_state_is_published(rig):
    sensor, _, _, states, send, spin = rig
    sensor.set_parameters([Parameter('loss.enabled', value=True)])
    send(1)
    spin(1.5)
    entries = [s for message in states for s in message.status if 'gnss' in s.name]
    assert entries
    values = {v.key: v.value for v in entries[-1].values}
    assert values['loss_enabled'] == 'true'
    assert values['dropped'] == '1'
