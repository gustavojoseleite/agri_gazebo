"""Testes da patinagem da odometria e dos perfis por nome (F1.4b, R3 e R4), sem `rclpy`.

`slip` é um efeito só da `OdometrySensor`: episódios sorteados (`rate` por segundo, `duration`)
em que a velocidade linear medida sai `(1 + fraction)` vezes a real.

`profiles.load(sensor, perfil)` devolve os valores de parâmetros de um perfil, lidos de
`config/profiles.yaml` (instalado no `share` do pacote); `profiles.names(sensor)` lista os perfis
de um sensor. O σh do GNSS é horizontal: cada eixo recebe σh/√2.
"""
import math

import numpy as np
import pytest
from nav_msgs.msg import Odometry

from agri_sensor_degradation import profiles
from agri_sensor_degradation.sensors import GnssSensor, ImuSensor, OdometrySensor


def odom(t, v=1.0):
    msg = Odometry()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    msg.pose.pose.position.x = v * t
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = v
    return msg


def test_slip_belongs_to_the_odometry_only():
    assert 'slip' in OdometrySensor(seed=0).effects
    assert set(OdometrySensor(seed=0).effects['slip'].values) == {'rate', 'duration', 'fraction'}
    assert 'slip' not in GnssSensor(seed=0).effects
    assert 'slip' not in ImuSensor(seed=0).effects


def test_slip_episodes():
    sensor = OdometrySensor(seed=2)
    sensor.set_effect('slip', rate=0.05, duration=2.0, fraction=0.1)
    speeds, out = [], None
    for k in range(50000):
        t = k / 10
        out = sensor.apply(odom(t), t, None)
        speeds.append(out.twist.twist.linear.x)
    speeds = np.array(speeds)
    slipping = np.isclose(speeds, 1.1)
    assert np.isclose(speeds[~slipping], 1.0).all()
    # Fração do tempo em patinagem: 2 / (1/0,05 + 2) = 0,091.
    assert 0.075 < slipping.mean() < 0.107
    assert out.pose.pose.position.x > 5000.0 + 30.0


def test_slip_state():
    sensor = OdometrySensor(seed=0)
    sensor.set_effect('slip', rate=0.05, fraction=0.1)
    sensor.apply(odom(0.0), 0.0, None)
    state = sensor.state()
    assert (state['slip_enabled'], state['slip_active']) == ('true', 'true')


def test_profile_names():
    assert profiles.names('gnss') == ['dgnss', 'rtk_float']
    assert profiles.names('imu') == ['mems']
    assert profiles.names('odometry') == ['field']
    assert profiles.names('camera') == []


def test_rtk_float_and_dgnss():
    rtk = profiles.load('gnss', 'rtk_float')
    assert rtk['markov.enabled'] is True
    assert rtk['markov.position'] == pytest.approx(0.3 / math.sqrt(2), rel=1e-3)
    assert rtk['markov.tau'] == pytest.approx(60.0)
    assert rtk['jumps.enabled'] is True
    assert rtk['jumps.position'] == pytest.approx(2.0)
    assert rtk['jumps.min_fraction'] == pytest.approx(0.25)
    assert (rtk['jumps.duration_min'], rtk['jumps.duration']) == pytest.approx((1.0, 3.0))
    assert rtk['jumps.rate'] == pytest.approx(1.0 / 120.0, rel=0.01)
    dgnss = profiles.load('gnss', 'dgnss')
    assert dgnss['markov.position'] == pytest.approx(0.7 / math.sqrt(2), rel=1e-3)
    assert {k: v for k, v in dgnss.items() if k != 'markov.position'} == {
        k: v for k, v in rtk.items() if k != 'markov.position'}


def test_mems():
    mems = profiles.load('imu', 'mems')
    assert mems['noise.enabled'] is True and mems['markov.enabled'] is True
    assert (mems['noise.gyro'], mems['noise.accel']) == pytest.approx((0.005, 0.02))
    assert mems['markov.gyro'] == pytest.approx(math.radians(10.0) / 3600.0, rel=0.01)
    assert mems['markov.tau'] == pytest.approx(300.0)


def test_field_odometry():
    field = profiles.load('odometry', 'field')
    assert field['scale.enabled'] is True and field['scale.linear'] == pytest.approx(1.02)
    assert field['slip.enabled'] is True
    assert field['slip.fraction'] == pytest.approx(0.1)
    assert field['slip.duration'] == pytest.approx(2.0)
    assert field['slip.rate'] == pytest.approx(1.0 / 60.0, rel=0.01)


def test_profiles_leave_loss_and_zones_out():
    for sensor in ('gnss', 'imu', 'odometry'):
        for name in profiles.names(sensor):
            keys = profiles.load(sensor, name)
            assert not any(k.startswith('loss.') or k.endswith('.zone') for k in keys)
            assert 'seed' not in keys and 'vertical_factor' not in keys


def test_unknown_profile():
    with pytest.raises(KeyError):
        profiles.load('gnss', 'survey_grade')
