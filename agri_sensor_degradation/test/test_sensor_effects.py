"""Testes do que cada sensor degrada (F1.4a, R2 e R3), sem `rclpy`.

- GNSS: a grandeza `position` (leste, norte, altitude), em metros, a partir da primeira fixação;
  a altitude pesa `vertical_factor` vezes (padrão 2) nos efeitos aditivos.
- IMU: `gyro` e `accel`, 3 eixos cada; a orientação fica ideal.
- Odometria: `linear` (vx) e `angular` (ωz); com algum efeito ativo, a pose é integrada da
  velocidade degradada.
- Com ruído ativo, a covariância diz σ²; sem ruído, fica como veio. A mensagem de entrada nunca
  é alterada.
"""
import math

import numpy as np
import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

from agri_sensor_degradation.sensors import GnssSensor, ImuSensor, OdometrySensor

R = 6378137.0
LAT, LON, ALT = -20.9, -48.4, 571.5


def fix(east=0.0, sec=0):
    msg = NavSatFix()
    msg.header.stamp.sec = sec
    msg.latitude = LAT
    msg.longitude = LON + math.degrees(east / (R * math.cos(math.radians(LAT))))
    msg.altitude = ALT
    msg.position_covariance = [4.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0]
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
    return msg


def enu(msg):
    """Leste, norte e altura de uma fixação em relação a (LAT, LON, ALT), em metros."""
    east = math.radians(msg.longitude - LON) * R * math.cos(math.radians(LAT))
    north = math.radians(msg.latitude - LAT) * R
    return east, north, msg.altitude - ALT


def test_channels():
    assert set(GnssSensor.channels) == {'position'}
    assert GnssSensor.channels['position'].size == 3
    assert {k: c.size for k, c in ImuSensor.channels.items()} == {'gyro': 3, 'accel': 3}
    assert {k: c.size for k, c in OdometrySensor.channels.items()} == {'linear': 1, 'angular': 1}
    assert GnssSensor(seed=0).vertical_factor == pytest.approx(2.0)


def test_gnss_bias_moves_the_fix_in_meters():
    sensor = GnssSensor(seed=0)
    sensor.set_effect('bias', position=1.0)
    msg = fix()
    out = sensor.apply(msg, 0.0, None)
    assert enu(out) == pytest.approx((1.0, 1.0, 2.0), rel=0.01)
    assert (msg.latitude, msg.longitude, msg.altitude) == (LAT, fix().longitude, ALT)


def test_gnss_vertical_factor():
    sensor = GnssSensor(seed=0)
    sensor.vertical_factor = 3.0
    sensor.set_effect('bias', position=1.0)
    assert enu(sensor.apply(fix(), 0.0, None))[2] == pytest.approx(3.0, rel=0.01)


def test_gnss_scale_acts_on_the_displacement_from_the_first_fix():
    sensor = GnssSensor(seed=0)
    sensor.set_effect('scale', position=1.1)
    first = sensor.apply(fix(), 0.0, None)
    assert enu(first) == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    out = sensor.apply(fix(east=10.0, sec=1), 1.0, None)
    assert enu(out)[0] == pytest.approx(11.0, rel=0.01)


def test_gnss_noise_and_covariance():
    sensor = GnssSensor(seed=1)
    sensor.set_effect('noise', position=0.5)
    outs = [sensor.apply(fix(sec=k), float(k), None) for k in range(5000)]
    errors = np.array([enu(o) for o in outs])
    assert errors.std(axis=0) == pytest.approx([0.5, 0.5, 1.0], rel=0.05)
    cov = np.array(outs[0].position_covariance).reshape(3, 3)
    assert np.diag(cov) == pytest.approx([0.25, 0.25, 1.0])
    assert cov[~np.eye(3, dtype=bool)] == pytest.approx(np.zeros(6))
    assert outs[0].position_covariance_type == NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN


def test_gnss_without_noise_keeps_the_covariance():
    sensor = GnssSensor(seed=0)
    sensor.set_effect('bias', position=1.0)
    out = sensor.apply(fix(), 0.0, None)
    assert list(out.position_covariance) == pytest.approx([4.0, 0, 0, 0, 4.0, 0, 0, 0, 9.0])
    assert out.position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED


def imu(sec=0):
    msg = Imu()
    msg.header.stamp.sec = sec
    msg.orientation.w = 1.0
    msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
    msg.angular_velocity.z = 0.2
    msg.linear_acceleration.z = 9.81
    return msg


def vectors(msg):
    g, a = msg.angular_velocity, msg.linear_acceleration
    return [g.x, g.y, g.z], [a.x, a.y, a.z]


def test_imu_noise_and_covariance():
    sensor = ImuSensor(seed=1)
    sensor.set_effect('noise', gyro=0.005, accel=0.02)
    outs = [sensor.apply(imu(sec=k), float(k), None) for k in range(5000)]
    gyro = np.array([vectors(o)[0] for o in outs]) - [0.0, 0.0, 0.2]
    accel = np.array([vectors(o)[1] for o in outs]) - [0.0, 0.0, 9.81]
    assert gyro.std(axis=0) == pytest.approx([0.005] * 3, rel=0.05)
    assert accel.std(axis=0) == pytest.approx([0.02] * 3, rel=0.05)
    assert np.diag(np.reshape(outs[0].angular_velocity_covariance, (3, 3))) == pytest.approx(
        [0.005 ** 2] * 3)
    assert np.diag(np.reshape(outs[0].linear_acceleration_covariance, (3, 3))) == pytest.approx(
        [0.02 ** 2] * 3)
    assert outs[0].orientation.w == 1.0
    assert list(outs[0].orientation_covariance) == pytest.approx(
        [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01])


def test_imu_bias_only_on_the_gyro():
    sensor = ImuSensor(seed=0)
    sensor.set_effect('bias', gyro=0.01)
    gyro, accel = vectors(sensor.apply(imu(), 0.0, None))
    assert gyro == pytest.approx([0.01, 0.01, 0.21])
    assert accel == pytest.approx([0.0, 0.0, 9.81])


def odom(t, v=1.0, w=0.0):
    """Odometria ideal de quem anda em linha reta no eixo x a `v` m/s desde t = 0."""
    msg = Odometry()
    msg.header.stamp.sec = int(t)
    msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    msg.pose.pose.position.x = v * t
    msg.pose.pose.orientation.w = 1.0
    msg.twist.twist.linear.x = v
    msg.twist.twist.angular.z = w
    return msg


def yaw(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def drive(sensor, seconds, hz=10):
    out = None
    for k in range(round(seconds * hz) + 1):
        t = k / hz
        out = sensor.apply(odom(t), t, None)
    return out


def test_odometry_scale_stretches_the_path():
    sensor = OdometrySensor(seed=0)
    sensor.set_effect('scale', linear=1.02)
    out = drive(sensor, 100.0)
    assert out.twist.twist.linear.x == pytest.approx(1.02)
    assert out.pose.pose.position.x == pytest.approx(102.0, rel=0.005)


def test_odometry_angular_bias_turns_the_heading():
    sensor = OdometrySensor(seed=0)
    sensor.set_effect('bias', angular=0.01)
    out = drive(sensor, 100.0)
    assert out.twist.twist.angular.z == pytest.approx(0.01)
    assert yaw(out) == pytest.approx(1.0, rel=0.01)
    assert out.pose.pose.position.y > 10.0


def test_odometry_goes_back_to_the_ideal_pose():
    sensor = OdometrySensor(seed=0)
    sensor.set_effect('scale', linear=1.5)
    drive(sensor, 10.0)
    sensor.effects['scale'].enabled = False
    msg = odom(10.1)
    assert sensor.apply(msg, 10.1, None) is msg


def test_odometry_noise_covariance():
    sensor = OdometrySensor(seed=0)
    sensor.set_effect('noise', linear=0.1)
    out = sensor.apply(odom(0.0), 0.0, None)
    assert out.twist.covariance[0] == pytest.approx(0.01)
    assert out.twist.covariance[35] == pytest.approx(0.0)
