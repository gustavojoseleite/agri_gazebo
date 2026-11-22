"""Uma classe por sensor do simulador: o tipo da mensagem e os tópicos padrão.

O tópico de entrada é o ideal, que o simulador publica; o de saída é o normal, que os
outros nós consomem.
"""
import copy
import math

import numpy as np
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, NavSatFix

from agri_sensor_degradation.core import Channel, SensorBase

EARTH_RADIUS = 6378137.0


def _diagonal(sigma):
    """Covariância 3x3 (lista de 9) diagonal com σ² de cada eixo."""
    cov = [0.0] * 9
    for i in range(3):
        cov[4 * i] = float(sigma[i]) ** 2
    return cov


class GnssSensor(SensorBase):
    name = 'gnss'
    msg_type = NavSatFix
    input_topic = 'gnss/fix_ideal'
    output_topic = 'gnss/fix'
    channels = {'position': Channel(3, 5.0)}

    def __init__(self, seed=0):
        super().__init__(seed)
        self.vertical_factor = 2.0
        self._origin = None

    def _set_origin(self, msg):
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude, msg.altitude)

    def weights(self, channel):
        """A altura pesa `vertical_factor` vezes nos efeitos aditivos."""
        return np.array([1.0, 1.0, self.vertical_factor])

    def idle(self, msg, t):
        """Guarda a primeira fixação mesmo sem efeito ativo."""
        self._set_origin(msg)

    def read(self, msg):
        """Leste, norte e altura em metros desde a primeira fixação (esfera de 6 378 137 m)."""
        self._set_origin(msg)
        lat0, lon0, alt0 = self._origin
        east = math.radians(msg.longitude - lon0) * EARTH_RADIUS * math.cos(math.radians(lat0))
        north = math.radians(msg.latitude - lat0) * EARTH_RADIUS
        return {'position': np.array([east, north, msg.altitude - alt0])}

    def write(self, msg, values, sigmas, t):
        """Converte os metros de volta para latitude, longitude e altitude, numa cópia."""
        out = copy.deepcopy(msg)
        lat0, lon0, alt0 = self._origin
        east, north, up = values['position']
        out.latitude = lat0 + math.degrees(north / EARTH_RADIUS)
        out.longitude = lon0 + math.degrees(east / (EARTH_RADIUS * math.cos(math.radians(lat0))))
        out.altitude = alt0 + up
        if sigmas is not None:
            out.position_covariance = _diagonal(sigmas['position'])
            out.position_covariance_type = type(msg).COVARIANCE_TYPE_DIAGONAL_KNOWN
        return out


class ImuSensor(SensorBase):
    name = 'imu'
    msg_type = Imu
    input_topic = 'imu/data_ideal'
    output_topic = 'imu/data'
    channels = {'gyro': Channel(3, 0.1), 'accel': Channel(3, 1.0)}

    def read(self, msg):
        """Velocidade angular e aceleração linear, 3 eixos cada."""
        g, a = msg.angular_velocity, msg.linear_acceleration
        return {'gyro': np.array([g.x, g.y, g.z]), 'accel': np.array([a.x, a.y, a.z])}

    def write(self, msg, values, sigmas, t):
        """Grava os vetores numa cópia; a orientação não muda."""
        out = copy.deepcopy(msg)
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = (
            float(v) for v in values['gyro'])
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = (
            float(v) for v in values['accel'])
        if sigmas is not None:
            out.angular_velocity_covariance = _diagonal(sigmas['gyro'])
            out.linear_acceleration_covariance = _diagonal(sigmas['accel'])
        return out


class OdometrySensor(SensorBase):
    name = 'odometry'
    msg_type = Odometry
    input_topic = 'odom_ideal'
    output_topic = 'odom'
    channels = {'linear': Channel(1, 1.0), 'angular': Channel(1, 0.5)}
    extra_effects = {'slip': {'rate': 0.0, 'duration': 2.0, 'fraction': 0.0}}
    # Faixas dos parâmetros próprios, para o nó validar.
    extra_ranges = {('slip', 'rate'): (0, 1), ('slip', 'duration'): (0, 30),
                    ('slip', 'fraction'): (0, 1)}

    def __init__(self, seed=0):
        super().__init__(seed)
        self._pose = None
        self._pose_t = None
        self._slip_end = None

    def extra_chain(self, values, t, dt, active):
        """Patinagem: num episódio, a velocidade linear sai multiplicada por 1 + fraction."""
        if not active['slip']:
            self._slip_end = None
            return values
        slip = self.effects['slip'].values
        if self._slip_end is not None and t >= self._slip_end:
            self._slip_end = None
        if self._slip_end is None:
            rate = slip['rate']
            if rate > 0 and self.rng.random() < 1.0 - np.exp(-rate * dt):
                self._slip_end = t + slip['duration']
        if self._slip_end is not None:
            values['linear'] = values['linear'] * (1.0 + slip['fraction'])
        return values

    def idle(self, msg, t):
        """Sem efeito ativo a pose é a ideal; zera o integrador."""
        self._pose = None
        self._pose_t = None

    def read(self, msg):
        """Velocidade linear em x e angular em z."""
        return {'linear': np.array([msg.twist.twist.linear.x]),
                'angular': np.array([msg.twist.twist.angular.z])}

    def write(self, msg, values, sigmas, t):
        """Integra a velocidade degradada a partir da pose ideal da primeira mensagem ativa."""
        out = copy.deepcopy(msg)
        v = float(values['linear'][0])
        w = float(values['angular'][0])
        if self._pose is None:
            q = msg.pose.pose.orientation
            psi = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, psi]
        else:
            dt = t - self._pose_t
            x, y, psi = self._pose
            self._pose = [x + v * math.cos(psi) * dt, y + v * math.sin(psi) * dt, psi + w * dt]
        self._pose_t = t
        x, y, psi = self._pose
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(psi / 2.0)
        out.pose.pose.orientation.w = math.cos(psi / 2.0)
        out.twist.twist.linear.x = v
        out.twist.twist.angular.z = w
        if sigmas is not None:
            cov = list(out.twist.covariance)
            cov[0] = float(sigmas['linear'][0]) ** 2
            cov[35] = float(sigmas['angular'][0]) ** 2
            out.twist.covariance = cov
        return out


class CameraSensor(SensorBase):
    name = 'camera'
    msg_type = Image
    input_topic = 'camera_front/image_ideal'
    output_topic = 'camera_front/image_raw'


class TagDetectionSensor(SensorBase):
    name = 'tag_detection'
    msg_type = AprilTagDetectionArray
    input_topic = 'detections_ideal'
    output_topic = 'detections'
