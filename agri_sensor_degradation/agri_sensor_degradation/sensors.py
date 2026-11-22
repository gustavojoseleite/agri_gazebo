"""Uma classe por sensor do simulador: o tipo da mensagem e os tópicos padrão.

O tópico de entrada é o ideal, que o simulador publica; o de saída é o normal, que os
outros nós consomem.
"""
from apriltag_msgs.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, NavSatFix

from agri_sensor_degradation.core import SensorBase


class GnssSensor(SensorBase):
    name = 'gnss'
    msg_type = NavSatFix
    input_topic = 'gnss/fix_ideal'
    output_topic = 'gnss/fix'


class ImuSensor(SensorBase):
    name = 'imu'
    msg_type = Imu
    input_topic = 'imu/data_ideal'
    output_topic = 'imu/data'


class OdometrySensor(SensorBase):
    name = 'odometry'
    msg_type = Odometry
    input_topic = 'odom_ideal'
    output_topic = 'odom'


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
