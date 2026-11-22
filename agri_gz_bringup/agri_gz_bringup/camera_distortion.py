"""Nó que aplica a distorção de Brown à imagem da câmera.

O Gazebo Harmonic publica os coeficientes de Brown no `camera_info`, mas o ogre2 não distorce
a imagem renderizada. Este nó assina `<câmera>/image_ideal` e `<câmera>/camera_info` e publica
`<câmera>/image_raw` já distorcida, com o mesmo cabeçalho. Como gera a imagem da câmera, é
também o nó que a degrada: tem os parâmetros `loss.*` e publica `degradation/state`; uma imagem
perdida simplesmente não é publicada.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from agri_gz_bringup.distortion import Distorter
from agri_sensor_degradation.node import DegradationParameters
from agri_sensor_degradation.sensors import CameraSensor


class CameraDistortion(Node):

    def __init__(self):
        super().__init__('camera_distortion')
        camera = self.declare_parameter('camera', 'camera_front').value
        self._bridge = CvBridge()
        self._degradation = DegradationParameters(self, CameraSensor())
        self._key = None
        self._distorter = None
        self._pub = self.create_publisher(Image, f'{camera}/image_raw', 10)
        self.create_subscription(CameraInfo, f'{camera}/camera_info', self._on_info, 10)
        self.create_subscription(Image, f'{camera}/image_ideal', self._on_image,
                                 qos_profile_sensor_data)

    def _on_info(self, msg):
        """Recria o `Distorter` só se K, D ou o tamanho mudaram."""
        key = (tuple(msg.k), tuple(msg.d), msg.width, msg.height)
        if key == self._key:
            return
        self._key = key
        if not any(msg.d):
            self._distorter = None
            return
        self._distorter = Distorter(np.array(msg.k).reshape(3, 3), np.array(msg.d),
                                    msg.width, msg.height)

    def _on_image(self, msg):
        if self._key is None:
            return
        if self._distorter is None:
            msg = self._degradation.apply(msg)
            if msg is not None:
                self._pub.publish(msg)
            return
        ideal = self._bridge.imgmsg_to_cv2(msg)
        distorted = self._distorter.apply(ideal)
        out = self._bridge.cv2_to_imgmsg(distorted, encoding=msg.encoding)
        out.header = msg.header
        out = self._degradation.apply(out)
        if out is not None:
            self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraDistortion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
