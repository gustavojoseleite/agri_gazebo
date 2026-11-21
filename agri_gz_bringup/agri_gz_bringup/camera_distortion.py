"""Nó que aplica a distorção de Brown à imagem da câmera.

O Gazebo Harmonic publica os coeficientes de Brown no `camera_info`, mas o ogre2 não distorce
a imagem renderizada. Este nó assina `<câmera>/image_ideal` e `<câmera>/camera_info` e publica
`<câmera>/image_raw` já distorcida, com o mesmo cabeçalho.
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from agri_gz_bringup.distortion import distortion_maps


class CameraDistortion(Node):

    def __init__(self):
        super().__init__('camera_distortion')
        camera = self.declare_parameter('camera', 'camera_front').value
        self._bridge = CvBridge()
        self._key = None
        self._maps = None
        self._pub = self.create_publisher(Image, f'{camera}/image_raw', 10)
        self.create_subscription(CameraInfo, f'{camera}/camera_info', self._on_info, 10)
        self.create_subscription(Image, f'{camera}/image_ideal', self._on_image,
                                 qos_profile_sensor_data)

    def _on_info(self, msg):
        """Recalcula os mapas só se K, D ou o tamanho mudaram."""
        key = (tuple(msg.k), tuple(msg.d), msg.width, msg.height)
        if key == self._key:
            return
        self._key = key
        if not any(msg.d):
            self._maps = None
            return
        self._maps = distortion_maps(np.array(msg.k).reshape(3, 3), np.array(msg.d),
                                     msg.width, msg.height)

    def _on_image(self, msg):
        if self._key is None:
            return
        if self._maps is None:
            self._pub.publish(msg)
            return
        ideal = self._bridge.imgmsg_to_cv2(msg)
        distorted = cv2.remap(ideal, self._maps[0], self._maps[1], cv2.INTER_LINEAR)
        out = self._bridge.cv2_to_imgmsg(distorted, encoding=msg.encoding)
        out.header = msg.header
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
