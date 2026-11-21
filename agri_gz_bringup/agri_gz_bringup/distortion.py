"""A distorção de Brown aplicada à imagem renderizada.

`distortion_maps` calcula os mapas do `cv2.remap` que levam a imagem ideal à imagem distorcida
descrita por um `camera_info`.
"""

import cv2
import numpy as np


def distortion_maps(K, D, width, height):
    """Devolve `(map_x, map_y)`, em `float32` e com a forma `(height, width)`.

    `K` é a matriz intrínseca 3x3 e `D` os coeficientes na ordem do ROS (k1, k2, p1, p2, k3).
    Para cada pixel da imagem distorcida, os mapas apontam o pixel da imagem ideal que o
    originou; `cv2.remap(ideal, map_x, map_y, ...)` dá a imagem distorcida. Com `D` nulo, os
    mapas são a identidade.
    """
    K = np.asarray(K, dtype=np.float64).reshape(3, 3)
    D = np.asarray(D, dtype=np.float64).ravel()
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    pixels = np.stack([u, v], axis=-1).reshape(-1, 1, 2).astype(np.float64)
    # Só `undistortPointsIter` aceita o critério de iteração no OpenCV 4.6.
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 20, 1e-9)
    normalized = cv2.undistortPointsIter(pixels, K, D, None, None, criteria).reshape(-1, 2)
    map_x = normalized[:, 0] * K[0, 0] + K[0, 2]
    map_y = normalized[:, 1] * K[1, 1] + K[1, 2]
    return (map_x.reshape(height, width).astype(np.float32),
            map_y.reshape(height, width).astype(np.float32))


class Distorter:
    """Aplica a distorção com o mapa do `remap` em ponto fixo,
    mais rápido que o de ponto flutuante.

    Os mapas ficam em `map1` (`int16`, com a forma `(height, width, 2)`) e `map2` (`uint16`).
    Com `D` nulo não há mapa: `apply` devolve a própria imagem, sem cópia.
    """

    def __init__(self, K, D, width, height):
        self.map1 = None
        self.map2 = None
        if not np.any(D):
            return
        map_x, map_y = distortion_maps(K, D, width, height)
        self.map1, self.map2 = cv2.convertMaps(map_x, map_y, cv2.CV_16SC2)

    def apply(self, image):
        """Devolve a imagem distorcida; com `D` nulo, a própria `image`."""
        if self.map1 is None:
            return image
        return cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
