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
