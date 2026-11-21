"""Testes da distorção de Brown aplicada à imagem renderizada (F1.2, R2).

O Gazebo Harmonic publica os coeficientes de Brown no `camera_info`, mas o ogre2 não distorce a
imagem. `distortion_maps(K, D, width, height)` devolve os mapas do `cv2.remap` que levam a imagem
ideal à imagem distorcida que esse `camera_info` descreve. `D` está na ordem do ROS
(k1, k2, p1, p2, k3). Um ponto da imagem ideal tem de cair onde o modelo de Brown o projeta.
"""
import cv2
import numpy as np
import pytest

from agri_gz_bringup.distortion import distortion_maps

WIDTH, HEIGHT = 1280, 720
FOCAL = (WIDTH / 2) / np.tan(np.radians(120) / 2)
K = np.array([[FOCAL, 0, WIDTH / 2], [0, FOCAL, HEIGHT / 2], [0, 0, 1]])
D = np.array([-0.10, 0.01, 0.0, 0.0, 0.0])


def test_maps_shape_and_type():
    map_x, map_y = distortion_maps(K, D, WIDTH, HEIGHT)
    assert map_x.shape == map_y.shape == (HEIGHT, WIDTH)
    assert map_x.dtype == map_y.dtype == np.float32


def test_zero_distortion_is_the_identity():
    map_x, map_y = distortion_maps(K, np.zeros(5), WIDTH, HEIGHT)
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    assert np.abs(map_x - u).max() < 1e-2
    assert np.abs(map_y - v).max() < 1e-2


@pytest.mark.parametrize('x, y', [(0.0, 0.0), (0.6, 0.0), (-0.8, 0.3), (0.5, -0.45),
                                  (1.1, 0.5)])
def test_point_lands_where_brown_puts_it(x, y):
    ideal = np.zeros((HEIGHT, WIDTH), np.uint8)
    ideal_px = K @ np.array([x, y, 1.0])
    cv2.circle(ideal, (int(round(ideal_px[0])), int(round(ideal_px[1]))), 3, 255, -1)
    map_x, map_y = distortion_maps(K, D, WIDTH, HEIGHT)
    distorted = cv2.remap(ideal, map_x, map_y, cv2.INTER_LINEAR)
    moments = cv2.moments(distorted, binaryImage=False)
    found = np.array([moments['m10'] / moments['m00'], moments['m01'] / moments['m00']])
    expected, _ = cv2.projectPoints(np.array([[x, y, 1.0]]), np.zeros(3), np.zeros(3), K, D)
    # O círculo foi desenhado no pixel inteiro mais próximo do ideal: meio pixel de folga.
    shift = ideal_px[:2] - np.round(ideal_px[:2])
    assert np.linalg.norm(found - (expected.ravel() - shift)) < 1.0


def test_barrel_pulls_the_edges_in():
    map_x, _ = distortion_maps(K, D, WIDTH, HEIGHT)
    # Num barril, o pixel da borda da imagem distorcida vem de fora da imagem ideal.
    assert map_x[HEIGHT // 2, WIDTH - 1] > WIDTH - 1
    assert map_x[HEIGHT // 2, 0] < 0
