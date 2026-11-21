"""Testes da distorção em ponto fixo (R2).

`Distorter(K, D, width, height)` guarda o mapa do `cv2.remap` em ponto fixo (`cv2.convertMaps`
para `CV_16SC2`: `map1` com `int16` (altura, largura, 2) e `map2` com `uint16`), que é mais
rápido que o mapa em ponto flutuante de `distortion_maps`. `apply(image)` devolve a imagem
distorcida, que fica igual à do mapa em ponto flutuante dentro de uma tolerância pequena. Com `D`
nulo, `apply` devolve a própria imagem, sem cópia.
"""
import cv2
import numpy as np
import pytest

from agri_gz_bringup.distortion import Distorter, distortion_maps

WIDTH, HEIGHT = 1280, 720
FOCAL = (WIDTH / 2) / np.tan(np.radians(120) / 2)
K = np.array([[FOCAL, 0, WIDTH / 2], [0, FOCAL, HEIGHT / 2], [0, 0, 1]])
D = np.array([-0.10, 0.01, 0.0, 0.0, 0.0])


def scene(channels):
    """Uma imagem com gradientes e formas suaves, como as que a câmera vê."""
    u, v = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
    base = (127 + 60 * np.sin(u / 37.0) + 40 * np.cos(v / 23.0)).astype(np.uint8)
    for x, y, r in ((300, 200, 60), (900, 500, 90), (640, 360, 40)):
        cv2.circle(base, (x, y), r, 255, -1)
    base = cv2.GaussianBlur(base, (7, 7), 0)
    return base if channels == 1 else np.dstack([base, 255 - base, base // 2])


def test_fixed_point_maps():
    distorter = Distorter(K, D, WIDTH, HEIGHT)
    assert distorter.map1.dtype == np.int16
    assert distorter.map1.shape == (HEIGHT, WIDTH, 2)
    assert distorter.map2.dtype == np.uint16


@pytest.mark.parametrize('channels', [1, 3])
def test_matches_the_floating_point_maps(channels):
    image = scene(channels)
    fast = Distorter(K, D, WIDTH, HEIGHT).apply(image)
    map_x, map_y = distortion_maps(K, D, WIDTH, HEIGHT)
    reference = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)
    assert fast.shape == reference.shape
    diff = np.abs(fast.astype(np.int16) - reference.astype(np.int16))
    assert diff.mean() < 0.5
    assert diff.max() <= 4


def test_zero_distortion_returns_the_same_image():
    image = scene(3)
    assert Distorter(K, np.zeros(5), WIDTH, HEIGHT).apply(image) is image
