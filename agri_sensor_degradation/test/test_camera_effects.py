"""Testes do detector de tags e dos efeitos na `CameraSensor` (F1.4c, R2, R3 e R4), sem `rclpy`.

- `apriltag.Detector(family).detect(gray)` devolve `[(id, cantos 4x2)]`, pela `libapriltag`.
- A `CameraSensor` tem os efeitos próprios `occlusion` (`fraction`), `dust` (`contrast`, `blur`) e
  `brightness` (`gain`, `offset`). Ela os aplica numa cópia da `sensor_msgs/Image`, com o mesmo
  cabeçalho, codificação e tamanho. A oclusão acha as tags na própria imagem; a borda da faixa é
  sorteada no começo do episódio e fica fixa enquanto o efeito está ativo.
- O `profiles.yaml` tem os perfis da câmera.
"""
import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

from agri_sensor_degradation import profiles
from agri_sensor_degradation.apriltag import Detector
from agri_sensor_degradation.image import MUD
from agri_sensor_degradation.sensors import CameraSensor

SIDE = 240
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def frame(tags=((1, 400, 200),), channels=3):
    """Imagem branca de 1280×720 com tags 36h11 (id, x, y do canto superior esquerdo)."""
    gray = np.full((720, 1280), 255, np.uint8)
    for tag_id, x, y in tags:
        gray[y:y + SIDE, x:x + SIDE] = cv2.aruco.drawMarker(DICTIONARY, tag_id, SIDE,
                                                            borderBits=1)
    return gray if channels == 1 else np.dstack([gray] * channels)


def message(image):
    msg = Image()
    msg.header.stamp.sec = 7
    msg.header.frame_id = 'camera_front_optical'
    msg.height, msg.width = image.shape[:2]
    msg.encoding = 'mono8' if image.ndim == 2 else 'rgb8'
    msg.step = msg.width * (1 if image.ndim == 2 else 3)
    msg.data = image.tobytes()
    return msg


def pixels(msg):
    shape = (msg.height, msg.width) if msg.encoding == 'mono8' else (msg.height, msg.width, 3)
    return np.frombuffer(bytes(msg.data), np.uint8).reshape(shape)


def square(x=400, y=200):
    mask = np.zeros((720, 1280), bool)
    mask[y:y + SIDE, x:x + SIDE] = True
    return mask


def test_detector_finds_the_tags_and_their_corners():
    found = Detector('36h11').detect(frame(((1, 400, 200), (3, 900, 300)), channels=1))
    assert sorted(tag_id for tag_id, _ in found) == [1, 3]
    corners = dict(found)[1]
    assert corners.shape == (4, 2)
    expected = np.array([[400, 200], [400 + SIDE, 200], [400 + SIDE, 200 + SIDE],
                         [400, 200 + SIDE]], float)
    for point in expected:
        assert np.min(np.linalg.norm(corners - point, axis=1)) < 3.0
    assert Detector('36h11').detect(np.full((720, 1280), 255, np.uint8)) == []


def test_camera_effect_names_and_defaults():
    effects = CameraSensor(seed=0).effects
    assert effects['occlusion'].values == {'fraction': 0.0}
    assert effects['dust'].values == {'contrast': 0.0, 'blur': 0.0}
    assert effects['brightness'].values == {'gain': 1.0, 'offset': 0.0}


def test_occlusion_covers_the_tag_only():
    sensor = CameraSensor(seed=1)
    sensor.set_effect('occlusion', fraction=0.5)
    image = frame()
    msg = message(image)
    out = sensor.apply(msg, 0.0, None)
    result = pixels(out)
    mud = np.all(result == MUD, axis=2)
    assert mud[square()].mean() == pytest.approx(0.5, abs=0.05)
    # Fora da tag nada muda, a não ser a borda de um pixel que os cantos detectados admitem.
    changed_outside = np.any(result != image, axis=2) & ~square()
    assert changed_outside.sum() < 0.01 * square().sum()
    assert (out.header.stamp.sec, out.header.frame_id) == (7, 'camera_front_optical')
    assert (out.encoding, out.width, out.height, out.step) == ('rgb8', 1280, 720, 3840)
    assert bytes(msg.data) == image.tobytes()


def test_occlusion_without_a_tag_changes_nothing():
    sensor = CameraSensor(seed=1)
    sensor.set_effect('occlusion', fraction=0.75)
    image = frame(tags=())
    assert np.array_equal(pixels(sensor.apply(message(image), 0.0, None)), image)


def test_occlusion_edge_is_fixed_in_an_episode_and_drawn_again_after():
    sensor = CameraSensor(seed=3)
    sensor.set_effect('occlusion', fraction=0.25)
    image = frame()
    masks = set()
    for episode in range(12):
        sensor.effects['occlusion'].enabled = True
        first = None
        for k in range(3):
            mud = np.all(pixels(sensor.apply(message(image), episode + k * 0.1, None)) == MUD, 2)
            if first is None:
                first = mud
            assert np.array_equal(mud, first)
        masks.add(first.tobytes())
        sensor.effects['occlusion'].enabled = False
        msg = message(image)
        assert sensor.apply(msg, episode + 0.5, None) is msg
    assert len(masks) >= 2


def test_dust_and_brightness_on_the_message():
    rng = np.random.default_rng(0)
    image = rng.integers(40, 200, (720, 1280, 3), dtype=np.uint8)
    sensor = CameraSensor(seed=0)
    sensor.set_effect('brightness', gain=1.6)
    bright = pixels(sensor.apply(message(image), 0.0, None)).astype(float)
    expected = np.clip(1.6 * image.astype(float), 0, 255).mean()
    assert bright.mean() == pytest.approx(expected, abs=1.0)
    sensor = CameraSensor(seed=0)
    sensor.set_effect('dust', contrast=0.5, blur=2.0)
    dusty = pixels(sensor.apply(message(image), 0.0, None))
    assert dusty.std() < 0.5 * image.std()


def test_effects_on_a_mono_image():
    sensor = CameraSensor(seed=0)
    sensor.set_effect('occlusion', fraction=0.5)
    out = sensor.apply(message(frame(channels=1)), 0.0, None)
    assert out.encoding == 'mono8'
    assert (pixels(out)[square()] != pixels(message(frame(channels=1)))[square()]).any()


def test_camera_profiles():
    for fraction in (25, 50, 75):
        values = profiles.load('camera', f'occlusion_{fraction}')
        assert values == {'occlusion.enabled': True,
                          'occlusion.fraction': pytest.approx(fraction / 100)}
    assert profiles.load('camera', 'dust') == {
        'dust.enabled': True, 'dust.contrast': pytest.approx(0.5),
        'dust.blur': pytest.approx(2.0)}
    assert profiles.load('camera', 'glare') == {
        'brightness.enabled': True, 'brightness.gain': pytest.approx(1.6)}
