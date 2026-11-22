"""Testes dos efeitos da imagem (F1.4c, R1), em funções puras sobre `numpy`.

`agri_sensor_degradation.image`:
- `occlude(image, corners, fraction, edge)`: devolve uma cópia com uma faixa opaca (`MUD`) que
  parte do lado entre `corners[edge]` e `corners[edge + 1]` e cobre `fraction` da área do
  quadrilátero da tag (4 cantos em sequência, em pixels);
- `dust(image, contrast, blur)`: `contrast` é a perda de contraste, de 0 (nenhuma) a 1 (cinza
  liso): a imagem é misturada com o cinza médio (128) nessa proporção; depois, desfoque
  gaussiano com σ = `blur` px;
- `brightness(image, gain, offset)`: `gain · pixel + offset`, saturando em 0 e 255.

Todas devolvem uma imagem nova, `uint8`, do mesmo formato, sem mudar a de entrada.
"""
import cv2
import numpy as np
import pytest

from agri_sensor_degradation.image import MUD, brightness, dust, occlude


def tag_mask(shape, corners):
    mask = np.zeros(shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, np.round(corners).astype(np.int32), 1)
    return mask.astype(bool)


def covered(out, mask):
    mud = np.all(out.reshape(-1, out.shape[-1]) == MUD, axis=1).reshape(out.shape[:2])
    return (mud & mask).sum() / mask.sum(), (mud & ~mask).sum()


SQUARE = np.array([[400, 200], [600, 200], [600, 400], [400, 400]], float)
TILTED = np.array([[420, 180], [640, 230], [600, 430], [390, 390]], float)


@pytest.mark.parametrize('fraction', [0.25, 0.5, 0.75])
@pytest.mark.parametrize('edge', [0, 1, 2, 3])
def test_occlusion_covers_the_fraction_of_the_tag(fraction, edge):
    image = np.full((720, 1280, 3), 255, np.uint8)
    for corners, tolerance in ((SQUARE, 0.02), (TILTED, 0.05)):
        out = occlude(image, corners, fraction, edge)
        share, outside = covered(out, tag_mask(image.shape, corners))
        assert share == pytest.approx(fraction, abs=tolerance)
        assert outside < 0.01 * tag_mask(image.shape, corners).sum()


def test_occlusion_starts_at_the_given_edge():
    image = np.full((720, 1280, 3), 255, np.uint8)
    top = occlude(image, SQUARE, 0.25, 0)
    assert np.all(top[210, 500] == MUD) and np.all(top[390, 500] == 255)
    bottom = occlude(image, SQUARE, 0.25, 2)
    assert np.all(bottom[390, 500] == MUD) and np.all(bottom[210, 500] == 255)


def test_occlusion_of_nothing_and_input_untouched():
    image = np.full((720, 1280, 3), 255, np.uint8)
    assert np.array_equal(occlude(image, SQUARE, 0.0, 0), image)
    occlude(image, SQUARE, 0.5, 1)
    assert np.all(image == 255)


def test_occlusion_on_a_mono_image():
    image = np.full((720, 1280), 255, np.uint8)
    out = occlude(image, SQUARE, 0.5, 3)
    assert out.shape == image.shape and out.dtype == np.uint8
    assert (out[tag_mask(image.shape, SQUARE)] != 255).mean() == pytest.approx(0.5, abs=0.02)


def pattern():
    rng = np.random.default_rng(0)
    return rng.integers(0, 256, (720, 1280, 3), dtype=np.uint8)


def test_dust_lowers_the_contrast():
    image = pattern()
    out = dust(image, contrast=0.5, blur=0.0)
    expected = 128 + 0.5 * (image.astype(float) - 128)
    assert np.abs(out.astype(float) - expected).max() <= 1.0
    assert out.std() == pytest.approx(0.5 * image.std(), rel=0.02)


def test_dust_blurs():
    image = pattern()
    sharp = cv2.Laplacian(image, cv2.CV_64F).var()
    out = dust(image, contrast=0.0, blur=2.0)
    assert cv2.Laplacian(out, cv2.CV_64F).var() < 0.1 * sharp
    assert out.mean() == pytest.approx(image.mean(), abs=1.0)


def test_dust_off_is_the_same_image():
    image = pattern()
    assert np.array_equal(dust(image, 0.0, 0.0), image)


def test_brightness_gain_offset_and_saturation():
    image = np.array([[[0, 100, 200]]], np.uint8)
    assert brightness(image, 1.6, 0.0).tolist() == [[[0, 160, 255]]]
    assert brightness(image, 1.0, -50.0).tolist() == [[[0, 50, 150]]]
    assert brightness(image, 1.0, 0.0).tolist() == image.tolist()
    assert brightness(image, 1.6, 0.0).dtype == np.uint8
