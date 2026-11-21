"""Testes da textura da tag (F1.2, R1).

`tag_texture(family, tag_id, folder)` grava a imagem do quadrado preto da tag (com o anel preto
de fora) num PNG quadrado de 512 px e devolve o caminho. A margem branca não faz parte da
imagem: no veículo, ela vem da placa. A textura é gerada pelo OpenCV e decodificada aqui pela
`libapriltag`, a biblioteca do `apriltag_ros`, que é independente do gerador.
"""
import ctypes
import glob

import cv2
import numpy as np
import pytest

from agri_gz_bringup.tags import tag_texture


class _Image(ctypes.Structure):
    _fields_ = [('width', ctypes.c_int32), ('height', ctypes.c_int32),
                ('stride', ctypes.c_int32), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _ZArray(ctypes.Structure):
    _fields_ = [('el_sz', ctypes.c_size_t), ('size', ctypes.c_int),
                ('alloc', ctypes.c_int), ('data', ctypes.c_void_p)]


class _Detection(ctypes.Structure):
    _fields_ = [('family', ctypes.c_void_p), ('id', ctypes.c_int),
                ('hamming', ctypes.c_int), ('decision_margin', ctypes.c_float)]


def decode(image, family):
    """IDs que a `libapriltag` acha na imagem em tons de cinza."""
    lib = ctypes.CDLL(glob.glob('/opt/ros/jazzy/lib/**/libapriltag.so', recursive=True)[0])
    create = getattr(lib, f'tag{family}_create')
    create.restype = ctypes.c_void_p
    lib.apriltag_detector_create.restype = ctypes.c_void_p
    lib.apriltag_detector_add_family_bits.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                                      ctypes.c_int]
    lib.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
    lib.apriltag_detector_detect.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Image)]
    detector = lib.apriltag_detector_create()
    lib.apriltag_detector_add_family_bits(detector, create(), 2)
    image = np.ascontiguousarray(image, dtype=np.uint8)
    found = lib.apriltag_detector_detect(detector, ctypes.byref(_Image(
        image.shape[1], image.shape[0], image.shape[1],
        image.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))))).contents
    return [ctypes.cast(ctypes.c_void_p.from_address(found.data + i * found.el_sz).value,
                        ctypes.POINTER(_Detection)).contents.id for i in range(found.size)]


def with_margin(image):
    """A imagem com uma margem branca de 25%, como a placa dá no veículo."""
    pad = image.shape[0] // 4
    return cv2.copyMakeBorder(image, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


@pytest.mark.parametrize('family, tag_id', [('36h11', 0), ('36h11', 1), ('36h11', 4),
                                            ('36h11', 17), ('25h9', 3), ('16h5', 2)])
def test_decodes_with_the_right_id(tmp_path, family, tag_id):
    image = cv2.imread(str(tag_texture(family, tag_id, tmp_path)), cv2.IMREAD_GRAYSCALE)
    assert decode(with_margin(image), family) == [tag_id]


def test_image_is_the_black_square(tmp_path):
    image = cv2.imread(str(tag_texture('36h11', 5, tmp_path)), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (512, 512)
    # O anel de fora é preto: a imagem é o quadrado preto, sem a margem branca.
    for edge in (image[0, :], image[-1, :], image[:, 0], image[:, -1]):
        assert edge.max() < 64


def test_same_arguments_same_file(tmp_path):
    first = tag_texture('36h11', 2, tmp_path)
    second = tag_texture('36h11', 2, tmp_path)
    assert first == second
    assert first.exists()
    assert first.parent == tmp_path
    assert first.name == 'tag36h11_2.png'


def test_default_folder():
    assert tag_texture('36h11', 0).exists()


def test_unknown_family():
    with pytest.raises(ValueError):
        tag_texture('99h1', 0)


def test_id_out_of_the_family():
    with pytest.raises(ValueError):
        tag_texture('36h11', 587)
