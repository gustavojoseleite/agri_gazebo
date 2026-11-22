"""Detector de tags AprilTag pela `libapriltag`, carregada com `ctypes`."""
import ctypes
import glob

import numpy as np


class _Image(ctypes.Structure):
    _fields_ = [('width', ctypes.c_int32), ('height', ctypes.c_int32),
                ('stride', ctypes.c_int32), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class _ZArray(ctypes.Structure):
    _fields_ = [('el_sz', ctypes.c_size_t), ('size', ctypes.c_int),
                ('alloc', ctypes.c_int), ('data', ctypes.c_void_p)]


class _Detection(ctypes.Structure):
    _fields_ = [('family', ctypes.c_void_p), ('id', ctypes.c_int),
                ('hamming', ctypes.c_int), ('decision_margin', ctypes.c_float),
                ('H', ctypes.c_void_p), ('c', ctypes.c_double * 2),
                ('p', (ctypes.c_double * 2) * 4)]


class Detector:
    """Detecta tags de uma família (por exemplo `36h11`) numa imagem em tons de cinza."""

    def __init__(self, family='36h11'):
        path = glob.glob('/opt/ros/jazzy/lib/**/libapriltag.so', recursive=True)[0]
        self._lib = lib = ctypes.CDLL(path)
        create = getattr(lib, f'tag{family}_create')
        create.restype = ctypes.c_void_p
        lib.apriltag_detector_create.restype = ctypes.c_void_p
        lib.apriltag_detector_add_family_bits.argtypes = [ctypes.c_void_p, ctypes.c_void_p,
                                                          ctypes.c_int]
        lib.apriltag_detector_detect.restype = ctypes.POINTER(_ZArray)
        lib.apriltag_detector_detect.argtypes = [ctypes.c_void_p, ctypes.POINTER(_Image)]
        lib.apriltag_detections_destroy.argtypes = [ctypes.POINTER(_ZArray)]
        self._detector = lib.apriltag_detector_create()
        lib.apriltag_detector_add_family_bits(self._detector, create(), 2)

    def detect(self, gray):
        """Devolve `[(id, cantos 4x2)]` das tags achadas na imagem uint8 de um canal."""
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        image = _Image(gray.shape[1], gray.shape[0], gray.shape[1],
                       gray.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)))
        found = self._lib.apriltag_detector_detect(self._detector, ctypes.byref(image))
        result = []
        for i in range(found.contents.size):
            address = ctypes.c_void_p.from_address(
                found.contents.data + i * found.contents.el_sz).value
            det = ctypes.cast(address, ctypes.POINTER(_Detection)).contents
            corners = np.array([[det.p[k][0], det.p[k][1]] for k in range(4)], float)
            result.append((int(det.id), corners))
        self._lib.apriltag_detections_destroy(found)
        return result
