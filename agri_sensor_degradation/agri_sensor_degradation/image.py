"""Efeitos de degradação da imagem da câmera, em funções puras sobre arrays do numpy."""
import cv2
import numpy as np

# Cor do barro (três valores de 0 a 255).
MUD = (60, 45, 30)


def _to_uint8(values):
    """Arredonda, satura em 0 e 255 e converte para uint8."""
    return np.clip(np.rint(values), 0, 255).astype(np.uint8)


def occlude(image, corners, fraction, edge):
    """Cobre de barro a fração `fraction` da área da tag, a partir do lado `edge`."""
    out = image.copy()
    if fraction <= 0.0:
        return out
    c = np.asarray(corners, float)
    a, b = c[edge % 4], c[(edge + 1) % 4]
    c2, c3 = c[(edge + 2) % 4], c[(edge + 3) % 4]
    polygon = np.array([a, b, b + fraction * (c2 - b), a + fraction * (c3 - a)])
    color = MUD if out.ndim == 3 else int(round(sum(MUD) / len(MUD)))
    cv2.fillConvexPoly(out, np.round(polygon).astype(np.int32), color)
    return out


def dust(image, contrast, blur):
    """Perde contraste em direção ao cinza médio e desfoca com sigma `blur`."""
    values = 128.0 + (1.0 - contrast) * (image.astype(np.float64) - 128.0)
    out = _to_uint8(values)
    if blur > 0:
        out = cv2.GaussianBlur(out, (0, 0), blur)
    return out


def brightness(image, gain, offset):
    """Aplica `gain * pixel + offset`, saturando em 0 e 255."""
    return _to_uint8(gain * image.astype(np.float64) + offset)
