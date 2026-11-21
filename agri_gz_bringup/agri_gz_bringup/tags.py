"""A textura da tag fiducial que o veículo leva na placa.

`tag_texture` grava o PNG do quadrado preto da tag, para o modelo do Gazebo usar como textura.
"""

import tempfile
from pathlib import Path

import cv2

# Lado da imagem, em pixels.
SIZE = 512

# Famílias aceitas, pelos dicionários AprilTag do OpenCV.
_DICTIONARIES = {
    '16h5': cv2.aruco.DICT_APRILTAG_16h5,
    '25h9': cv2.aruco.DICT_APRILTAG_25h9,
    '36h10': cv2.aruco.DICT_APRILTAG_36h10,
    '36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


def tag_texture(family, tag_id, folder=None):
    """Grava `tag<família>_<ID>.png` e devolve o caminho.

    A imagem inclui o anel preto de fora e não inclui a margem branca, que no veículo vem da
    placa. Sem `folder`, o arquivo vai para `<pasta temporária>/agri_gz_tags`. Se ele já
    existe, não é regravado. Família desconhecida ou ID fora dela dá `ValueError`.
    """
    if family not in _DICTIONARIES:
        raise ValueError(f'família de tag desconhecida: {family!r}; '
                         f'use uma de {", ".join(_DICTIONARIES)}')
    dictionary = cv2.aruco.getPredefinedDictionary(_DICTIONARIES[family])
    count = dictionary.bytesList.shape[0]
    if not 0 <= tag_id < count:
        raise ValueError(f'ID {tag_id} fora da família {family}, que vai de 0 a {count - 1}')

    folder = Path(folder) if folder is not None else Path(tempfile.gettempdir()) / 'agri_gz_tags'
    path = folder / f'tag{family}_{tag_id}.png'
    if not path.exists():
        folder.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), cv2.aruco.drawMarker(dictionary, tag_id, SIZE, borderBits=1))
    return path
