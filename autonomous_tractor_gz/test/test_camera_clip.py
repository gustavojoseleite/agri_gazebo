"""Testes do plano de corte da câmera dos seguidores (R3).

O plano de corte distante passa de 200 m para 60 m, pelo argumento `camera_far` do xacro: a tag
é lida até uns 20 m, e cada árvore além disso só custava tempo de renderização. O plano de corte
próximo não muda (0,2 m).
"""
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro

XACRO = Path(__file__).resolve().parents[1] / 'urdf' / 'autonomous_tractor.gz.xacro'


def cameras(**args):
    """As câmeras do SDF do trator, por nome."""
    args.setdefault('name', 'tractor_1')
    args.setdefault('tag_texture', '/tmp/agri_gz_test/tag36h11_1.png')
    urdf = xacro.process_file(str(XACRO),
                              mappings={k: str(v) for k, v in args.items()}).toxml()
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(urdf)
        f.flush()
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    model = ET.fromstring(result.stdout).find('model')
    return {s.get('name'): s for s in model.iter('sensor') if s.get('type') == 'camera'}


def clip(camera):
    """(near, far) da câmera, em m."""
    return (float(camera.find('camera/clip/near').text),
            float(camera.find('camera/clip/far').text))


def test_far_clip_is_sixty_metres():
    assert clip(cameras()['camera_front_link']) == pytest.approx((0.2, 60.0))


def test_far_clip_is_an_argument():
    assert clip(cameras(camera_far=120)['camera_front_link']) == pytest.approx((0.2, 120.0))


def test_dual_mount_uses_the_same_clip():
    for camera in cameras(camera_mount='dual').values():
        assert clip(camera) == pytest.approx((0.2, 60.0))
