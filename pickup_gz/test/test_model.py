"""Testes do modelo da pickup no Gazebo (F1.2, R1, R3, R4 e R6).

O líder não tem câmera: ele segue a rota predefinida dos carreadores. Tem GNSS e IMU com os
mesmos perfis do trator, a textura da tag e a publicação da pose real.
"""
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
XACRO = PACKAGE / 'urdf' / 'pickup.gz.xacro'
METRES_PER_DEGREE = 111320.0
TEXTURE = '/tmp/agri_gz_test/tag36h11_0.png'


def sdf(**args):
    """O SDF da pickup, como a simulação o recebe."""
    args.setdefault('name', 'pickup')
    args.setdefault('tag_texture', TEXTURE)
    urdf = xacro.process_file(str(XACRO),
                              mappings={k: str(v) for k, v in args.items()}).toxml()
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(urdf)
        f.flush()
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout).find('model')


def kinds(model):
    """Os tipos de sensor do modelo."""
    return sorted(s.get('type') for s in model.iter('sensor'))


def test_leader_has_gnss_and_imu_and_no_camera():
    assert kinds(sdf()) == ['imu', 'navsat']


@pytest.mark.parametrize('profile, gnss_rate, horizontal', [
    ('default', 5.0, 1.0), ('nominal', 1.0, 0.02), ('ideal', 1.0, 0.0)])
def test_gnss_profiles(profile, gnss_rate, horizontal):
    gnss = next(s for s in sdf(sensor_profile=profile).iter('sensor')
                if s.get('type') == 'navsat')
    assert float(gnss.find('update_rate').text) == pytest.approx(gnss_rate)
    assert float(gnss.find('navsat/position_sensing/horizontal/noise/stddev').text) == \
        pytest.approx(horizontal / METRES_PER_DEGREE, abs=1e-12)


def test_tag_texture():
    textured = [v.find('material/pbr/metal/albedo_map').text for v in sdf().iter('visual')
                if v.find('material/pbr/metal/albedo_map') is not None]
    assert textured == [TEXTURE]


def test_ground_truth_pose_publisher():
    plugins = [p for p in sdf().iter('plugin') if 'pose-publisher' in p.get('filename', '')]
    assert len(plugins) == 1
    assert plugins[0].find('publish_model_pose').text == 'true'


def test_bridge_entries():
    entries = yaml.safe_load((PACKAGE / 'config' / 'bridge.yaml').read_text())
    ros = [e['ros_topic_name'] for e in entries]
    assert not any('camera' in topic for topic in ros)
    assert '/ground_truth/{name}/pose' in ros
