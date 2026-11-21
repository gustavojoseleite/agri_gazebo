"""Testes do modelo do trator no Gazebo (F1.2, R1 a R4).

Expande o xacro do Gazebo com os argumentos, converte para SDF com o `gz sdf -p` (que funde as
juntas fixas, como na simulação) e confere os sensores, a textura da tag e a publicação da pose
real. Perfis (`sensor_profile`):
- `default`, os valores de hoje: GNSS a 5 Hz com 1 m e 2 m; IMU a 100 Hz com ruído;
- `nominal`, a D13: GNSS a 1 Hz com 2 cm e 4 cm; IMU a 20 Hz sem ruído;
- `ideal`, as taxas nominais sem ruído nenhum.
O σ horizontal do GNSS chega ao Gazebo em graus (dividido por 111320 m).
"""
import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
XACRO = PACKAGE / 'urdf' / 'autonomous_tractor.gz.xacro'
METRES_PER_DEGREE = 111320.0
TEXTURE = '/tmp/agri_gz_test/tag36h11_1.png'


def urdf(**args):
    """O URDF do trator no Gazebo, expandido com `args`."""
    args.setdefault('name', 'tractor_1')
    args.setdefault('tag_texture', TEXTURE)
    doc = xacro.process_file(str(XACRO), mappings={k: str(v) for k, v in args.items()})
    return doc.toxml()


def sdf(**args):
    """O SDF do modelo, como a simulação o recebe."""
    with tempfile.NamedTemporaryFile('w', suffix='.urdf') as f:
        f.write(urdf(**args))
        f.flush()
        result = subprocess.run(['gz', 'sdf', '-p', f.name], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return ET.fromstring(result.stdout).find('model')


def sensors(model, kind):
    """Os sensores do tipo `kind`, por nome."""
    return {s.get('name'): s for s in model.iter('sensor') if s.get('type') == kind}


def number(element, path):
    """O número no caminho dado."""
    return float(element.find(path).text)


def test_camera_of_a_follower():
    camera = sensors(sdf(), 'camera')['camera_front_link']
    assert number(camera, 'camera/horizontal_fov') == pytest.approx(math.radians(120), abs=1e-3)
    assert number(camera, 'update_rate') == pytest.approx(10.0)
    assert number(camera, 'camera/image/width') == 1280
    assert number(camera, 'camera/image/height') == 720
    assert camera.find('camera/noise') is None
    assert camera.find('topic').text == '/model/tractor_1/camera_front/image'
    distortion = {k: number(camera, f'camera/distortion/{k}') for k in
                  ('k1', 'k2', 'k3', 'p1', 'p2')}
    assert distortion == pytest.approx({'k1': -0.10, 'k2': 0.01, 'k3': 0.0, 'p1': 0.0,
                                        'p2': 0.0})


def test_distortion_coefficients_are_arguments():
    camera = sensors(sdf(camera_k1=-0.2, camera_p2=0.001), 'camera')['camera_front_link']
    assert number(camera, 'camera/distortion/k1') == pytest.approx(-0.2)
    assert number(camera, 'camera/distortion/p2') == pytest.approx(0.001)


def test_dual_camera_mount():
    cameras = sensors(sdf(camera_mount='dual'), 'camera')
    assert set(cameras) == {'camera_front_left_link', 'camera_front_right_link'}
    for side in ('left', 'right'):
        camera = cameras[f'camera_front_{side}_link']
        assert number(camera, 'camera/horizontal_fov') == pytest.approx(math.pi / 2, abs=1e-3)
        assert number(camera, 'update_rate') == pytest.approx(10.0)
        assert camera.find('topic').text == f'/model/tractor_1/camera_front_{side}/image'


@pytest.mark.parametrize('profile, gnss_rate, horizontal, vertical, imu_rate, imu_noise', [
    ('default', 5.0, 1.0, 2.0, 100.0, True),
    ('nominal', 1.0, 0.02, 0.04, 20.0, False),
    ('ideal', 1.0, 0.0, 0.0, 20.0, False),
])
def test_sensor_profiles(profile, gnss_rate, horizontal, vertical, imu_rate, imu_noise):
    model = sdf(sensor_profile=profile)
    gnss = sensors(model, 'navsat')['gnss_link']
    assert number(gnss, 'update_rate') == pytest.approx(gnss_rate)
    assert number(gnss, 'navsat/position_sensing/horizontal/noise/stddev') == \
        pytest.approx(horizontal / METRES_PER_DEGREE, abs=1e-12)
    assert number(gnss, 'navsat/position_sensing/vertical/noise/stddev') == \
        pytest.approx(vertical, abs=1e-9)
    imu = sensors(model, 'imu')['imu_link']
    assert number(imu, 'update_rate') == pytest.approx(imu_rate)
    stddevs = [float(s.text) for s in imu.iter('stddev')]
    assert any(s > 0 for s in stddevs) == imu_noise


def test_tag_texture_only_on_the_texture_face():
    textured = [(v.get('name'), v.find('material/pbr/metal/albedo_map').text)
                for v in sdf().iter('visual')
                if v.find('material/pbr/metal/albedo_map') is not None]
    assert len(textured) == 1
    name, texture = textured[0]
    assert 'tag_rear_texture' in name
    assert texture == TEXTURE


@pytest.mark.parametrize('size', [0.5, 0.3])
def test_texture_face_sits_on_the_square(size):
    root = ET.fromstring(urdf(tag_size=size))
    joint = next(j for j in root.iter('joint') if j.find('child').get('link') == 'tag_rear_texture')
    assert joint.find('parent').get('link') == 'tag_rear'
    x = float(joint.find('origin').get('xyz').split()[0]) if joint.find('origin') is not None \
        else 0.0
    link = next(link for link in root.iter('link') if link.get('name') == 'tag_rear_texture')
    visual = link.find('visual')
    offset = float(visual.find('origin').get('xyz').split()[0]) \
        if visual.find('origin') is not None else 0.0
    box = [float(v) for v in visual.find('geometry/box').get('size').split()]
    assert box[1:] == pytest.approx([size, size])
    # Na frente do quadrado da F1.1, que ocupa x em [-0.002, 0] no frame da tag.
    centre = x + offset
    assert centre + box[0] / 2 <= -0.002 + 1e-6
    assert centre - box[0] / 2 >= -0.006


def test_tag_arguments_reach_the_description():
    root = ET.fromstring(urdf(tag_family='25h9', tag_id=3))
    assert any(m.get('name') == 'tag25h9_3' for m in root.iter('material'))


def test_ground_truth_pose_publisher():
    plugins = [p for p in sdf().iter('plugin') if 'pose-publisher' in p.get('filename', '')]
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.find('publish_model_pose').text == 'true'
    assert plugin.find('publish_link_pose').text == 'false'
    assert plugin.find('use_pose_vector_msg').text == 'false'


def test_bridge_entries():
    entries = yaml.safe_load((PACKAGE / 'config' / 'bridge.yaml').read_text())
    by_ros = {e['ros_topic_name']: e for e in entries}
    assert by_ros['camera_front/image_ideal']['gz_topic_name'] == \
        '/model/{name}/camera_front/image'
    assert 'camera_front/image_raw' not in by_ros
    assert by_ros['camera_front/camera_info']['gz_type_name'] == 'gz.msgs.CameraInfo'
    truth = by_ros['/ground_truth/{name}/pose']
    assert truth['gz_topic_name'] == '/model/{name}/pose'
    assert truth['ros_type_name'] == 'geometry_msgs/msg/PoseStamped'
    assert truth['gz_type_name'] == 'gz.msgs.Pose'
