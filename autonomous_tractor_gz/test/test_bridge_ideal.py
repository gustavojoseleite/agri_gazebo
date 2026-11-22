"""A ponte do trator entrega o GNSS e a IMU nos tópicos ideais (F1.3, R2).

Os nós `gnss` e `imu` do veículo recebem o sinal ideal e publicam nos tópicos normais, então a
ponte não pode publicar direto em `gnss/fix` nem em `imu/data`.
"""
from pathlib import Path

import pytest
import yaml

BRIDGE = Path(__file__).resolve().parents[1] / 'config' / 'bridge.yaml'


@pytest.mark.parametrize('gz, ideal, normal', [
    ('/model/{name}/navsat', '_sim/gnss/fix', 'gnss/fix'),
    ('/model/{name}/imu', '_sim/imu/data', 'imu/data'),
])
def test_sensor_goes_to_the_ideal_topic(gz, ideal, normal):
    entries = yaml.safe_load(BRIDGE.read_text())
    by_gz = {e['gz_topic_name']: e['ros_topic_name'] for e in entries}
    assert by_gz[gz] == ideal
    assert normal not in [e['ros_topic_name'] for e in entries]
