"""Testes do relógio das pontes dos veículos (R1).

Num comboio, cada veículo tem a própria ponte, e cada ponte trazia o `/clock` do Gazebo: o
relógio chegava cinco vezes a cada passo de física. `bridge_config(..., clock=False)` tira a
entrada do `/clock` da ponte de um veículo; o comboio sobe uma ponte só para o relógio. Sozinho
(`sim.launch.py`), o veículo continua trazendo o relógio, que é o padrão.
"""
from agri_gz_bringup.vehicles import bridge_config

BRIDGE = """
- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
- ros_topic_name: "imu/data"
  gz_topic_name: "/model/{name}/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
"""


def topics(entries):
    """Os tópicos ROS das entradas."""
    return [e['ros_topic_name'] for e in entries]


def test_a_vehicle_alone_keeps_the_clock():
    assert topics(bridge_config(BRIDGE, 'pickup', 'single')) == ['/clock', 'imu/data']


def test_a_vehicle_in_a_convoy_has_no_clock():
    entries = bridge_config(BRIDGE, 'tractor_1', 'single', clock=False)
    assert topics(entries) == ['imu/data']
    assert entries[0]['gz_topic_name'] == '/model/tractor_1/imu'
