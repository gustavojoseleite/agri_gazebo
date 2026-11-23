"""
O IMU dá a orientação no referencial ENU do mundo, e não relativa à pose inicial. Marca
`simulation`.

Sobe o `convoy.launch.py` sem GUI, com a pickup e um trator virados para o norte
(`yaw:=1.5708`). O rumo da orientação de `imu/data` de cada veículo fica a menos de 3 graus do rumo
da pose real. Sem o referencial ENU no sensor, o Gazebo dá a orientação relativa à pose em que o
veículo nasceu, e o rumo sai perto de zero.
"""
import math
import os
import random
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

pytestmark = pytest.mark.simulation

VEHICLES = ('pickup', 'tractor_1')
WALL_TIMEOUT = 300.0


def heading(q):
    """Devolve o rumo, em rad, de um quatérnio."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def test_imu_heading_is_in_the_world_frame():
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)),
               GZ_PARTITION=f'teste_{random.getrandbits(32):08x}')
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    launch = subprocess.Popen(['ros2', 'launch', 'agri_gz_bringup', 'convoy.launch.py',
                               'gui:=false', 'followers:=1', 'yaw:=1.5708',
                               'sensor_profile:=ideal'],
                              env=env, start_new_session=True, stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('test_sim_imu_heading')
    data = {}
    for vehicle in VEHICLES:
        for topic, kind in ((f'/{vehicle}/imu/data', Imu),
                            (f'/ground_truth/{vehicle}/pose', PoseStamped)):
            node.create_subscription(kind, topic,
                                     lambda msg, t=topic: data.setdefault(t, []).append(msg),
                                     qos_profile_sensor_data)
    try:
        start = time.monotonic()
        while (any(len(data.get(t, [])) < 20 for t in [f'/{v}/imu/data' for v in VEHICLES]
                   + [f'/ground_truth/{v}/pose' for v in VEHICLES])
               and time.monotonic() - start < WALL_TIMEOUT):
            rclpy.spin_once(node, timeout_sec=0.1)
        for vehicle in VEHICLES:
            imu = data.get(f'/{vehicle}/imu/data')
            truth = data.get(f'/ground_truth/{vehicle}/pose')
            assert imu and truth, f'{vehicle}: sem IMU ou pose real'
            real = heading(truth[-1].pose.orientation)
            measured = heading(imu[-1].orientation)
            error = math.degrees(math.atan2(math.sin(measured - real), math.cos(measured - real)))
            assert abs(real - math.pi / 2) < math.radians(3), vehicle
            assert abs(error) < 3.0, f'{vehicle}: rumo do IMU {math.degrees(measured):.1f} graus'
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
