"""Teste de simulação do relógio único (R1). Marca `simulation`.

Sobe o comboio sem GUI com a pickup e dois tratores. Depois que as pontes dos três veículos
estão de pé (cada uma publica a pose real do seu veículo), o `/clock` tem de ter um único
publicador. Depois sobe um veículo sozinho (`sim.launch.py`), que continua com o próprio relógio:
também um publicador.
"""
import os
import random
import signal
import subprocess
import time

import pytest
import rclpy

pytestmark = pytest.mark.simulation

WALL_TIMEOUT = 240.0


def clock_publishers(command, ready_topics):
    """Sobe `command`, espera os `ready_topics` terem publicador e conta os do `/clock`."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)))
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    process = subprocess.Popen(command, env=env, start_new_session=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('test_sim_clock')
    try:
        start = time.monotonic()
        while time.monotonic() - start < WALL_TIMEOUT:
            rclpy.spin_once(node, timeout_sec=0.5)
            if all(node.count_publishers(t) > 0 for t in ready_topics):
                break
        ready = {t: node.count_publishers(t) for t in ready_topics}
        # Uma ponte atrasada ainda poderia trazer o relógio: espera mais um pouco.
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.5)
        return ready, node.count_publishers('/clock')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)


def test_convoy_has_a_single_clock():
    vehicles = ('pickup', 'tractor_1', 'tractor_2')
    ready, clocks = clock_publishers(
        ['ros2', 'launch', 'agri_gz_bringup', 'convoy.launch.py', 'gui:=false',
         'followers:=2'],
        [f'/ground_truth/{v}/pose' for v in vehicles])
    assert all(ready.values()), ready
    assert clocks == 1


def test_a_vehicle_alone_keeps_its_clock():
    ready, clocks = clock_publishers(
        ['ros2', 'launch', 'agri_gz_bringup', 'sim.launch.py', 'robot:=pickup', 'gui:=false'],
        ['/ground_truth/pickup/pose'])
    assert all(ready.values()), ready
    assert clocks == 1
