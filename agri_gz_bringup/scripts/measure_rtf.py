#!/usr/bin/env python3
"""Mede o fator de tempo real do comboio sem GUI: tempo simulado (/clock) sobre tempo de relógio.

    ros2 run agri_gz_bringup measure_rtf.py [--seconds 30] [--world citrus_farm.sdf]

Sobe o `convoy.launch.py` com `followers:=4` (1 líder e 4 tratores com câmera), espera o
primeiro /clock e só então começa a contar.
"""
import argparse
import os
import random
import signal
import subprocess
import time

import rclpy
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30.0,
                        help='duração da medida, em s de relógio')
    parser.add_argument('--world', default='citrus_farm.sdf')
    parser.add_argument('--startup-timeout', type=float, default=300.0)
    args = parser.parse_args()

    env = dict(os.environ, ROS_DOMAIN_ID=str(random.randint(30, 99)))
    env.pop('DISPLAY', None)
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    launch = subprocess.Popen(
        ['ros2', 'launch', 'agri_gz_bringup', 'convoy.launch.py', 'gui:=false',
         'followers:=4', f'world:={args.world}'],
        env=env, start_new_session=True,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('measure_rtf')
    clocks = []
    node.create_subscription(
        Clock, '/clock',
        lambda m: clocks.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 10)
    try:
        start = time.monotonic()
        while not clocks and time.monotonic() - start < args.startup_timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not clocks:
            raise SystemExit('o /clock não começou')
        wall0, sim0 = clocks[0]
        while time.monotonic() - wall0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        wall1, sim1 = clocks[-1]
        print(f'world={args.world} followers=4 sim={sim1 - sim0:.2f}s '
              f'wall={wall1 - wall0:.2f}s rtf={(sim1 - sim0) / (wall1 - wall0):.3f}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)


if __name__ == '__main__':
    main()
