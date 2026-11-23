"""Manobras roteirizadas para medir a física de um veículo na simulação, sem tela.

`Session` sobe o veículo sozinho (`sim.launch.py` do `<robot>_gz`, com `name:=v` e
`namespace:=v`) num domínio ROS e numa partição do Gazebo próprios, e comanda `/v/cmd_vel`
lendo a pose real em `/ground_truth/v/pose`. Os laços contam o tempo pelo carimbo dessa pose,
isto é, em tempo simulado.

    with Session('pickup') as session:
        reta = session.straight(speed=11.1, hold=8.0)
        curva = session.circle(radius=30.0, speed=11.1)

`fit_circle` e `stopping` são as contas puras, sem simulação.
"""
import math
import os
import random
import signal
import subprocess
import time

import numpy as np

# Quanto esperar o veículo aparecer, em s de relógio, e o limite de cada manobra.
STARTUP_TIMEOUT = 240.0
MANEUVER_TIMEOUT = 900.0
# Velocidade abaixo da qual o veículo conta como parado, em m/s.
STOPPED = 0.05


def fit_circle(points):
    """Ajusta um círculo por mínimos quadrados a pontos (x, y).

    Devolve `(cx, cy, raio, desvio)`, em que o desvio é a maior distância de um ponto ao círculo.
    """
    pts = np.asarray(points, dtype=float)
    design = np.c_[2 * pts, np.ones(len(pts))]
    target = (pts ** 2).sum(axis=1)
    cx, cy, c = np.linalg.lstsq(design, target, rcond=None)[0]
    radius = math.sqrt(c + cx * cx + cy * cy)
    deviation = float(np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - radius).max())
    return float(cx), float(cy), radius, deviation


def stopping(samples, window=0.1):
    """Mede a frenagem a partir de amostras `(t, x, y)` que começam no comando de parada.

    Devolve `(v0, distância, duração)`: a velocidade no início, o caminho percorrido até a
    velocidade (média em `window` s) ficar abaixo de 0,05 m/s e o tempo que isso levou.
    """
    data = list(samples)
    first = next((k for k in range(1, len(data)) if data[k][0] - data[0][0] >= 0.049), None)
    if first is None:
        return 0.0, 0.0, 0.0
    v0 = math.hypot(data[first][1] - data[0][1], data[first][2] - data[0][2]) \
        / (data[first][0] - data[0][0])
    stop = len(data) - 1
    for i in range(len(data) - 1):
        k = next((k for k in range(i + 1, len(data)) if data[k][0] - data[i][0] >= window), None)
        if k is None:
            break
        speed = math.hypot(data[k][1] - data[i][1], data[k][2] - data[i][2]) \
            / (data[k][0] - data[i][0])
        if speed < STOPPED:
            stop = i
            break
    distance = sum(math.hypot(data[i + 1][1] - data[i][1], data[i + 1][2] - data[i][2])
                   for i in range(stop))
    return v0, distance, data[stop][0] - data[0][0]


def _roll_pitch(q):
    """Rolagem e arfagem, em graus, de um quatérnio (geometry_msgs)."""
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class Session:
    """Um veículo simulado, pronto para ser comandado. Use como gerenciador de contexto."""

    def __init__(self, robot, world='empty.sdf'):
        self.robot = robot
        self.world = world
        self._launch = None
        self._context = None
        self._node = None
        self._executor = None
        self._publisher = None
        self._queue = []

    def __enter__(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor

        domain = random.randint(30, 99)
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                   GZ_PARTITION=f'manobra_{random.getrandbits(32):08x}')
        env.pop('DISPLAY', None)
        self._launch = subprocess.Popen(
            ['ros2', 'launch', f'{self.robot}_gz', 'sim.launch.py', 'gui:=false', 'name:=v',
             'namespace:=v', f'world:={self.world}'],
            env=env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            self._context = Context()
            rclpy.init(context=self._context, domain_id=domain)
            self._executor = SingleThreadedExecutor(context=self._context)
            self._node = rclpy.create_node('maneuvers', context=self._context)
            self._executor.add_node(self._node)
            self._wait_for_vehicle()
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def _wait_for_vehicle(self):
        from geometry_msgs.msg import PoseStamped, Twist
        from rclpy.qos import qos_profile_sensor_data

        self._publisher = self._node.create_publisher(Twist, '/v/cmd_vel', 10)
        start = time.monotonic()
        truth = None
        while time.monotonic() - start < STARTUP_TIMEOUT:
            self._executor.spin_once(timeout_sec=0.5)
            if self._launch.poll() is not None:
                raise RuntimeError('o launch do veículo terminou antes da hora')
            topics = dict(self._node.get_topic_names_and_types())
            truth = next((t for t in topics if 'ground_truth' in t), None)
            if truth and self._node.count_subscribers('/v/cmd_vel') > 0:
                break
        else:
            raise TimeoutError('o veículo não apareceu (ground_truth e /v/cmd_vel)')
        self._node.create_subscription(PoseStamped, truth, self._on_pose,
                                       qos_profile_sensor_data)
        # Os controladores ficam ativos só depois da criação do veículo, e um `cmd_vel` enviado
        # antes é descartado: anda devagar até a pose real se mexer, e então para.
        cmd = Twist()
        cmd.linear.x = 0.5
        first = None
        start = time.monotonic()
        while time.monotonic() - start < STARTUP_TIMEOUT:
            self._executor.spin_once(timeout_sec=0.05)
            self._publisher.publish(cmd)
            if self._queue:
                if first is None:
                    first = self._queue[0]
                last = self._queue[-1]
                self._queue.clear()
                if math.hypot(last[1] - first[1], last[2] - first[2]) > 0.2:
                    break
        else:
            raise TimeoutError('o veículo não respondeu ao cmd_vel')
        settle = time.monotonic()
        while time.monotonic() - settle < 5.0:
            self._executor.spin_once(timeout_sec=0.05)
            self._publisher.publish(Twist())
        self._queue.clear()

    def _on_pose(self, msg):
        roll, pitch = _roll_pitch(msg.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._queue.append((stamp, msg.pose.position.x, msg.pose.position.y,
                            msg.pose.position.z, roll, pitch))

    def __exit__(self, *exc):
        import rclpy

        if self._launch is not None:
            try:
                os.killpg(self._launch.pid, signal.SIGINT)
                self._launch.wait(timeout=30)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            try:
                os.killpg(self._launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._launch = None
        if self._context is not None:
            if self._executor is not None:
                self._executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
            rclpy.shutdown(context=self._context)
            self._context = None
        return False

    def _drive(self, duration, linear, angular, collect):
        """Publica `cmd_vel` por `duration` s simulados e entrega cada amostra a `collect`."""
        from geometry_msgs.msg import Twist

        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        wall = time.monotonic()
        end = None
        while True:
            # O comando vai antes de ler a pose, para a nova fase valer desde a próxima amostra.
            self._publisher.publish(cmd)
            self._executor.spin_once(timeout_sec=0.02)
            # `spin_once` atende um evento só: sem esvaziar a fila, as amostras se acumulam e o
            # comando passa a chegar atrasado em relação ao tempo simulado.
            for _ in range(200):
                queued = len(self._queue)
                self._executor.spin_once(timeout_sec=0.0)
                if len(self._queue) == queued:
                    break
            if time.monotonic() - wall > MANEUVER_TIMEOUT:
                raise TimeoutError('a manobra não terminou a tempo')
            samples, self._queue = self._queue, []
            for sample in samples:
                if end is None:
                    end = sample[0] + duration
                collect(sample)
                if sample[0] >= end:
                    return

    def straight(self, speed, hold):
        """Acelera, mantém `speed` por `hold` s e freia até parar."""
        ramp = speed / 1.5 + 2.0
        everything = []
        held = []
        brake = []

        def collect_all(sample):
            everything.append(sample)

        def collect_held(sample):
            everything.append(sample)
            if not held or sample[0] - held[-1][0] >= 0.25:
                held.append(sample)

        def collect_brake(sample):
            everything.append(sample)
            brake.append(sample)

        self._drive(ramp, speed, 0.0, collect_all)
        self._drive(hold, speed, 0.0, collect_held)
        self._drive(speed / 2 + 4.0, 0.0, 0.0, collect_brake)
        speeds = [math.hypot(b[1] - a[1], b[2] - a[2]) / (b[0] - a[0])
                  for a, b in zip(held, held[1:])]
        # A frenagem parte da última amostra mantida.
        start = [held[-1]] + brake
        v0, distance, duration = stopping([(s[0], s[1], s[2]) for s in start])
        return {
            'v_min': min(speeds), 'v_max': max(speeds),
            'brake_from': v0, 'stop_distance': distance, 'stop_time': duration,
            'z_min': min(s[3] for s in everything), 'z_max': max(s[3] for s in everything),
            'max_roll': max(abs(s[4]) for s in everything),
            'max_pitch': max(abs(s[5]) for s in everything),
        }

    def circle(self, radius, speed):
        """Faz uma volta completa a `speed`, depois de acelerar em reta e estabilizar."""
        points = []
        rolls = []

        def collect(sample):
            points.append((sample[1], sample[2]))
            rolls.append(abs(sample[4]))

        ignore = []
        self._drive(speed / 1.5 + 2.0, speed, 0.0, ignore.append)
        self._drive(4.0, speed, speed / radius, ignore.append)
        self._drive(2 * math.pi * radius / speed, speed, speed / radius, collect)
        self._drive(speed / 2 + 4.0, 0.0, 0.0, ignore.append)
        _, _, fitted, deviation = fit_circle(points)
        return {'radius': fitted, 'deviation': deviation, 'max_roll': max(rolls)}
