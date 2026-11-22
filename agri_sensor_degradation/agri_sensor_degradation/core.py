"""Núcleo da degradação: gatilho, perda de mensagens e a base de cada sensor.

Python puro, sem `rclpy`: tudo aqui roda e é testado fora do ROS.
"""
import numpy as np

MODES = ('always', 'window', 'zone')


def point_in_polygon(x, y, polygon):
    """Diz se (x, y) está dentro do polígono, contando os cruzamentos de um raio (paridade)."""
    inside = False
    count = len(polygon)
    for i in range(count):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % count]
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


class Trigger:
    """Diz quando uma degradação está ativa: sempre, numa janela de tempo ou numa zona."""

    def __init__(self, mode='always', window=None, zone=None):
        if mode not in MODES:
            raise ValueError(f'modo de gatilho desconhecido: {mode!r}')
        self.mode = mode
        self.window = window
        self.zone = zone

    def active(self, t, position):
        """Ativo no instante `t` (s) e na posição real (x, y); a janela é [início, fim)."""
        if self.mode == 'always':
            return True
        if self.mode == 'window':
            start, end = self.window
            return start <= t < end
        if position is None:
            return False
        return point_in_polygon(position[0], position[1], self.zone)


class Loss:
    """Perda de mensagens: com o gatilho ativo, perde a fração `fraction` delas."""

    def __init__(self, enabled=False, fraction=1.0, trigger=None):
        self.enabled = enabled
        self.fraction = fraction
        self.trigger = trigger if trigger is not None else Trigger('always')

    def drops(self, t, position, rng):
        """Decide se a mensagem se perde; a fração 1,0 perde sempre, sem sortear."""
        if not (self.enabled and self.trigger.active(t, position)):
            return False
        if self.fraction >= 1.0:
            return True
        return bool(rng.random() < self.fraction)


class SensorBase:
    """Base de um sensor: aplica a degradação e conta as mensagens recebidas e perdidas."""

    name = ''
    msg_type = None
    input_topic = ''
    output_topic = ''

    def __init__(self, seed=0):
        self.loss = Loss()
        self.received = 0
        self.dropped = 0
        self._active = False
        self.reseed(seed)

    def reseed(self, seed):
        """Reinicia a sequência aleatória a partir da semente."""
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def degrade(self, msg, t, position):
        """Gancho da degradação do sinal; nesta versão é a identidade."""
        return msg

    def apply(self, msg, t, position):
        """Devolve a mensagem degradada, ou `None` se ela se perdeu."""
        self.received += 1
        self._active = self.loss.enabled and self.loss.trigger.active(t, position)
        if self.loss.drops(t, position, self.rng):
            self.dropped += 1
            return None
        return self.degrade(msg, t, position)

    def state(self):
        """Descreve o estado como um dicionário de textos."""
        return {
            'loss_enabled': str(self.loss.enabled).lower(),
            'loss_active': str(self._active).lower(),
            'trigger': self.loss.trigger.mode,
            'fraction': str(float(self.loss.fraction)),
            'received': str(self.received),
            'dropped': str(self.dropped),
        }
