"""Núcleo da degradação: gatilho, perda de mensagens e a base de cada sensor.

Python puro, sem `rclpy`: tudo aqui roda e é testado fora do ROS.
"""
import numpy as np

MODES = ('always', 'window', 'zone')
EFFECTS = ('scale', 'bias', 'drift', 'markov', 'noise', 'jumps', 'freeze', 'rate', 'delay')

# Valores de um efeito que não são por grandeza, com o padrão de cada um. O `jumps` tem também
# a amplitude por grandeza; o `markov`, o σ por grandeza.
SCALAR_VALUES = {
    'markov': {'tau': 60.0},
    'jumps': {'rate': 0.0, 'duration': 1.0, 'min_fraction': 1.0, 'duration_min': 0.0},
    'rate': {'max_hz': 0.0},
    'delay': {'seconds': 0.0},
    'freeze': {},
}


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


class Channel:
    """Uma grandeza do sensor: quantos números ela tem e o fundo de escala."""

    def __init__(self, size=1, full_scale=1.0):
        self.size = size
        self.full_scale = full_scale


class Effect:
    """Um efeito: desligado de início, com o próprio gatilho e os valores que ele usa."""

    def __init__(self, name, channels, defaults=None):
        self.name = name
        self.enabled = False
        self.trigger = Trigger('always')
        if defaults is not None:
            self.values = dict(defaults)
            return
        if name in SCALAR_VALUES:
            self.values = dict(SCALAR_VALUES[name])
        else:
            self.values = {}
        if name not in ('rate', 'delay', 'freeze'):
            for ch in channels:
                self.values[ch] = 1.0 if name == 'scale' else 0.0


class SensorBase:
    """Base de um sensor: aplica a degradação e conta as mensagens recebidas e perdidas."""

    name = ''
    msg_type = None
    input_topic = ''
    output_topic = ''
    channels = {}
    # Efeitos próprios do sensor: nome -> valores padrão (sem grandeza).
    extra_effects = {}

    def __init__(self, seed=0):
        self.loss = Loss()
        self.effects = {name: Effect(name, self.channels) for name in EFFECTS}
        for name, defaults in self.extra_effects.items():
            self.effects[name] = Effect(name, self.channels, defaults)
        self._effect_active = {name: False for name in self.effects}
        self._drift_start = None
        self._markov = None
        self._jump = None
        self._last_t = None
        self._last_values = None
        self._last_msg = None
        self._last_out_t = None
        self._queue = []
        self.received = 0
        self.dropped = 0
        self._active = False
        self.reseed(seed)

    def reseed(self, seed):
        """Reinicia a sequência aleatória a partir da semente."""
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def set_effect(self, name, trigger=None, **values):
        """Liga um efeito, com o gatilho dado (padrão: sempre) e os valores que mudam."""
        if name not in self.effects:
            raise ValueError(f'efeito desconhecido: {name!r}')
        effect = self.effects[name]
        for key in values:
            if key not in effect.values:
                raise ValueError(f'valor desconhecido no efeito {name!r}: {key!r}')
        effect.values.update(values)
        effect.trigger = trigger if trigger is not None else Trigger('always')
        effect.enabled = True

    def read(self, msg):
        """Gancho: devolve os números da mensagem, `{grandeza: vetor}`."""
        raise NotImplementedError

    def write(self, msg, values, sigmas, t):
        """Gancho: grava os números degradados numa mensagem nova."""
        raise NotImplementedError

    def idle(self, msg, t):
        """Gancho: chamado quando a mensagem passa sem nenhum efeito ativo."""

    def extra_chain(self, values, t, dt, active):
        """Gancho: aplica os efeitos próprios no fim da cadeia; o padrão devolve `values`."""
        return values

    def weights(self, channel):
        """Peso de cada eixo da grandeza nos efeitos aditivos (padrão: uns)."""
        return np.ones(self.channels[channel].size)

    def _active_of(self, name, t, position):
        effect = self.effects[name]
        return effect.enabled and effect.trigger.active(t, position)

    def apply(self, msg, t, position):
        """Devolve a mensagem degradada, ou `None` se ela se perdeu ou ficou retida."""
        self.received += 1
        self._active = self.loss.enabled and self.loss.trigger.active(t, position)
        if self.loss.drops(t, position, self.rng):
            self.dropped += 1
            return None
        active = {name: self._active_of(name, t, position) for name in self.effects}
        self._effect_active = active
        if not active['drift']:
            self._drift_start = None
        if not active['markov']:
            self._markov = None
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if not any(active.values()):
            self._last_values = None
            self._last_msg = msg
            self.idle(msg, t)
            return msg
        sigmas = None
        if active['freeze']:
            values = self._frozen_values(msg)
        else:
            values, sigmas = self._chain(msg, t, dt, active)
            self._last_values = {k: v.copy() for k, v in values.items()}
            self._last_msg = None
        out = self.write(msg, values, sigmas, t)
        if active['rate']:
            max_hz = self.effects['rate'].values['max_hz']
            if (max_hz > 0 and self._last_out_t is not None
                    and t - self._last_out_t < 1.0 / max_hz - 1e-9):
                return None
        self._last_out_t = t
        if active['delay']:
            self._queue.append((t + self.effects['delay'].values['seconds'], out))
            return None
        return out

    def release(self, t):
        """Devolve, em ordem, as mensagens retidas pelo atraso cuja hora já chegou."""
        due = [item for item in self._queue if item[0] <= t]
        self._queue = [item for item in self._queue if item[0] > t]
        return [out for _, out in due]

    def _frozen_values(self, msg):
        if self._last_values is None:
            source = self._last_msg if self._last_msg is not None else msg
            self._last_values = self.read(source)
        return {k: v.copy() for k, v in self._last_values.items()}

    def _chain(self, msg, t, dt, active):
        """Escala, viés, deriva, Markov, ruído, saltos e extras; devolve (valores, sigmas)."""
        values = {k: np.asarray(v, dtype=float).copy() for k, v in self.read(msg).items()}
        weights = {k: np.asarray(self.weights(k), dtype=float) for k in values}
        if active['scale']:
            for k in values:
                values[k] = values[k] * self.effects['scale'].values.get(k, 1.0)
        if active['bias']:
            for k in values:
                values[k] = values[k] + self.effects['bias'].values.get(k, 0.0) * weights[k]
        if active['drift']:
            if self._drift_start is None:
                self._drift_start = t
            elapsed = t - self._drift_start
            for k in values:
                rate = self.effects['drift'].values.get(k, 0.0)
                values[k] = values[k] + rate * elapsed * weights[k]
        sigmas = None
        variances = {k: np.zeros_like(weights[k]) for k in values}
        if active['markov']:
            self._markov_step(values, dt, weights)
            for k in values:
                variances[k] = variances[k] + (self.effects['markov'].values.get(k, 0.0)
                                               * weights[k]) ** 2
        if active['noise']:
            for k in values:
                sigma = self.effects['noise'].values.get(k, 0.0) * weights[k]
                variances[k] = variances[k] + sigma ** 2
                values[k] = values[k] + self.rng.normal(size=values[k].shape) * sigma
        if active['noise'] or active['markov']:
            sigmas = {k: np.sqrt(variances[k]) for k in values}
        if active['jumps']:
            offsets = self._jump_offsets(t, dt, weights)
            if offsets is not None:
                for k in values:
                    values[k] = values[k] + offsets[k]
        values = self.extra_chain(values, t, dt, active)
        return values, sigmas

    def _markov_step(self, values, dt, weights):
        """Soma o erro de Gauss–Markov; o primeiro passo sorteia o regime estacionário."""
        markov = self.effects['markov'].values
        tau = markov['tau']
        a = float(np.exp(-dt / tau)) if tau > 0 else 0.0
        first = self._markov is None
        if first:
            self._markov = {}
        for k in values:
            sigma = markov.get(k, 0.0) * weights[k]
            noise = self.rng.normal(size=values[k].shape)
            if first:
                self._markov[k] = sigma * noise
            else:
                self._markov[k] = self._markov[k] * a + sigma * np.sqrt(1.0 - a * a) * noise
            values[k] = values[k] + self._markov[k]

    def _jump_offsets(self, t, dt, weights):
        """Deslocamento do salto em curso; sorteia um novo se não há salto e a sorte pedir."""
        if self._jump is not None and t >= self._jump[0]:
            self._jump = None
        if self._jump is None:
            rate = self.effects['jumps'].values['rate']
            if rate > 0 and self.rng.random() < 1.0 - np.exp(-rate * dt):
                jumps = self.effects['jumps'].values
                factor = 1.0
                if jumps['min_fraction'] < 1.0:
                    factor = self.rng.uniform(jumps['min_fraction'], 1.0)
                length = jumps['duration']
                if jumps['duration_min'] > 0:
                    length = self.rng.uniform(jumps['duration_min'], length)
                offsets = {}
                for k, channel in self.channels.items():
                    amplitude = jumps.get(k, 0.0) * factor
                    offsets[k] = self._direction(channel.size) * amplitude * weights[k]
                self._jump = (t + length, offsets)
        return None if self._jump is None else self._jump[1]

    def _direction(self, size):
        """Direção sorteada de norma 1: normal normalizada, ou ±1 em um eixo."""
        if size == 1:
            return np.array([self.rng.choice([-1.0, 1.0])])
        vector = self.rng.normal(size=size)
        return vector / np.linalg.norm(vector)

    def state(self):
        """Descreve o estado como um dicionário de textos."""
        state = {
            'loss_enabled': str(self.loss.enabled).lower(),
            'loss_active': str(self._active).lower(),
            'trigger': self.loss.trigger.mode,
            'fraction': str(float(self.loss.fraction)),
            'received': str(self.received),
            'dropped': str(self.dropped),
        }
        for name in self.effects:
            state[f'{name}_enabled'] = str(self.effects[name].enabled).lower()
            state[f'{name}_active'] = str(self._effect_active[name]).lower()
        return state
