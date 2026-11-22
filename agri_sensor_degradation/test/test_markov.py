"""Testes do erro correlacionado e dos saltos com faixa (F1.4b, R1 e R2), em Python puro.

`markov`: Gauss–Markov de 1ª ordem, um σ por grandeza e `tau` (s), discretizado exato, começando
no regime estacionário. Com ele ativo, os `sigmas` passam a ser o σ total,
√(σ_ruído² + σ_markov²).

`jumps`: a amplitude é sorteada entre `min_fraction` × o valor e o valor; a duração, entre
`duration_min` e `duration` (`duration_min` 0 vale como duração fixa).
"""
import numpy as np
import pytest

from agri_sensor_degradation.core import Channel, SensorBase, Trigger


class Sample:

    def __init__(self, a):
        self.a = np.asarray(a, dtype=float)
        self.sigmas = None


class ToySensor(SensorBase):
    name = 'toy'
    channels = {'a': Channel(size=2, full_scale=1.0)}

    def read(self, msg):
        return {'a': msg.a.copy()}

    def write(self, msg, values, sigmas, t):
        out = Sample(values['a'])
        out.sigmas = sigmas
        return out


def errors(sensor, count, dt=1.0, start=0.0):
    """O erro de cada saída para uma entrada fixa em (1, 2)."""
    out = []
    for k in range(count):
        res = sensor.apply(Sample((1.0, 2.0)), start + k * dt, None)
        out.append(res.a - (1.0, 2.0))
    return np.array(out)


def test_markov_value_names():
    assert set(ToySensor(seed=0).effects['markov'].values) == {'a', 'tau'}
    assert ToySensor(seed=0).effects['markov'].values['tau'] == pytest.approx(60.0)


def test_markov_sigma_and_time_constant():
    sensor = ToySensor(seed=4)
    sensor.set_effect('markov', a=0.3, tau=20.0)
    e = errors(sensor, 60000)[:, 0]
    assert e.std() == pytest.approx(0.3, rel=0.05)
    assert abs(e.mean()) < 0.03
    lag = 20
    rho = np.corrcoef(e[:-lag], e[lag:])[0, 1]
    assert rho == pytest.approx(np.exp(-1.0), abs=0.05)
    # Correlacionado, não branco: um passo de 1 s quase não muda o erro.
    assert np.corrcoef(e[:-1], e[1:])[0, 1] > 0.9


def test_markov_starts_stationary():
    first = []
    for seed in range(400):
        sensor = ToySensor(seed=seed)
        sensor.set_effect('markov', a=0.3, tau=60.0)
        first.append(errors(sensor, 1)[0, 0])
    assert np.std(first) == pytest.approx(0.3, rel=0.15)


def test_markov_restarts_when_inactive():
    sensor = ToySensor(seed=1)
    sensor.set_effect('markov', trigger=Trigger('window', window=(0.0, 10.0)), a=0.3)
    errors(sensor, 10)
    msg = Sample((1.0, 2.0))
    assert sensor.apply(msg, 20.0, None) is msg


def test_markov_is_seeded():
    def run(seed):
        sensor = ToySensor(seed=seed)
        sensor.set_effect('markov', a=0.3, tau=20.0)
        return errors(sensor, 50).tolist()
    assert run(3) == run(3)
    assert run(3) != run(4)


def test_sigmas_are_the_total_sigma():
    sensor = ToySensor(seed=1)
    sensor.set_effect('markov', a=0.3)
    assert sensor.apply(Sample((1.0, 2.0)), 0.0, None).sigmas['a'] == pytest.approx([0.3, 0.3])
    sensor.set_effect('noise', a=0.4)
    assert sensor.apply(Sample((1.0, 2.0)), 1.0, None).sigmas['a'] == pytest.approx([0.5, 0.5])


def jump_runs(sensor, seconds, hz=10):
    """(amplitude, duração em mensagens) de cada salto, separados pelo deslocamento."""
    runs, current, count = [], None, 0
    for k in range(round(seconds * hz)):
        msg = Sample((1.0, 2.0))
        res = sensor.apply(msg, k / hz, None)
        offset = res.a - msg.a
        jumping = np.linalg.norm(offset) > 1e-9
        if jumping and current is not None and np.allclose(offset, current):
            count += 1
            continue
        if count:
            runs.append((float(np.linalg.norm(current)), count))
        current, count = (offset, 1) if jumping else (None, 0)
    return runs


def test_jumps_with_ranges():
    sensor = ToySensor(seed=6)
    sensor.set_effect('jumps', rate=0.05, a=2.0, min_fraction=0.25, duration=3.0,
                      duration_min=1.0)
    runs = jump_runs(sensor, 20000.0)
    amplitudes = np.array([a for a, _ in runs])
    lengths = np.array([n for _, n in runs])
    assert len(runs) > 300
    assert amplitudes.min() >= 0.5 - 1e-9 and amplitudes.max() <= 2.0 + 1e-9
    assert amplitudes.std() > 0.3
    assert lengths.min() >= 9 and lengths.max() <= 31
    assert (lengths < 15).any() and (lengths > 25).any()


def test_jumps_defaults_keep_the_fixed_jump():
    sensor = ToySensor(seed=6)
    values = sensor.effects['jumps'].values
    assert values['min_fraction'] == pytest.approx(1.0)
    assert values['duration_min'] == pytest.approx(0.0)
    sensor.set_effect('jumps', rate=0.05, a=2.0, duration=2.0)
    runs = jump_runs(sensor, 5000.0)
    assert runs and all(a == pytest.approx(2.0) for a, _ in runs)
    assert all(19 <= n <= 21 for _, n in runs)
