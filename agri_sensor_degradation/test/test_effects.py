"""Testes dos efeitos gerais da degradação (F1.4a, R1), em Python puro.

A classe base aplica os efeitos em cadeia: escala, viés, deriva, erro correlacionado (`markov`,
F1.4b), ruído, saltos, congelamento, queda de taxa e atraso. Cada sensor diz as grandezas que tem
(`channels`), como lê os números da mensagem (`read`) e como grava os números degradados numa
mensagem nova (`write`). Aqui, um sensor de brinquedo com duas grandezas: `a` (2 eixos) e `b`
(1 eixo).

`set_effect(nome, trigger=None, **valores)` liga um efeito (gatilho padrão: sempre) e guarda os
valores; `effects[nome]` tem `enabled`, `trigger` e `values`. `release(t)` devolve as mensagens
atrasadas cuja hora chegou.
"""
import numpy as np
import pytest

from agri_sensor_degradation.core import EFFECTS, Channel, SensorBase, Trigger

HZ = 10


class Sample:
    """Mensagem de brinquedo."""

    def __init__(self, a, b=3.0):
        self.a = np.asarray(a, dtype=float)
        self.b = float(b)
        self.sigmas = 'original'


class ToySensor(SensorBase):
    name = 'toy'
    channels = {'a': Channel(size=2, full_scale=1.0), 'b': Channel(size=1, full_scale=10.0)}

    def read(self, msg):
        return {'a': msg.a.copy(), 'b': np.array([msg.b])}

    def write(self, msg, values, sigmas, t):
        out = Sample(values['a'], values['b'][0])
        out.sigmas = sigmas
        return out


def times(seconds, hz=HZ, start=0.0):
    return [start + k / hz for k in range(round(seconds * hz))]


def feed(sensor, seconds, hz=HZ, start=0.0, a=(1.0, 2.0)):
    """Passa mensagens iguais pelo sensor; devolve (t, entrada, saída) de cada uma."""
    out = []
    for t in times(seconds, hz, start):
        msg = Sample(a)
        out.append((t, msg, sensor.apply(msg, t, None)))
    return out


def test_effect_names_and_order():
    assert EFFECTS == ('scale', 'bias', 'drift', 'markov', 'noise', 'jumps', 'freeze', 'rate',
                       'delay')


def test_all_off_passes_the_same_message():
    sensor = ToySensor(seed=1)
    msg = Sample((1.0, 2.0))
    assert sensor.apply(msg, 0.0, None) is msg
    assert all(not sensor.effects[name].enabled for name in EFFECTS)


def test_scale():
    sensor = ToySensor(seed=1)
    sensor.set_effect('scale', a=1.5)
    out = sensor.apply(Sample((1.0, 2.0)), 0.0, None)
    assert out.a == pytest.approx([1.5, 3.0])
    assert out.b == pytest.approx(3.0)


def test_bias():
    sensor = ToySensor(seed=1)
    sensor.set_effect('bias', a=0.3, b=-1.0)
    out = sensor.apply(Sample((1.0, 2.0)), 0.0, None)
    assert out.a == pytest.approx([1.3, 2.3])
    assert out.b == pytest.approx(2.0)


def test_drift_grows_from_activation_and_stops_outside_the_trigger():
    sensor = ToySensor(seed=1)
    sensor.set_effect('drift', trigger=Trigger('window', window=(10.0, 20.0)), a=0.1)
    out = {round(t, 1): (msg, res) for t, msg, res in feed(sensor, 30.0)}
    msg, res = out[5.0]
    assert res is msg
    assert out[15.0][1].a == pytest.approx([1.5, 2.5], abs=1e-9)
    msg, res = out[25.0]
    assert res is msg


def test_noise_statistics_and_reported_sigma():
    sensor = ToySensor(seed=1)
    sensor.set_effect('noise', a=0.2)
    results = [res for _, _, res in feed(sensor, 2000.0)]
    a = np.array([r.a for r in results]) - np.array([1.0, 2.0])
    assert a.std(axis=0) == pytest.approx([0.2, 0.2], rel=0.03)
    assert np.abs(a.mean(axis=0)).max() < 0.01
    assert all(r.b == pytest.approx(3.0) for r in results)
    assert results[0].sigmas['a'] == pytest.approx([0.2, 0.2])
    assert results[0].sigmas['b'] == pytest.approx([0.0])


def test_without_noise_the_sigmas_are_none():
    sensor = ToySensor(seed=1)
    sensor.set_effect('bias', a=0.1)
    assert sensor.apply(Sample((1.0, 2.0)), 0.0, None).sigmas is None


def noisy(seed):
    sensor = ToySensor(seed=seed)
    sensor.set_effect('noise', a=0.2)
    return sensor, [res.a.tolist() for _, _, res in feed(sensor, 5.0)]


def test_noise_is_seeded():
    assert noisy(7)[1] == noisy(7)[1]
    assert noisy(7)[1] != noisy(8)[1]
    sensor, first = noisy(7)
    sensor.reseed(7)
    assert [res.a.tolist() for _, _, res in feed(sensor, 5.0)] == first


def test_jumps():
    sensor = ToySensor(seed=2)
    sensor.set_effect('jumps', rate=0.05, duration=2.0, a=3.0)
    results = feed(sensor, 10000.0)
    offsets = [res.a - msg.a if res is not msg else np.zeros(2) for _, msg, res in results]
    norms = np.array([np.linalg.norm(o) for o in offsets])
    jumping = norms > 1e-9
    assert norms[jumping] == pytest.approx(3.0, abs=1e-9)
    # Sem salto novo durante um salto: a fração do tempo em salto é 2 / (1/0,05 + 2) = 0,091.
    assert 0.075 < jumping.mean() < 0.107
    # Cada salto mantém o mesmo deslocamento por 2 s (20 mensagens a 10 Hz). Um salto pode
    # começar logo depois do anterior, então os saltos são separados pelo deslocamento.
    runs, count, current = [], 0, None
    for offset, flag in zip(offsets, jumping):
        if flag and current is not None and np.allclose(offset, current):
            count += 1
            continue
        if count:
            runs.append(count)
        count, current = (1, offset) if flag else (0, None)
    assert len(runs) > 300 and min(runs) >= 19 and max(runs) <= 21
    assert all(res is msg or res.b == pytest.approx(3.0) for _, msg, res in results)


def test_freeze_repeats_the_last_values():
    sensor = ToySensor(seed=1)
    sensor.set_effect('freeze', trigger=Trigger('window', window=(5.0, 8.0)))
    out = {}
    for t in times(10.0):
        msg = Sample((t, 2 * t))
        out[round(t, 1)] = (msg, sensor.apply(msg, t, None))
    assert out[6.0][1].a == pytest.approx([4.9, 9.8])
    assert out[7.9][1].a == pytest.approx([4.9, 9.8])
    msg, res = out[9.0]
    assert res is msg


def test_rate_limit():
    sensor = ToySensor(seed=1)
    sensor.set_effect('rate', max_hz=5.0)
    passed = [t for t, _, res in feed(sensor, 10.0, hz=20) if res is not None]
    assert 49 <= len(passed) <= 51
    assert min(b - a for a, b in zip(passed, passed[1:])) >= 0.2 - 1e-9


def test_delay_holds_the_message():
    sensor = ToySensor(seed=1)
    sensor.set_effect('delay', seconds=0.3)
    first, second = Sample((1.0, 2.0)), Sample((5.0, 6.0))
    assert sensor.apply(first, 1.0, None) is None
    assert sensor.apply(second, 1.1, None) is None
    assert sensor.release(1.29) == []
    released = sensor.release(1.31)
    assert len(released) == 1 and released[0].a == pytest.approx([1.0, 2.0])
    released = sensor.release(2.0)
    assert len(released) == 1 and released[0].a == pytest.approx([5.0, 6.0])
    assert sensor.release(3.0) == []


def test_each_effect_has_its_own_trigger():
    sensor = ToySensor(seed=1)
    sensor.set_effect('noise', trigger=Trigger('window', window=(5.0, 10.0)), a=0.2)
    sensor.set_effect('bias', b=1.0)
    out = {round(t, 1): res for t, _, res in feed(sensor, 15.0)}
    assert out[2.0].a == pytest.approx([1.0, 2.0])
    assert out[7.0].a != pytest.approx([1.0, 2.0])
    assert out[12.0].a == pytest.approx([1.0, 2.0])
    assert all(res.b == pytest.approx(4.0) for res in out.values())


def test_disabling_brings_the_message_back():
    sensor = ToySensor(seed=1)
    sensor.set_effect('noise', a=0.2)
    sensor.effects['noise'].enabled = False
    msg = Sample((1.0, 2.0))
    assert sensor.apply(msg, 0.0, None) is msg


def test_state_lists_each_effect():
    sensor = ToySensor(seed=1)
    sensor.set_effect('noise', trigger=Trigger('window', window=(5.0, 10.0)), a=0.1)
    sensor.apply(Sample((1.0, 2.0)), 7.0, None)
    state = sensor.state()
    assert (state['noise_enabled'], state['noise_active']) == ('true', 'true')
    assert (state['bias_enabled'], state['bias_active']) == ('false', 'false')
    assert all(f'{name}_enabled' in state and f'{name}_active' in state for name in EFFECTS)
    assert state['received'] == '1'


def test_set_effect_rejects_unknown_names():
    sensor = ToySensor(seed=1)
    with pytest.raises(ValueError):
        sensor.set_effect('blur', a=1.0)
    with pytest.raises(ValueError):
        sensor.set_effect('noise', z=1.0)
