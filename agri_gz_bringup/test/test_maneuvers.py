"""Testes das contas do módulo de manobras (F1.7, R4), sem simulação.

`agri_gz_bringup.maneuvers`:
- `fit_circle(points)` ajusta um círculo por mínimos quadrados a pontos (x, y) e devolve
  `(cx, cy, raio, desvio)`, em que o desvio é a maior distância de um ponto ao círculo;
- `stopping(samples)` recebe as amostras `(t, x, y)` a partir do comando de frenagem e devolve
  `(v0, distância, duração)`: a velocidade no início, o caminho percorrido até a velocidade ficar
  abaixo de 0,05 m/s e o tempo que isso levou.

O README do `agri_gazebo` tem a seção da validação, com a tabela gerada pelo `validate_physics`.
"""
import math
from pathlib import Path

import numpy as np
import pytest

from agri_gz_bringup.maneuvers import fit_circle, stopping

README = Path(__file__).resolve().parents[2] / 'README.md'


def test_fit_circle():
    rng = np.random.default_rng(0)
    angles = np.linspace(0.0, 2 * math.pi, 200)
    points = np.c_[10 + 24 * np.cos(angles), -5 + 24 * np.sin(angles)]
    cx, cy, radius, deviation = fit_circle(points + rng.normal(0.0, 0.01, points.shape))
    assert (cx, cy, radius) == pytest.approx((10.0, -5.0, 24.0), abs=0.01)
    assert deviation < 0.05


def test_fit_circle_reports_the_worst_point():
    angles = np.linspace(0.0, 2 * math.pi, 100, endpoint=False)
    points = np.c_[12 * np.cos(angles), 12 * np.sin(angles)]
    points[10] *= 1.05
    assert fit_circle(points)[3] == pytest.approx(0.6, abs=0.05)


def test_stopping_at_constant_deceleration():
    v0, a, dt = 11.1, 3.0, 0.05
    samples, t, x, v = [], 0.0, 0.0, v0
    while t < 6.0:
        samples.append((t, x, 0.0))
        step = max(0.0, v - a * dt)
        x += (v + step) / 2 * dt
        v, t = step, t + dt
    start, distance, duration = stopping(samples)
    assert start == pytest.approx(v0, rel=0.02)
    assert distance == pytest.approx(v0 ** 2 / (2 * a), rel=0.02)
    assert duration == pytest.approx(v0 / a, abs=0.1)


def test_readme_records_the_validation():
    text = README.read_text()
    assert '## Validação a 40 km/h' in text
    section = text.split('## Validação a 40 km/h')[1].split('\n## ')[0]
    for needle in ('validate_physics', '11,1', '3,0 m/s²', 'rolagem', 'autonomous_tractor',
                   'pickup', 'relevo'):
        assert needle in section, needle
