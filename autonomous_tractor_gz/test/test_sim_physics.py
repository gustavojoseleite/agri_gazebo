"""Teste de simulação da física a 40 km/h no trator (F1.7, R1 a R3). Marca `simulation`.

`Session(robot, world)` sobe o veículo sozinho, sem GUI, num domínio e numa partição próprios, e
comanda `cmd_vel` lendo a pose real (`/ground_truth/...`):
- `straight(speed, hold)` acelera, mantém `hold` s e freia até parar. Devolve as velocidades no
  trecho mantido (`v_min`, `v_max`), a frenagem (`brake_from`, `stop_distance`), as altitudes
  (`z_min`, `z_max`) e as inclinações máximas (`max_roll`, `max_pitch`, em graus);
- `circle(radius, speed)` faz uma volta completa depois de estabilizar. Devolve `radius`,
  `deviation` e `max_roll`.
"""
import pytest

from agri_gz_bringup.maneuvers import Session

pytestmark = pytest.mark.simulation

# Raios da seção 5.6, cada um no limite √(a_lat·R) com a_lat = 1,0 m/s², e 30 m a 40 km/h.
CURVES = [(6.0, 2.45), (12.0, 3.46), (24.0, 4.90), (30.0, 5.48), (30.0, 11.1)]


@pytest.fixture(scope='module')
def flat():
    with Session('autonomous_tractor', world='empty.sdf') as session:
        yield session


def test_reaches_and_holds_40_kmh(flat):
    run = flat.straight(speed=11.1, hold=8.0)
    assert run['v_min'] >= 11.0 and run['v_max'] <= 11.2


def test_brakes_at_3_m_s2(flat):
    run = flat.straight(speed=11.1, hold=3.0)
    ideal = run['brake_from'] ** 2 / (2 * 3.0)
    assert run['stop_distance'] == pytest.approx(ideal, rel=0.1)


@pytest.mark.parametrize('radius, speed', CURVES)
def test_curves_at_the_limit(flat, radius, speed):
    run = flat.circle(radius=radius, speed=speed)
    assert run['radius'] == pytest.approx(radius, rel=0.03)
    assert run['deviation'] < 0.2
    assert abs(run['max_roll']) < 5.0


def test_relief_of_the_farm():
    with Session('autonomous_tractor', world='citrus_farm.sdf') as session:
        run = session.straight(speed=11.1, hold=25.0)
    assert run['z_max'] - run['z_min'] > 5.0
    assert run['v_min'] >= 10.8 and run['v_max'] <= 11.4
    assert abs(run['max_roll']) < 20.0 and abs(run['max_pitch']) < 20.0
    assert run['stop_distance'] == pytest.approx(run['brake_from'] ** 2 / 6.0, rel=0.15)
