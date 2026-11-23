"""Teste de simulação da física a 40 km/h na pickup (F1.7, R1 e R3). Marca `simulation`.

A mesma `Session` do teste do trator. A pickup freia no limite do limitador dela, 6 m/s².
"""
import pytest

from agri_gz_bringup.maneuvers import Session

pytestmark = pytest.mark.simulation


@pytest.fixture(scope='module')
def flat():
    with Session('pickup', world='empty.sdf') as session:
        yield session


def test_reaches_and_holds_40_kmh(flat):
    run = flat.straight(speed=11.1, hold=8.0)
    assert run['v_min'] >= 11.0 and run['v_max'] <= 11.2
    assert run['stop_distance'] == pytest.approx(run['brake_from'] ** 2 / 12.0, rel=0.15)


@pytest.mark.parametrize('radius, speed', [(12.0, 3.46), (30.0, 11.1)])
def test_curves(flat, radius, speed):
    run = flat.circle(radius=radius, speed=speed)
    assert run['radius'] == pytest.approx(radius, rel=0.03)
    assert run['deviation'] < 0.2
