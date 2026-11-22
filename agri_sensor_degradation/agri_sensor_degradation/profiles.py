"""Perfis de degradação por nome, lidos de `config/profiles.yaml`."""
import os

import yaml


def _default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('agri_sensor_degradation'),
                        'config', 'profiles.yaml')


def _read(path):
    if path is None:
        path = _default_path()
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def names(sensor, path=None):
    """Lista ordenada dos perfis do sensor, ou lista vazia se não houver nenhum."""
    return sorted((_read(path).get(sensor) or {}).keys())


def load(sensor, name, path=None):
    """Devolve os valores do perfil; `KeyError` se o perfil não existir."""
    found = (_read(path).get(sensor) or {})
    if name not in found:
        raise KeyError(f'{sensor}/{name}')
    return dict(found[name])
