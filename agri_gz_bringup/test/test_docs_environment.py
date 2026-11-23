"""O README tem a referência do ambiente, com todo parâmetro de sensor e de degradação (F1.9).

A seção `## Referência do ambiente` do README da raiz tem as tabelas de mundos, veículos,
sensores e perfis de degradação, cada uma com a coluna `Fonte`. Todo parâmetro dos arquivos de
configuração aparece nela, entre crases:
- cada perfil de sensor do `sensors.gz.xacro` (`default`, `nominal`, `ideal`) e cada propriedade
  que eles definem (`gnss_rate`, `gnss_horizontal`...);
- cada perfil de degradação do `profiles.yaml` e cada chave dele (`markov.position`...);
- cada mundo do `agri_gz_worlds`.
"""
from pathlib import Path
import re

import yaml

REPO = Path(__file__).resolve().parents[2]


def section():
    text = (REPO / 'README.md').read_text()
    assert '## Referência do ambiente' in text
    return text.split('## Referência do ambiente')[1].split('\n## ')[0]


def test_tables_have_a_source_column():
    text = section()
    for title in ('Mundos', 'Veículos', 'Sensores', 'Perfis de degradação'):
        assert f'### {title}' in text, title
    headers = [line for line in text.splitlines() if line.startswith('|') and 'Fonte' in line]
    assert len(headers) >= 4


def test_sensor_profiles_and_properties():
    xacro = (REPO / 'agri_gz_bringup' / 'urdf' / 'sensors.gz.xacro').read_text()
    profiles = set(re.findall(r"profile == '(\w+)'", xacro))
    properties = set(re.findall(r'<xacro:property name="((?:gnss|imu)_\w+)"', xacro))
    assert profiles == {'default', 'nominal', 'ideal'} and properties
    text = section()
    for name in sorted(profiles | properties):
        assert f'`{name}`' in text, name


def test_degradation_profiles_and_keys():
    profiles = yaml.safe_load(
        (REPO / 'agri_sensor_degradation' / 'config' / 'profiles.yaml').read_text())
    text = section()
    for sensor, named in profiles.items():
        for name, values in named.items():
            assert f'`{name}`' in text, name
            for key in values:
                assert f'`{key}`' in text, f'{name}: {key}'


def test_worlds():
    text = section()
    for world in sorted((REPO / 'agri_gz_worlds' / 'worlds').glob('*.sdf')):
        assert f'`{world.name}`' in text, world.name


def test_no_invented_source():
    # A fonte é o plano, um arquivo do repositório ou a pendência de conferir com datasheet.
    text = section()
    assert 'http' not in text or 'datasheet' in text
