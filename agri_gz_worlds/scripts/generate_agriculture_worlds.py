#!/usr/bin/env python3
"""Gera os três mundos de fazenda de citros em worlds/.

    python3 scripts/generate_agriculture_worlds.py            # reescreve no lugar
    python3 scripts/generate_agriculture_worlds.py --out /tmp  # para testar uma mudança antes

Os mundos são versionados e o build nunca roda este script: o Gazebo lê o SDF, não o gerador.
Ele existe porque a parte interessante de um campo é um laço -- quatro mil árvores cortadas no
formato que as estradas deixam -- e porque os números que vale editar são os agronômicos no topo
de cada construtor (espaçamento de fileira, largura de carreador, raio de curva), e não as
poses em que eles se expandem.

Uma propriedade, três redes de estradas: citrus_farm_90, _60 e _30 compartilham o terreno, o
plantio e a mata, e diferem só no ângulo que cada curva faz.

Convenções comuns aos três mundos, que vale manter se você criar um quarto:

  +X corre ao longo das fileiras, +Y através delas, +Z para cima (o referencial usual do SDF).

  A origem do mundo fica num carreador, em terreno plano, em z = 0. É isso que permite ao
  `sim.launch.py` criar o rover em z = 0,15 em qualquer um destes mundos sem uma pose de
  criação por mundo.

  As marcações no chão -- a terra trabalhada de um talhão, a superfície de um carreador --
  são visuais sem colisão. Um degrau de 2 cm não é nada para um trator e é uma parede
  intransponível para um rover com rodízios de 4 cm, então as faixas são pintadas no chão e
  não levantadas sobre ele.
"""

from __future__ import annotations

import argparse
import re
import math
import random
from pathlib import Path as FilePath
from textwrap import indent

# Importado com outro nome porque um `Path` mais adiante neste arquivo é uma estrada, e a estrada
# leva a melhor: é sobre ela que está a maior parte do código. Deixado como `Path`, o argparse
# resolvia `type=Path` para a classe da estrada na hora da chamada e todo `--out` terminava em
# "invalid Path value".
HERE = FilePath(__file__).resolve().parent
DEFAULT_OUT = HERE.parent / "worlds"

# ---------------------------------------------------------------- paleta --

C_GRASS = (0.30, 0.40, 0.20)
C_BARK = (0.29, 0.23, 0.17)     # e o tronco dela, e o da mata
C_LATOSOL = (0.62, 0.31, 0.18)  # a terra vermelha de um pomar de citros
C_SHED = (0.70, 0.68, 0.64)


# --------------------------------------------------------------------- XML --

def _n(v: float) -> str:
    """Números como o SDF quer: sem expoentes, sem ruído no fim."""
    return f"{v:.4f}".rstrip("0").rstrip(".") or "0"


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> str:
    return "<pose>" + " ".join(_n(v) for v in (x, y, z, roll, pitch, yaw)) + "</pose>"


def material(rgb, specular=0.05) -> str:
    r, g, b = rgb
    return (
        "<material>\n"
        f"  <ambient>{_n(r * 0.7)} {_n(g * 0.7)} {_n(b * 0.7)} 1</ambient>\n"
        f"  <diffuse>{_n(r)} {_n(g)} {_n(b)} 1</diffuse>\n"
        f"  <specular>{_n(specular)} {_n(specular)} {_n(specular)} 1</specular>\n"
        "</material>"
    )


# Atrito explícito, pelo mesmo motivo de o empty.sdf defini-lo: o padrão é escorregadio o
# bastante para um acionamento diferencial girar as rodas no lugar. Na rampa e nos aclives é a
# diferença entre subir e deslizar.
FRICTION = (
    "<surface>\n"
    "  <friction>\n"
    "    <ode>\n"
    "      <mu>1.0</mu>\n"
    "      <mu2>1.0</mu2>\n"
    "    </ode>\n"
    "  </friction>\n"
    "</surface>"
)


def include(uri: str, name: str, x, y, z=0.0, roll=0.0, pitch=0.0, yaw=0.0) -> str:
    return (
        "<include>\n"
        f"  <uri>{uri}</uri>\n"
        f"  <name>{name}</name>\n"
        f"  {pose(x, y, z, roll, pitch, yaw)}\n"
        "</include>"
    )


def box(name, size, position, rgb, rpy=(0.0, 0.0, 0.0), collide=True) -> str:
    """Uma caixa estática. `collide=False` deixa uma marcação pura no chão."""
    sx, sy, sz = size
    geometry = f"<geometry><box><size>{_n(sx)} {_n(sy)} {_n(sz)}</size></box></geometry>"
    collision = ""
    if collide:
        collision = (
            '    <collision name="collision">\n'
            f"      {geometry}\n"
            f"{indent(FRICTION, '      ')}\n"
            "    </collision>\n"
        )
    return (
        f'<model name="{name}">\n'
        "  <static>true</static>\n"
        f"  {pose(*position, *rpy)}\n"
        '  <link name="link">\n'
        f"{collision}"
        '    <visual name="visual">\n'
        f"      {geometry}\n"
        f"{indent(material(rgb), '      ')}\n"
        "    </visual>\n"
        "  </link>\n"
        "</model>"
    )


def rotate(x, y, yaw):
    """Converte uma posição dada no referencial próprio de um bloco em coordenadas do mundo."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


WORLD = """\
<?xml version="1.0"?>
{comment}
<sdf version="1.10">
  <world name="{name}">

    <physics name="{step_name}" type="ignored">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- The four systems empty.sdf explains, plus the three sensor ones: a
         field is where a camera, a lidar and a GNSS earn their keep, and each
         of those systems is silent about its own absence — the sensor spawns,
         appears in the model and never publishes. They cost nothing until a
         model carrying one is spawned. -->
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system"
            name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"
            name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"
            name="gz::sim::systems::NavSat"/>

    <gravity>0 0 -9.8</gravity>

    <!-- Where on Earth the origin of this world is. NavSat converts the local
         ENU position into a fix against this datum, and without it a GNSS
         sensor has nothing to report. Bebedouro, SP: citrus country, and the
         same datum in every world here so a recorded track means the same
         thing wherever it was driven. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>-20.9494</latitude_deg>
      <longitude_deg>-48.4794</longitude_deg>
      <elevation>570</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.66 0.75 0.86 1</background>
      <sky/>
      <shadows>true</shadows>
    </scene>

    <!-- Mid-morning sun, from the east and high: shadows long enough to show
         down the rows, short enough not to swallow the lanes. -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.9 0.88 0.82 1</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>0.4 0.25 -0.88</direction>
    </light>

{body}

  </world>
</sdf>
"""


def emit(name: str, comment: str, parts: list[str], step: float = 0.001) -> str:
    """Monta um mundo. `step` é o passo da física, 1 ms a menos que o mundo seja grande o
    bastante para pedir outro: o custo de um passo aqui é quase todo sobrecarga fixa por
    entidade, então um passo de 2 ms devolve metade do fator de tempo real, a um custo em
    precisão de contato que nenhum robô terrestre lento dirigindo entre obstáculos estáticos
    vai notar."""
    # Um traço duplo é ilegal dentro de um comentário XML. O parser do Gazebo deixa passar,
    # o ElementTree e o xmllint não, então um cabeçalho desenhado com réguas ASCII feitas de
    # traços produz um mundo que só o simulador consegue ler. Desenhe as plantas com '=' e ':'.
    comment = comment.strip()
    inner = comment.removeprefix("<!--").removesuffix("-->")
    if "--" in inner:
        raise ValueError(f"{name}: '--' inside the header comment")
    body = "\n\n".join(indent(p, "    ") for p in parts)
    return WORLD.format(name=name, comment=comment, body=body, step=_n(step),
                        step_name=f"{_n(step * 1000)}ms")


# --------------------------------------------------------------- cenário --

# As mesmas plantas que a biblioteca de modelos guarda, escritas como formas soltas para que
# centenas delas compartilhem um único elo.
#
# Medido, porque não é óbvio: 300 objetos estáticos como 300 modelos custam 0,8 ms por passo;
# os mesmos 300 como elos dentro de um modelo, 0,5 ms; como formas simples dentro de um único
# elo, 0,3 ms. O Gazebo cobra por entidades, e um modelo é várias. Nas seiscentas árvores,
# montes e pedras espalhados por cada fazenda, essa é a diferença entre um mundo que acompanha
# o relógio e um que roda a um terço dele; então aqui o `include` sai e a geometria é
# embutida. O que está escrito abaixo é, portanto, a única definição de uma árvore da mata e
# de uma pedra. Onde a quantidade é pequena o `model://` fica, e models/ guarda exatamente
# esses: o tanque, as caixas, o trator e a carreta, cinco de cada tipo em vez de seiscentos.
#
# (tipo: [(colide, geometria, (dx, dy, dz), rgb)])
SCENERY = {
    # `model://forest_tree`, embutido. Dois elipsoides, um dentro do outro, sobre um tronco: o
    # externo é a copa e o interno, mais claro, é a copa densa em volta do tronco, o que impede
    # a madeira em pé de parecer um citros grande demais. Mesmos números de models/forest_tree,
    # e é nesse arquivo que eles são explicados.
    "forest_tree": [
        (True, "<cylinder><radius>0.26</radius><length>2.9</length></cylinder>",
         (0, 0, 1.45), C_BARK),
        (False, "<ellipsoid><radii>2.58 2.50 2.06</radii></ellipsoid>",
         (0, 0, 4.65), (0.10, 0.24, 0.09)),
        (False, "<ellipsoid><radii>1.46 1.38 1.29</radii></ellipsoid>",
         (0, 0, 4.90), (0.19, 0.38, 0.13)),
    ],
    "boulder": [
        (True, "<sphere><radius>0.42</radius></sphere>", (0, 0, 0.28), None),
        (False, "<ellipsoid><radii>0.55 0.4 0.42</radii></ellipsoid>",
         (0, 0, 0.28), (0.45, 0.44, 0.41)),
    ],
}


def scenery(name, items):
    """Um modelo, um elo, com toda planta e pedra espalhada dentro dele.

    items: (tipo, x, y, z, yaw). O yaw gira as peças fora do centro, o que impede cem cópias
    da mesma árvore de parecerem cem cópias da mesma árvore.
    """
    out = [f'<model name="{name}">', "  <static>true</static>",
           '  <link name="link">']
    for n, (kind, x, y, z, yaw) in enumerate(items):
        for k, (collides, geom, (dx, dy, dz), rgb) in enumerate(SCENERY[kind]):
            ox, oy = rotate(dx, dy, yaw)
            at = pose(x + ox, y + oy, z + dz, 0, 0, yaw)
            # Colisão e visual são perguntas separadas, e lê-las como uma só é como cem árvores
            # viraram copas flutuando sobre o nada: um tronco colide e também precisa ser visto,
            # e o cilindro de colisão de um monte é um substituto de visuais que ele já tem.
            if collides:
                out += [f'    <collision name="c{n}_{k}">', f"      {at}",
                        f"      <geometry>{geom}</geometry>", "    </collision>"]
            if rgb is not None:
                out += [f'    <visual name="v{n}_{k}">', f"      {at}",
                        f"      <geometry>{geom}</geometry>",
                        f"{indent(material(rgb), '      ')}", "    </visual>"]
    out += ["  </link>", "</model>"]
    return "\n".join(out)


# ----------------------------------------------------------------- terreno --

class Terrain:
    """Terreno ondulado, feito de lajes inclinadas.

    O DART do Gazebo Harmonic não colide com um `heightmap` nem com uma malha neste build. Os
    dois foram tentados e medidos: uma esfera solta sobre um `heightmap` cai até z = -1686, e
    sobre uma `trimesh` até z = -1093, no segundo caso depois de a malha carregar sem reclamar.
    Então o terreno aqui é feito da única coisa que certamente colide, que são caixas, e o
    vinhedo prova: um rover deixado numa laje inclinada ali se assenta exatamente nos seus 8
    graus e fica parado.

    A face de cima de uma caixa é um plano, então a superfície tem de ser plana por partes, e
    para as lajes se encaixarem sem deixar um degrau nas junções cada uma só pode inclinar em
    torno de um único eixo. Inclinando em torno de dois, as faces de cima se projetam em
    paralelogramos que deixam de se encontrar nos cantos: nestes tamanhos são dez a vinte
    centímetros de vão, que não é nada para os olhos e é tudo para um rover com rodízios de
    4 cm.

    Daí o relevo que varia ao longo de X e é plano ao longo de Y. Cristas e vales correndo de
    norte a sul, o que é uma forma de relevo real e não um compromisso, e lajes que se
    encontram exatamente em cada dobra porque os dois planos são definidos para passar pela
    mesma altura de nó ali.
    """

    def __init__(self, nodes, y_half=250.0, thickness=8.0):
        self.nodes = sorted(nodes)                  # [(x, z)] ao longo do perfil
        self.y_half = y_half
        self.thickness = thickness

    def _span(self, x):
        for i in range(len(self.nodes) - 1):
            if self.nodes[i][0] <= x <= self.nodes[i + 1][0]:
                return i
        return 0 if x < self.nodes[0][0] else len(self.nodes) - 2

    def strip(self, x):
        """Em que trecho plano um ponto está. As fileiras não podem cruzar um destes, para que
        toda sebe fique num único plano."""
        return self._span(x)

    def grade(self, x):
        i = self._span(x)
        (x0, z0), (x1, z1) = self.nodes[i], self.nodes[i + 1]
        return (z1 - z0) / (x1 - x0)

    def z(self, x, y=0.0):
        i = self._span(x)
        (x0, z0), (x1, z1) = self.nodes[i], self.nodes[i + 1]
        return z0 + (z1 - z0) * (x - x0) / (x1 - x0)

    def pose_on(self, x, y, yaw=0.0):
        """Onde e como uma coisa plana se apoia no terreno aqui.

        A rotação desejada é Ry(pitch) . Rz(yaw): gira o objeto para o rumo dele e depois
        deita a coisa toda na encosta. O SDF quer isso como roll, pitch e yaw na outra ordem,
        então ela é composta e lida de volta.
        """
        p = -math.atan(self.grade(x))
        c, s = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(p), math.sin(p)
        yaw_o = math.atan2(s, cp * c)
        pitch_o = math.asin(max(-1.0, min(1.0, sp * c)))
        roll_o = math.atan2(sp * s, cp)
        return x, y, self.z(x), roll_o, pitch_o, yaw_o

    def slabs(self, name, rgb):
        """O terreno em si: um modelo, uma laje por trecho do perfil."""
        out = [f'<model name="{name}">', "  <static>true</static>",
               '  <link name="link">']
        for i in range(len(self.nodes) - 1):
            (x0, z0), (x1, z1) = self.nodes[i], self.nodes[i + 1]
            grade = (z1 - z0) / (x1 - x0)
            theta = math.atan(grade)
            length = (x1 - x0) / math.cos(theta)
            t = self.thickness
            # a face de cima tem de passar pelos dois nós, então o centro da caixa fica meia
            # espessura para trás ao longo da normal da própria laje
            cx = (x0 + x1) / 2 + (t / 2) * math.sin(theta)
            cz = (z0 + z1) / 2 - (t / 2) * math.cos(theta)
            geom = (f"<geometry><box><size>{_n(length)} {_n(2 * self.y_half)} "
                    f"{_n(t)}</size></box></geometry>")
            out += [f'    <collision name="collision_{i}">',
                    f"      {pose(cx, 0, cz, 0, -theta, 0)}",
                    f"      {geom}",
                    f"{indent(FRICTION, '      ')}", "    </collision>",
                    f'    <visual name="visual_{i}">',
                    f"      {pose(cx, 0, cz, 0, -theta, 0)}",
                    f"      {geom}",
                    f"{indent(material(rgb), '      ')}", "    </visual>"]
        out += ["  </link>", "</model>"]
        return "\n".join(out)


# --------------------------------------------------------- estradas e curvas -

class Path:
    """Uma linha central de estrada, escrita do jeito que é percorrida.

        Path(0, 0, 90).straight(50).turn(-45, 22).straight(30)

    `turn()` recebe o ângulo em graus, positivo para a esquerda, e o raio da linha central. O
    raio é o número que importa. Um trator com carreta carregada em barra de tração rígida
    corta a curva, então a borda de dentro de uma curva é o que as rodas dele encontram
    primeiro, e uma curva que parece generosa no mapa é onde o motorista precisa manobrar.
    """

    def __init__(self, x, y, heading_deg):
        self.x0, self.y0, self.h0 = x, y, math.radians(heading_deg)
        self.segments: list[tuple[str, float, float]] = []

    def straight(self, length):
        self.segments.append(("s", length, 0.0))
        return self

    def turn(self, degrees, radius):
        """Curva a estrada. `radius=0` é uma esquina viva: sem arco, só um vértice.

        Uma estrada de fazenda trabalhada vira para os dois lados. Onde há espaço ela é
        abaulada em curva, e onde dois blocos se encontram em esquadro ela é simplesmente
        cortada, e as fotografias aéreas estão cheias dos dois. Um vértice também não custa
        reta -- um arco come raio * tan(metade da curva) da estrada em cada ponta, uma esquina
        não come nada -- e é por isso que um 90 vivo cabe onde um arredondado não caberia.
        """
        self.segments.append(("t", radius, math.radians(degrees)))
        return self

    def _arc_centre(self, x, y, h, radius, sweep):
        sign = 1.0 if sweep > 0 else -1.0
        return (x - sign * radius * math.sin(h), y + sign * radius * math.cos(h), sign)

    def points(self, step=1.0):
        """A linha central como uma poligonal densa, para medir distâncias até ela."""
        x, y, h = self.x0, self.y0, self.h0
        pts = [(x, y)]
        for kind, a, b in self.segments:
            if kind == "s":
                n = max(1, int(a / step))
                for i in range(1, n + 1):
                    pts.append((x + a * i / n * math.cos(h), y + a * i / n * math.sin(h)))
            else:
                radius, sweep = a, b
                if radius <= 0.0:
                    h += sweep                          # uma esquina é um ponto só, já em `pts`
                    continue
                cx, cy, sign = self._arc_centre(x, y, h, radius, sweep)
                n = max(2, int(abs(sweep) * radius / step))
                for i in range(1, n + 1):
                    ang = h + sweep * i / n
                    pts.append((cx + sign * radius * math.sin(ang),
                                cy - sign * radius * math.cos(ang)))
                h += sweep
            x, y = pts[-1]
        return pts

    def at(self, distance):
        """Posição e rumo a certa distância ao longo da estrada."""
        pts = self.points(0.25)
        i = min(int(distance / 0.25), len(pts) - 2)
        (px, py), (qx, qy) = pts[i], pts[i + 1]
        return px, py, math.atan2(qy - py, qx - px)

    def turns(self):
        """(graus, raio) de cada curva, para as verificações olharem."""
        return [(math.degrees(b), a) for kind, a, b in self.segments if kind == "t"]

    def end(self):
        """Onde a estrada chegou, e para que lado ela aponta."""
        x, y, h = self.x0, self.y0, self.h0
        for kind, a, b in self.segments:
            if kind == "s":
                x, y = x + a * math.cos(h), y + a * math.sin(h)
                continue
            cx, cy, sign = self._arc_centre(x, y, h, a, b)
            x = cx + sign * a * math.sin(h + b)
            y = cy - sign * a * math.cos(h + b)
            h += b
        return x, y, h

    def close_to_y(self, y_target):
        """Segue em frente até a estrada alcançar uma dada linha, e para ali.

        Toda estrada de bloco em citrus_farm sai do carreador principal e volta a ele. Calcular
        à mão o comprimento dessa última reta significa refazer a conta toda vez que um raio
        muda; isto apenas pergunta.
        """
        x, y, h = self.end()
        d = (y_target - y) / math.sin(h)
        if d <= 0:
            raise ValueError(f"road points away from y={y_target}")
        return self.straight(d)

    def close_to_boundary(self, points, reach_max=700.0):
        """Segue em frente até a estrada encontrar o limite da propriedade, e para ali.

        A contrapartida de `close_to_y` para um limite que não é uma reta: um hexágono ou um
        anel de doze lados. Mesmo propósito, que é nenhuma estrada terminar no meio de um
        campo.
        """
        x, y, h = self.end()
        grid = NearGrid(points)
        d = 5.0
        while d < reach_max:
            px, py = x + d * math.cos(h), y + d * math.sin(h)
            if grid.within(px, py, 3.0):
                return self.straight(d)
            d += 0.5
        raise ValueError("this road never reaches the boundary")

    def marks(self):
        """(x, y, rumo, graus, raio, trecho) na boca de cada curva.

        `run` é a reta que leva até ela, o que decide onde o ângulo pode ser escrito e quão
        grandes as figuras podem ser. Numa estrada de seis 30 há vinte metros entre curvas e
        numa de dois 90 há cento e trinta; um número posto a uma distância fixa antes da curva
        cai na curva anterior no primeiro caso.
        """
        x, y, h = self.x0, self.y0, self.h0
        out, run = [], 0.0
        for kind, a, b in self.segments:
            if kind == "s":
                x, y = x + a * math.cos(h), y + a * math.sin(h)
                run = a
                continue
            out.append((x, y, h, math.degrees(b), a, run))
            run = 0.0
            cx, cy, sign = self._arc_centre(x, y, h, a, b)
            x = cx + sign * a * math.sin(h + b)
            y = cy - sign * a * math.cos(h + b)
            h += b
        return out


class NearGrid:
    """Pontos em baldes (`buckets`), para perguntar 'há algo a até r daqui' dez mil vezes. Uma
    varredura linear por todos os pontos da estrada transforma o gerador num minuto de espera;
    uma grade de 10 m o mantém abaixo de um segundo."""

    def __init__(self, points, cell=10.0):
        self.cell = cell
        self.buckets: dict[tuple[int, int], list[tuple[float, float]]] = {}
        for px, py in points:
            self.buckets.setdefault((int(px // cell), int(py // cell)), []).append((px, py))

    def within(self, x, y, r):
        c = self.cell
        rr = r * r
        for i in range(int((x - r) // c), int((x + r) // c) + 1):
            for j in range(int((y - r) // c), int((y + r) // c) + 1):
                for px, py in self.buckets.get((i, j), ()):
                    if (px - x) ** 2 + (py - y) ** 2 <= rr:
                        return True
        return False


def in_polygon(x, y, poly):
    """Lançamento de raio, para a mata poder ter a forma que quiser."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            if x < x1 + (y - y1) / (y2 - y1) * (x2 - x1):
                inside = not inside
    return inside


def blob(cx, cy, rx, ry, seed, n=16, wobble=0.3):
    """Um polígono fechado irregular: o aspecto de uma mancha de mata num mapa."""
    rng = random.Random(seed)
    return [(cx + rx * (1 + rng.uniform(-wobble, wobble)) * math.cos(2 * math.pi * i / n),
             cy + ry * (1 + rng.uniform(-wobble, wobble)) * math.sin(2 * math.pi * i / n))
            for i in range(n)]


# ------------------------------------------------------------ fazendas de citros ---

FARM_COMMENT = """
<!--
{opening}

  A field on a grid is one thing. This is the property, and the difference
  is the roads. A carreador does not run straight because a farm is
  not a rectangle: it bends around the mata, around the low ground, around
  whatever was there first, and every row of every talhao is cut to whatever
  shape that leaves. Rows here are not laid out and then drawn; they are
  sampled along their line and kept only where the ground is really plantable,
  which is why they end square against a curve and why the corners of a block
  carry short rows of three or four trees, exactly as they do in the
  photographs this was built from (agri_gz_worlds/docs/citrus/).

  Two straight carreadores cross at the origin and cut the property into four
  blocks. Each block has one loop leaving the east-west road and coming back to
  it. {loops_note} The closing straight of every loop is computed, not typed,
  so changing a radius still lands the road back on the carreador.

  Nothing on the property dead ends. The two dividing roads run edge to edge
  and finish on a perimeter carreador that closes on itself; every block loop
  leaves and rejoins the east-west road. Seven roads, one connected network, no
  place a machine has to reverse out of.

  And no two of them run alongside each other. Roads are meant to cross, but a
  block loop passing 5 m outside the perimeter, which is what the first cut of
  these worlds did, is not two roads: it is one wide mess with a strip of hedge
  stranded down the middle. Every loop is sized so that nothing parallel comes
  within 32 m of it, which leaves room for three or four rows in between.

  The angle of every bend is painted in the middle of the bend itself: dark
  figures on a pale board, digits side by side, and every number on the
  property laid out north up whatever direction its road runs. On the arc and
  not set back along the straight, because a number on the ground is a label
  and not a warning: set back, it has to be matched to a bend by eye, and on a
  property of mixed angles the number before a bend and the number after it
  are both in view from the same place with nothing to say which is which.
  Consistent orientation is the other half. Numbers written along the direction of travel
  the way a carriageway writes SLOW are meant to be driven over one figure at a
  time, and read as nonsense from above, where a whole block is actually looked
  at. Seven boxes to a digit, because SDF has no text geometry and nothing here
  carries a texture.

  The perimeter's four corners are marked too, at 90 degrees, on the principle
  that a bend either gets a number or the numbers mean nothing.

  Sixteen metres of relief across the property, and how the ground is built is
  the paragraph below on slabs: it matters, because the obvious ways of making
  ground in Gazebo do not collide.

  Every carreador is the same 7 m of compacted running surface inside 11 m of
  bare earth, hedge to hedge. One width for the whole property, because the
  widest thing that uses a farm road is the same everywhere. No bend is tighter
  than 22 m of centreline radius, which leaves 18.5 m on the inside for a
  tractor and a loaded bin_trailer: 9 m of vehicle on a rigid drawbar, which
  cuts the corner rather than following the centreline.

  The tractor and its trailer are parked in the yard beside the shed, which is
  where a driver leaves them. An alley is a tighter thing entirely: 6.5 m of
  carreira less 3.95 m of canopy leaves 2.55 m of clear ground, which a 2.15 m
  tractor fits into with 0.2 m to spare on each side. It did not fit at all
  until the crowns were cut back by a seventh: at 4.6 m of canopy the alley was
  1.9 m, and the arithmetic everybody quoted assumed a 3.5 m canopy nobody had
  measured. Narrow is the point — it is why orchard tractors are built that way
  — and it also means a machine standing in an alley blocks it outright, so
  nothing is left parked anywhere but the yard and the road.

{map}

  The origin is the crossroads at the middle of the property.

  The ground is not flat. A profile runs east to west through nodes the land
  passes through: a steep face falling off the western boundary at 8.9%, a
  valley floor around x = -90, then a long climb at 5.3% to a ridge at x = 160
  and away off the eastern edge. Eight metres of relief across the property,
  and the yard around the origin levelled, as a yard is.

  Sixteen metres of relief across the property, and it is built from tilted
  slabs because in this build of Gazebo nothing else collides. A sphere dropped
  on a heightmap falls to z = -1686, and through a trimesh to z = -1093, the
  second time after the mesh has loaded without a word of complaint. Boxes collide, so boxes it is, and a box top is a plane:
  the surface has to be piecewise planar, and each slab may tilt about one axis
  only or the tops stop meeting at the corners. Hence relief along X and level
  along Y, ridges running north to south. The eleven slabs meet within 1.3 mm
  of one another, which matters because 4 cm casters do not climb steps.

  A hedge is broken where the ground folds as well as where a road crosses it,
  so every row lies in one plane; rows ending on the crest of a rise is what a
  field on rolling ground looks like anyway. Everything that rests on the
  ground rather than growing out of it takes the local slope with it, which is
  why a parked tractor sits nose down on a grade.

  Roads and the numbers painted on them are cut to follow the ground in both
  directions, and the second one is the one that is easy to miss. The folds
  here run north to south, so a carreador that runs north to south has a fold
  down the middle of it rather than across it: drawn as one flat plank, one
  edge ends up buried and the other in the air. On the 11 m of bare earth at
  x = -90, where the grade goes from -7% to +3%, that is 55 cm at the edges,
  and from above it reads as a road cut in half lengthwise. So a road piece is
  cut along its length where a fold crosses its centreline, across its width
  where a fold runs beneath it, and finely near the fold when it meets one at
  an angle, where the fold cuts the rectangle diagonally and neither cut alone
  is enough. Measured across the worst of them afterwards: 6 cm, against 55.

  What is not road or mata is planted. The talhoes run right up to the
  carreadores on all four blocks, which is what a property of this size looks
  like when it is worked: the ground between the rows and the road is a
  headland, not a paddock. The only things loose in it are standing timber the
  farm never cleared and the odd rock: one of them tall enough to plan a route
  around, the other the awkward size for a small robot.

  Four things here are sized for the fact that this world is four times the
  field the others are, and all four are worth knowing before copying them.

  The physics step is 2 ms rather than the 1 ms every other world uses. A step
  in a world like this costs almost nothing for the physics, since everything
  in it is static, and almost everything for the number of entities it walks,
  so halving the number of steps very nearly doubles the real-time factor.
  Nothing here needs 1 ms: the obstacles do not move and the robot is slow.

  The hedgerows are consolidated, and the consolidation goes further than the
  link. 233 rows as 233 models cost real time every step; as collision boxes
  and crowns inside one link per block they cost a fraction of it. Measured on
  300 identical objects: 300 models, 0.8 ms a step; the same as links in one
  model, 0.5; as plain shapes in a single link, 0.3.

  What that last number says, and what was missed for a long time, is that
  shapes in a single link are cheaper but not free. The server walks every
  visual on every step whether or not it moves and whether or not anything is
  rendering, so a property with a visual per tree carries 4966 of them and
  pays for all of it. Measured on this world with the visuals stripped out
  entirely: 1428 physics steps a second against 312 with them in.

  So the trees are baked into runs. Eight to a mesh, then four, then two, from
  a library in models/citrus_tree/meshes/, and at most one tree of any row is
  planted on its own. Whole rows would be better still and are impossible: an
  OBJ of one crown is 87 KB, so unique rows would put 430 MB of geometry in
  this world alone. Six runs of each length is 10 MB for every world here, and
  48 distinct trees is enough that a row of forty does not repeat visibly.

  The mata and the loose ground furniture are inlined for the same reason, into
  one model per quadrant. Everywhere the count is small, model:// stays: it is
  the better way to write a world, and the library remains the definition of
  what these things are.

  A tree is one visual: a crown from model://citrus_tree with its own trunk
  inside the same mesh. These worlds used to draw a bare ellipsoid at 1.75 m
  with nothing under it, on the argument that a trunk would double the visual
  count at 2275 trees. That argument was about a trunk as a second *shape*. A
  trunk that is part of the crown's mesh is free, because Gazebo charges per
  entity and not per triangle, and the tree now stands on something.

  Measured on the machine this was built on, with the ground and all of it:
  a little over 1.1 of wall clock headless. Written the obvious way, one model
  per row and per tree, the same world runs at about a third of that. If more
  is wanted back, a talhao is the cheapest metre to buy: drop one from
  THROUGH_BLOCKS and regenerate.

  Four tree models, and every one of them is in this world. Most talhoes are
  planted with model://citrus_tree, the polygon crown, baked eight to a run.
  Talhao e in the north west and talhao l south of the yard are planted with
  model://citrus_round, the same envelope as one ellipsoid: they are the
  control, because anything a perception stack does differently between a
  round block and a mesh one came from the foliage and not from the shape of
  the field. Talhao n, the block the crossroads looks into, carries
  model://citrus_tree_fruit: 131 trees with oranges on them, eleven metres
  south of the origin, and the only thing on the property a stack can be asked
  to find that is not a row, a road or a trunk. The mata is
  model://forest_tree, two ellipsoids one inside the other on a visible trunk,
  which is what keeps standing timber from reading as an oversized citrus.
  They are on the map above as : o * and #.
-->
"""


def rot_apply(roll, pitch, yaw, v):
    """Converte um deslocamento dado no referencial próprio de um objeto em deslocamento do
    mundo."""
    ca, sa = math.cos(yaw), math.sin(yaw)
    cb, sb = math.cos(pitch), math.sin(pitch)
    cg, sg = math.cos(roll), math.sin(roll)
    x, y, z = v
    return (
        (ca * cb) * x + (ca * sb * sg - sa * cg) * y + (ca * sb * cg + sa * sg) * z,
        (sa * cb) * x + (sa * sb * sg + ca * cg) * y + (sa * sb * cg - ca * sg) * z,
        (-sb) * x + (cb * sg) * y + (cb * cg) * z,
    )


CITRUS_VARIANTS = "abcdef"

# Árvores assadas de oito, quatro e duas por malha por scripts/generate_citrus_meshes.py, e o
# espaçamento em que foram assadas. Uma fileira é posta em sequências, as mais longas
# primeiro, e a última árvore ou as duas últimas são plantadas avulsas.
#
# Isto é o orçamento de entidades, não um truque de renderização. Cada `<visual>` é uma
# entidade e o servidor percorre todas a cada passo, movam-se ou não, sejam renderizadas ou
# não: citrus_farm_60 com 7255 visuais dá 312 passos por segundo, e 1428 sem os visuais. As
# 4966 árvores viram cerca de 620 visuais assim.
CITRUS_RUNS = "abcdef"
RUN_SIZES = (8, 4, 2)
RUN_SPACING = 3.0


# Com qual copa cada talhão é plantado. Três dos quatro modelos de árvore são de citros e são
# nomeados aqui; o quarto, `model://forest_tree`, é do que a mata é feita e nunca é plantado
# em fileiras.
#
#   "mesh"         a copa poligonal irregular, assada oito por sequência
#   "fruit"        a mesma copa, carregando laranjas
#   "round"        a copa lisa de `model://citrus_round`
#   "round_fruit"  a copa lisa, carregando laranjas
#
# Um bloco não nomeado aqui é plantado como "mesh", que é o único dos três que pode usar as
# sequências assadas e, portanto, o único quase de graça. Os outros dois custam um visual por
# árvore, e "fruit" custa dois, então o que esta tabela realmente define é quanto do orçamento
# de entidades vai para olhar as coisas e não para passar por elas. Três blocos de quatorze
# é a divisão em torno da qual estes mundos foram ajustados.
#
# Eles são espalhados e não adjacentes de propósito: um bloco redondo no noroeste, um ao sul
# do pátio, e o frutífero no cruzamento, para que nenhuma rota pela propriedade os perca e
# nenhum possa ser confundido com uma mudança no terreno.
PLANTING = {"e": "round", "l": "round", "n": "fruit",
            "nw": "round", "sw": "round_fruit", "se": "fruit"}


def hedgerow_shapes(tag, x0, y0, x1, y1, seed, spacing=3.0, canopy=3.5,
                    terrain=None, kind="mesh"):
    """Um trecho de sebe fechada de citros, como formas já em coordenadas do mundo.

    Cortado no comprimento em vez de fixo num padrão de 24 m, porque uma fileira que para onde
    um carreador curva não pode ser uma peça padrão. É por isso que a sebe é construída aqui e
    não guardada em models/ como uma seção pronta.

    Toda árvore é um visual, qualquer que seja a copa que ela use: uma malha de
    `model://citrus_tree` com o próprio tronco dentro, ou o único elipsoide de
    `model://citrus_round`. Só o fruto custa um segundo. O que estes mundos desenhavam antes era
    uma copa nua flutuando sem nada embaixo -- um elipsoide a 1,75 m e nenhum tronco --
    defendido pelo custo, e a defesa estava errada: um tronco como *segunda* forma de fato
    dobraria a contagem de visuais, mas um tronco dentro da malha da própria copa não custa
    nada, porque o Gazebo cobra por entidade e não por triângulo. A copa redonda tem saia até o
    chão pelo mesmo motivo: sai de graça apoiá-la em alguma coisa.

    Devolve formas soltas e não um modelo para que um bloco inteiro delas compartilhe um único
    elo. Duzentas sebes como duzentos modelos são uma parte mensurável de um passo de
    simulação, na mesma contabilidade.

    As árvores são plantadas a partir de sequências assadas por
    scripts/generate_citrus_meshes.py, de oito em oito e depois de quatro e de duas, com no
    máximo uma árvore de cada fileira sobrando para ser plantada avulsa. É isso que mantém a
    contagem de entidades baixa; ver `CITRUS_RUNS` acima.

    `kind` escolhe com qual dos três modelos de citros a fileira é plantada, e a escolha é
    tanto um orçamento de entidades quanto um visual:

      "mesh"         a copa poligonal irregular de `model://citrus_tree`.
      "fruit"        a mesma copa carregando laranjas.
      "round"        a forma de `model://citrus_round`: uma copa lisa, e o toco em que ela
                     se apoia.
      "round_fruit"  a copa lisa carregando laranjas.

    Duas copas e com fruto ou sem fruto, que são quatro bibliotecas e quatro jeitos de um
    talhão ser. Um por bloco e não mais: um bloco é plantado com uma idade e uma variedade, e
    um bloco de dois tipos de árvore são dois blocos.

    Os três são assados com oito, quatro e duas copas por malha, então os três custam o mesmo,
    cerca de um visual a cada oito árvores. Um talhão leva um deles e não mais: um bloco é
    plantado com uma idade e uma variedade, e um bloco de dois tipos de árvore são dois blocos.

    Qual talhão recebe qual está em `PLANTING`, perto do topo deste arquivo.
    """
    length = math.hypot(x1 - x0, y1 - y0)
    n = int(length // spacing)
    if n < 3:
        return []
    span = (n - 1) * spacing
    yaw = math.atan2(y1 - y0, x1 - x0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    rng = random.Random(seed)
    # Uma fileira nunca cruza uma dobra do terreno, então um plano descreve a fileira toda e a
    # sequência inteira pode simplesmente ser deitada nesse plano.
    px, py, pz, roll, pitch, yaw_w = (terrain.pose_on(cx, cy, yaw) if terrain
                                      else (cx, cy, 0.0, 0.0, 0.0, yaw))

    def at(local, spin=0.0):
        ox, oy, oz = rot_apply(roll, pitch, yaw_w, local)
        # O giro próprio de uma árvore vai no yaw do plano da fileira. Em terreno tão plano isso
        # é o mesmo que girá-la em torno do próprio tronco, com erro de um grau, e não custa
        # uma segunda rotação.
        return pose(px + ox, py + oy, pz + oz, roll, pitch, yaw_w + spin)

    out = [f'    <collision name="c_{tag}">',
           f"      {at((0, 0, 1.5))}",
           f"      <geometry><box><size>{_n(span + canopy - 0.3)} 3.2 3.0</size>"
           "</box></geometry>", "    </collision>",
           f'    <visual name="strip_{tag}">',
           f"      {at((0, 0, 0.01))}",
           f"      <geometry><box><size>{_n(span + canopy + 0.4)} 3.6 0.02</size>"
           "</box></geometry>",
           f"{indent(material(C_LATOSOL), '      ')}", "    </visual>"]

    # A fileira é posta em sequências assadas, as mais longas primeiro, e o que não preencher
    # uma sequência de duas é plantado árvore por árvore. Uma sequência só cabe se a fileira
    # está no espaçamento em que as sequências foram assadas.
    #
    # De qual biblioteca vêm as sequências é tudo o que `kind` faz. Os três são assados com
    # oito, quatro e duas copas por malha, então um talhão de frutos e um talhão de copas
    # simples custam o que custa um talhão das comuns. Antes de serem assados o fruto ia numa
    # árvore por vez, com um segundo visual para as laranjas, e dois blocos de nove hectares
    # chegavam a 6180 visuais contra os 922 que a copa comum custava para as mesmas árvores.
    run_name = "citrus_plain" if kind.startswith("round") else "citrus_run"
    run_tail = "_fruit" if kind.endswith("fruit") else ""
    baked = 0
    if abs(spacing - RUN_SPACING) < 1e-6:
        first = 0
        for size in RUN_SIZES:
            while n - first >= size:
                centre = -span / 2 + (first + (size - 1) / 2) * spacing
                v = CITRUS_RUNS[rng.randrange(len(CITRUS_RUNS))]
                out += [f'    <visual name="run{size}_{tag}_{first}">',
                        f"      {at((centre, 0.0, 0.0))}",
                        "      <geometry><mesh><uri>"
                        f"model://citrus_tree/meshes/{run_name}{size}_{v}"
                        f"{run_tail}.obj"
                        "</uri></mesh></geometry>", "    </visual>"]
                first += size
        baked = first

    for i in range(baked, n):
        along = -span / 2 + i * spacing
        # Seis copas, embaralhadas, giradas a qualquer ângulo e escaladas alguns por cento cada
        # uma. A variação tem de existir ou uma fileira parece uma árvore carimbada quarenta
        # vezes; dentro de uma sequência a mesma variação já vem assada.
        v = CITRUS_VARIANTS[rng.randrange(len(CITRUS_VARIANTS))]
        spin = rng.uniform(0, 2 * math.pi)
        sx = 1 + rng.uniform(-0.08, 0.08)
        sy = 1 + rng.uniform(-0.08, 0.08)
        sz = 1 + rng.uniform(-0.10, 0.13)
        scale = f"<scale>{_n(sx)} {_n(sy)} {_n(sz)}</scale>"
        # Fora da linha da fileira por alguns centímetros, do jeito que uma árvore plantada fica:
        # fileiras perfeitamente retas são a outra metade do motivo de um bloco gerado parecer
        # gerado.
        across = rng.uniform(-0.14, 0.14)

        # O que sobra no fim de uma fileira, plantado um por vez.
        stem = "citrus_plain_" if kind.startswith("round") else "citrus_"
        singles = ([(stem, ""), (stem, "_fruit")] if kind.endswith("fruit")
                   else [(stem, "")])
        for stem, suffix in singles:
            out += [f'    <visual name="t_{tag}_{i}{suffix}">',
                    f"      {at((along, across, 0.0), spin)}",
                    "      <geometry><mesh><uri>"
                    f"model://citrus_tree/meshes/{stem}{v}{suffix}.obj</uri>"
                    f"{scale}</mesh></geometry>", "    </visual>"]
    return out


def hedges(name, shapes):
    """Os trechos de sebe de um bloco, em um modelo e um elo."""
    return "\n".join([f'<model name="{name}">', "  <static>true</static>",
                      '  <link name="link">'] + shapes + ["  </link>", "</model>"])


# ------------------------------------------------- estradas, como uma fita --
#
# Um carreador era desenhado com caixas: uma laje por reta, um leque delas por arco, e depois
# mais delas, porque o terreno dobra por baixo e uma caixa é um plano só. Onde uma estrada
# cruzava uma dobra em ângulo, a dobra corria na diagonal pelo retângulo, o que nenhum corte
# único resolve, então o código picava o comprimento a cada 3 m perto do vinco e dividia cada
# pedaço na largura. Funcionava e era caro: as quatro estradas oblíquas de citrus_farm chegavam
# a 1139 visuais entre elas, contra 190 para todo o perímetro de 2240 m, e custavam ao mundo
# 0,16 do fator de tempo real.
#
# Agora uma estrada é uma malha só. A vantagem não é só a contagem de entidades: uma malha não
# tem obrigação de ser plana, então a dobra não precisa ser contornada -- pode ser uma aresta
# da geometria, que é o que ela é no terreno. A faixa é recortada por cada dobra que cruza e
# todo pedaço que sai fica em exatamente um plano do terreno. Exato, sem nada para ajustar, e
# mais barato que a aproximação que ele substitui.

ROAD_COLOURS = {
    "latosol": C_LATOSOL,
    "surface": (0.66, 0.36, 0.22),
}


def road_spine(path, arc_step=2.5):
    """A linha central como estações: (x, y, rumo de entrada, rumo de saída).

    Uma reta dá as duas pontas, um arco uma estação a cada `arc_step`, e uma esquina viva uma
    estação cujos dois rumos diferem. Esta última é o motivo de a espinha ter esta forma e não
    a de uma poligonal simples: um vértice não é um ponto de uma curva, é um ponto onde a
    estrada aponta para dois lados ao mesmo tempo, e a borda de uma fita posta sobre ele
    precisa das duas direções para saber onde fica o próprio canto.
    """
    x, y, h = path.x0, path.y0, path.h0
    out = [[x, y, h, h]]
    for kind, a, b in path.segments:
        if kind == "s":
            if a <= 0.0:
                continue
            x, y = x + a * math.cos(h), y + a * math.sin(h)
            out.append([x, y, h, h])
            continue
        radius, sweep = a, b
        if radius <= 0.0:
            out[-1][3] = h + sweep          # o vértice, entra em h, sai além dele
            h += sweep
            continue
        cx, cy, sign = path._arc_centre(x, y, h, radius, sweep)
        n = max(2, int(abs(sweep) * radius / arc_step))
        for i in range(1, n + 1):
            ang = h + sweep * i / n
            x = cx + sign * radius * math.sin(ang)
            y = cy - sign * radius * math.cos(ang)
            out.append([x, y, ang, ang])
        h += sweep
    return [tuple(s) for s in out]


def _mitre(h_in, h_out):
    """Para que lado, e quanto, a borda da estrada se afasta numa estação.

    Numa reta ou num arco isto é só a normal da esquerda. Num vértice é onde as duas linhas
    deslocadas se cruzam, que é essa normal esticada por 1 / cos(metade da curva) -- 1,41 num
    ângulo reto. Isso não é enfeite: é exatamente a esquina em esquadro que uma motoniveladora
    deixa, e sai das mesmas duas linhas de conta de qualquer outra estação, o que permitiu
    eliminar o remendo feito à mão sobre cada vértice. O remendo era um quadrado de estrada
    posto sobre o entalhe, e tinha de ser recortado nas dobras como tudo o mais.
    """
    nx_i, ny_i = -math.sin(h_in), math.cos(h_in)
    nx_o, ny_o = -math.sin(h_out), math.cos(h_out)
    k = 1.0 + nx_i * nx_o + ny_i * ny_o
    if k < 0.2:                    # uma curva perto de 180 graus não tem mitra honesta
        return nx_o, ny_o
    return (nx_i + nx_o) / k, (ny_i + ny_o) / k


def _cut_x(poly, f):
    """Um polígono convexo cortado pela reta vertical x = f, em até dois."""
    low, high = [], []
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        if ax <= f:
            low.append((ax, ay))
        if ax >= f:
            high.append((ax, ay))
        if (ax - f) * (bx - f) < 0:
            t = (f - ax) / (bx - ax)
            crossing = (f, ay + (by - ay) * t)
            low.append(crossing)
            high.append(crossing)
    return [p for p in (low, high) if len(p) >= 3]


def _one_plane_each(poly, folds):
    """Divide um polígono até nenhum pedaço dele cavalgar uma dobra do terreno."""
    out = [poly]
    for f in folds:
        cut = []
        for p in out:
            xs = [px for px, _ in p]
            if min(xs) < f - 1e-9 and max(xs) > f + 1e-9:
                cut += _cut_x(p, f)
            else:
                cut.append(p)
        out = cut
    return out


def _area(poly):
    s = 0.0
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        s += ax * by - bx * ay
    return abs(s) / 2


def _tri_normal(a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length < 1e-12:
        return None
    return tuple(round(v / length, 4) + 0.0 for v in (nx, ny, nz))


def ribbon(spine, terrain, z, lo, hi):
    """Uma faixa de superfície colada ao terreno, como uma lista de triângulos.

    `lo` e `hi` são deslocamentos laterais em metros, positivos à esquerda do sentido de
    marcha: um carreador é (-w/2, +w/2) e uma trilha de roda é uma faixa estreita bem fora do
    meio dele. Cada vértice fica no terreno no próprio x e com z acima dele, e cada triângulo
    está num plano do terreno, porque os quadriláteros são recortados pelas dobras antes de
    serem triangulados.
    """
    folds = sorted(f for f, _ in terrain.nodes) if terrain is not None else []
    rail = []
    for x, y, h_in, h_out in spine:
        mx, my = _mitre(h_in, h_out)
        rail.append(((x + mx * hi, y + my * hi), (x + mx * lo, y + my * lo)))

    def on_ground(p):
        return (p[0], p[1], (terrain.z(p[0]) if terrain is not None else 0.0) + z)

    tris = []
    for (l0, r0), (l1, r1) in zip(rail, rail[1:]):
        for piece in _one_plane_each([l0, r0, r1, l1], folds):
            if _area(piece) < 1e-4:
                continue
            pts = [on_ground(p) for p in piece]
            tris += [(pts[0], pts[k], pts[k + 1]) for k in range(1, len(pts) - 1)]
    return tris


def obj_mesh(groups, note, mtl):
    """Um OBJ com todas as estradas de um mundo, um objeto nomeado por faixa.

    O SDF as escolhe com `<submesh>`, então os modelos mantêm os nomes que sempre tiveram e cada
    um ainda custa exatamente um visual.
    """
    verts, vseen, norms, nseen, body = [], {}, [], {}, []
    for group, (colour, tris) in groups.items():
        body += [f"o {group}", f"usemtl {colour}"]
        for tri in tris:
            normal = _tri_normal(*tri)
            if normal is None:
                continue
            ids = []
            for p in tri:
                key = tuple(round(v, 4) + 0.0 for v in p)
                if key not in vseen:
                    vseen[key] = len(verts) + 1
                    verts.append(key)
                ids.append(vseen[key])
            if normal not in nseen:
                nseen[normal] = len(norms) + 1
                norms.append(normal)
            n = nseen[normal]
            body.append(f"f {ids[0]}//{n} {ids[1]}//{n} {ids[2]}//{n}")
    head = [f"# {note}",
            "# Written by scripts/generate_agriculture_worlds.py, edit there.",
            "# Flat shaded on purpose: one normal per face, because a road here",
            "# is planar between the folds and creased at them.",
            "", f"mtllib {mtl}", ""]
    return "\n".join(head
                     + [f"v {x:.4f} {y:.4f} {z:.4f}" for x, y, z in verts]
                     + [f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in norms]
                     + body + [""])


def mtl_text(groups, note):
    """As cores que o OBJ nomeia, escritas uma vez cada, na ordem do primeiro uso."""
    out = [f"# {note}",
           "# Written by scripts/generate_agriculture_worlds.py, edit there.", ""]
    for colour in dict.fromkeys(c for c, _ in groups.values()):
        r, g, b = ROAD_COLOURS[colour]
        out += [f"newmtl {colour}",
                f"Ka {r * 0.7:.3f} {g * 0.7:.3f} {b * 0.7:.3f}",
                f"Kd {r:.3f} {g:.3f} {b:.3f}",
                "Ks 0.050 0.050 0.050",
                "Ns 8.0", "d 1.0", "illum 2", ""]
    return "\n".join(out)


# O que é posto sobre o quê, em metros acima do terreno. A ordem é a ordem em que uma estrada é
# de fato construída, e tem de seguir por CAMADA e não por estrada, lição que este arquivo
# aprendeu do jeito difícil.
#
# Ele empilhava cada estrada como uma unidade -- terra, superfície, linha, depois a estrada
# seguinte um milímetro acima -- e isso enterra a linha de cada carreador sob a superfície de
# qualquer carreador que se junte a ele. Sete metros de superfície de rolagem cruzando a 30
# graus cobrem catorze metros da linha que ela cruza, então a linha de referência no meio da
# estrada principal simplesmente parava por catorze metros em cada junção. Um trator seguindo
# essa linha pela câmera seguia o que sobrava, que era a linha da estrada que se junta, e saía
# pela estrada de 30 graus para dentro do talhão.
#
# O meio milímetro dentro de uma faixa separa as estradas entre si, porque faces coplanares são
# o que um renderizador não consegue escolher -- dois carreadores se cruzando na mesma altura
# tremulam quando a câmera se move.
ROAD_LAYERS = {"earth": 0.010, "surface": 0.014}
ROAD_LAYER_STEP = 0.0005
PAINT_Z = 0.018          # os números pintados, acima de toda superfície de estrada


def road_parts(world, tag, path, width, earth_half, lift, terrain):
    """Um carreador: os modelos SDF dele e os grupos da malha que eles nomeiam.

    Duas faixas, cada uma um visual: a terra nua que a estrada desgasta e a superfície de
    rolagem aplainada por cima dela. A altura de cada uma está em `ROAD_LAYERS` acima; `lift` é
    o lugar desta estrada dentro da faixa dela.

    Um carreador não leva mais nada. Duas coisas foram tentadas no meio dele e as duas saíram --
    uma linha branca pintada, porque ninguém pinta um caminho de fazenda, e depois um par de
    trilhas de roda, porque listras de borda dura a 2,1 m uma da outra também parecem tinta e
    não coincidem com a bitola de nenhuma máquina da propriedade. O que sobra é o que uma
    estrada aplainada é: terra nua e a faixa mais estreita nela que o tráfego mantém lisa.
    """
    spine = road_spine(path)
    uri = f"model://carreador/meshes/{world}.obj"
    plan = [("earth", "latosol", ROAD_LAYERS["earth"] + lift,
             [(-earth_half, earth_half)]),
            ("surface", "surface", ROAD_LAYERS["surface"] + lift,
             [(-width / 2, width / 2)])]

    parts, groups = [], {}
    for layer, colour, z, bands in plan:
        group = f"{tag}_{layer}"
        groups[group] = (colour, [t for lo, hi in bands
                                  for t in ribbon(spine, terrain, z, lo, hi)])
        parts.append("\n".join([
            f'<model name="road_{group}">', "  <static>true</static>",
            '  <link name="link">',
            f'    <visual name="{layer}">',
            "      <geometry><mesh>",
            f"        <uri>{uri}</uri>",
            f"        <submesh><name>{group}</name></submesh>",
            "      </mesh></geometry>",
            "    </visual>", "  </link>", "</model>"]))
    return parts, groups


# Sete segmentos, e quais deles cada dígito acende. Só os seis dígitos que aparecem num ângulo
# desta fazenda estão aqui; um `KeyError` é a resposta certa para qualquer outro, porque a
# placa ficaria ilegível em vez de errada.
SEGMENTS = {
    "0": "abcdef", "3": "abcdg", "4": "bcfg",
    "5": "acdfg", "6": "acdefg", "9": "abcdfg",
}

def ground_number(name, x, y, text, digit_h=4.2, digit_w=2.5, stroke=0.7, gap=1.0,
                  margin=0.9, board=(0.92, 0.90, 0.80), ink=(0.16, 0.08, 0.06),
                  z=PAINT_Z, terrain=None):
    """Um número pintado deitado no chão: dígitos lado a lado, norte para cima.

    A primeira versão escrevia as cifras do jeito que uma pista escreve SLOW -- esticadas,
    empilhadas uma depois da outra no sentido da marcha, cada uma lida ao passar sobre ela.
    Numa estrada que é vista de cima e não dirigida, isso é ilegível: os dígitos chegam um de
    cada vez e no ângulo em que a curva por acaso aponta, então um 30 e um 90 se parecem muito.

    Então: dígitos lado a lado num só número, e todo número da propriedade disposto com o norte
    para cima, qualquer que seja a direção da estrada. Uma orientação consistente é o que torna
    um conjunto de números varrível com os olhos; quem olha a fazenda de cima não precisa
    descobrir qual é o lado certo de cada um. As cifras ficam sobre uma placa pintada, porque
    escuro sobre claro se lê de relance e claro sobre latossolo vermelho não.

    O conjunto todo tem menos de 10 m na diagonal, o que permite que caiba nos 11 m de terra nua
    em qualquer rumo de estrada.
    """
    n = len(text)
    span = n * digit_w + (n - 1) * gap
    # Uma pose de modelo, com as lajes como deslocamentos dentro dela. Isso só funciona porque a
    # placa nunca é posta sobre uma dobra do terreno: a busca de posição abaixo recusa esses
    # pontos, então o número inteiro sempre fica num plano só.
    placed = terrain.pose_on(x, y, 0.0) if terrain else (x, y, 0.0, 0.0, 0.0, 0.0)
    out = [f'<model name="{name}">', "  <static>true</static>",
           f"  {pose(*placed)}", '  <link name="link">']

    def slab(vis, size, gx, gy, rgb, zz):
        return [f'    <visual name="{vis}">',
                f"      {pose(gx, gy, zz)}",
                f"      <geometry><box><size>{_n(size[0])} {_n(size[1])} 0.02"
                "</size></box></geometry>",
                f"{indent(material(rgb), '      ')}", "    </visual>"]

    out += slab("board", (span + 2 * margin, digit_h + 2 * margin), 0, 0, board, z)

    h2, w2 = digit_h / 2, digit_w / 2
    for k, ch in enumerate(text):
        dx = (k - (n - 1) / 2.0) * (digit_w + gap)
        places = {
            "a": ((digit_w, stroke), dx, h2),
            "g": ((digit_w, stroke), dx, 0.0),
            "d": ((digit_w, stroke), dx, -h2),
            "f": ((stroke, digit_h / 2), dx - w2, digit_h / 4),
            "b": ((stroke, digit_h / 2), dx + w2, digit_h / 4),
            "e": ((stroke, digit_h / 2), dx - w2, -digit_h / 4),
            "c": ((stroke, digit_h / 2), dx + w2, -digit_h / 4),
        }
        for seg in SEGMENTS[ch]:
            size, gx, gy = places[seg]
            out += slab(f"d{k}{seg}", size, gx, gy, ink, z + 0.002)

    out += ["  </link>", "</model>"]
    return "\n".join(out)


def plan_map(roads, matas, rows, props, paint=(), wood=(), cell=5.0):
    """Rasteriza o que acabou de ser construído, para o leiaute poder ser lido sem GUI.

    Isto vai para o cabeçalho do próprio mundo. Um campo deste formato não é algo que alguém
    confira lendo seiscentas poses, e nem eu consigo conferir olhando uma tela.
    """
    xs = [p[0] for r in roads for p in r] + [x for row in rows for x in (row[0], row[2])]
    ys = [p[1] for r in roads for p in r] + [y for row in rows for y in (row[1], row[3])]
    x0, x1, y0, y1 = min(xs) - 6, max(xs) + 6, min(ys) - 6, max(ys) + 6
    nx, ny = int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1
    grid = [[" "] * nx for _ in range(ny)]

    # A prioridade é a ordem desta string, então uma estrada desenhada sobre uma fileira vence e
    # a origem vence tudo. Os três caracteres de fileira ficam juntos porque nunca dividem uma
    # célula: um talhão, uma copa.
    order = " .:o*%#=+TO"
    ROW_CHAR = {"mesh": ":", "round": "o", "fruit": "*", "round_fruit": "%"}

    def put(x, y, ch, priority):
        i, j = int((x - x0) / cell), int((y - y0) / cell)
        if 0 <= i < nx and 0 <= j < ny:
            if order.index(grid[ny - 1 - j][i]) < priority:
                grid[ny - 1 - j][i] = ch

    for x, y in wood:
        put(x, y, ".", 1)
    for poly in matas:
        for x in range(int(x0), int(x1), 2):
            for y in range(int(y0), int(y1), 2):
                if in_polygon(x, y, poly):
                    put(x, y, "#", order.index("#"))
    for rx0, ry0, rx1, ry1, kind in rows:
        ch = ROW_CHAR[kind]
        steps = max(2, int(math.hypot(rx1 - rx0, ry1 - ry0) / 2))
        for k in range(steps + 1):
            put(rx0 + (rx1 - rx0) * k / steps, ry0 + (ry1 - ry0) * k / steps,
                ch, order.index(ch))
    for pts in roads:
        for x, y in pts:
            put(x, y, "=", order.index("="))
    for x, y in paint:
        put(x, y, "+", order.index("+"))
    for x, y in props:
        put(x, y, "T", order.index("T"))
    put(0, 0, "O", order.index("O"))

    legend = ("    = carreador    : lumpy crown    o smooth crown\n"
              "    * lumpy with fruit    % smooth with fruit\n"
              "    # mata, of forest_tree    . loose tree or rock"
              "    + angle painted on the road\n"
              "    T machinery    O origin"
              f"    one character is {_n(cell)} m")
    body = "\n".join("    |" + "".join(line) + "|" for line in grid)
    return f"{legend}\n\n{body}"


def check_props(name, net, props, clear=15.0):
    """Nada que uma máquina teria de contornar pode ficar num carreador.

    As estradas são a única parte destes mundos a que um veículo sem motorista tem direito: os
    talhões são obstáculos por definição e a cabeceira é onde as coisas são deixadas, mas um
    carreador com um tanque de água no meio é uma estrada que não vai a lugar nenhum. Quinze
    metros da linha central são os 6 m de terra nua que a estrada desgasta, mais espaço para
    passar pelo que estiver estacionado sem pôr uma roda na lavoura.

    `props` é (nome, x, y) para tudo posto no chão: o pátio, as máquinas, as caixas, o galpão.
    """
    lines = [q for _, path in net for q in path.points(2.0)]
    grid = NearGrid(lines)
    bad = [f"{what} at ({x:.0f}, {y:.0f})" for what, x, y in props
           if grid.within(x, y, clear)]
    if bad:
        raise ValueError(f"{name}: standing in a carreador: " + "; ".join(bad))


def check_network(name, net, bend_spacing=0.0, oblique=False, pairs=False):
    """Recusa construir um mundo cujas estradas estão erradas onde isso não aparece.

    Cada uma destas foi uma falha real antes de ser uma verificação. O terceiro carreador do
    citrus_farm_60 saía da propriedade pelo limite sul, corria 180 dos seus 270 m fora do
    hexágono e voltava: na planta isso é uma estrada e um limite que se cruzam duas vezes com
    uma lente entre eles, o que parece um oito, e no mundo é uma estrada pelo campo de ninguém.
    Ela sobreviveu porque `close_to_boundary` só olha onde a estrada *termina*. Nada olhava
    por onde ela tinha passado.

    As três regras duras, na ordem em que são mais baratas de quebrar:

      `inside`      a linha central de uma estrada fica dentro do limite. Seis metros de
                  tolerância, porque o último ponto de uma estrada fica no limite por
                  construção e o ponto flutuante o põe de um lado ou do outro da linha.
      `ends on it`  as duas pontas estão no limite, ou num T contra outro carreador. Uma estrada
                  que para sem nada no fim é uma estrada de onde a máquina tem de sair de
                  ré; uma estrada que termina em outra entrou na rede.
      `not beside`  nenhuma estrada corre a menos de 32 m de outra, ou do limite, apontando para
                  o mesmo lado, por mais de 25 m. Cruzar é uma junção e junções são o ponto;
                  correr ao lado é uma bagunça larga com uma tira de sebe encalhada no meio.
                  O limite conta: um carreador posto 20 m dentro do perímetro é a mesma
                  bagunça, e a primeira versão desta verificação deixava isso passar por
                  comparar só carreadores.
      `not square`  opcional, e só o mundo misto pede: uma estrada não pode encontrar o limite a
                  menos de 15 graus de um ângulo reto. Um carreador que termina em esquadro no
                  limite desenha uma esquina exatamente como uma curva de 90 graus, e essa
                  esquina não leva número, então um mundo que pinta 90 nas curvas está
                  dizendo ao leitor que os ângulos retos sem pintura são outra coisa. Não são.
                  No mundo misto foi isto que afastou o limite de ser um quadrado: um anel
                  irregular tem arestas em vários rumos e uma estrada sempre consegue
                  encontrar uma delas a 30 ou 60. Os mundos de ângulo único também não adotam
                  esta regra, e pelo mesmo motivo não adotam a do espaçamento: num mundo em que
                  toda curva é de 90 e toda esquina é pintada 90, uma estrada que encontra o
                  limite em esquadro é o mundo sendo coerente consigo mesmo.
      `on the grid` o carreador que carrega a origem corre ao longo de x, com 3 graus de
                  tolerância. O veículo nasce em 0 0 apontando para +x sem yaw por mundo, então
                  a estrada sob ele tem de apontar para lá também: uma máquina que começa 20
                  graus atravessada no próprio carreador é uma máquina cuja primeira manobra é
                  uma correção.
      `both forms`  opcional: cada ângulo aparece tanto arredondado quanto como esquina viva. Uma
                  fazenda vira dos dois jeitos -- abaulada onde há espaço, cortada em esquadro
                  onde dois blocos se encontram -- e um mundo feito para perguntar o que uma
                  curva custa tem de levar os dois de cada, ou só pergunta sobre um deles.
      `bends apart` opcional, e só o mundo misto pede: curvas consecutivas têm de deixar
                  `bend_spacing` metros de reta de verdade entre elas, onde de verdade quer
                  dizer depois de as duas terem comido o raio * tan(metade da curva) da reta que
                  as une. Um 60 e um 90 de raio 24 a 60 m um do outro no papel ficam a 22 m no
                  terreno, e dois arcos tão próximos parecem uma varredura longa -- exatamente o
                  que um 90 não pode parecer no mundo cujo objetivo é distinguir ângulos. Os
                  mundos de ângulo único não adotam esta regra: três 30 em seguida é como uma
                  estrada de 30 vira uma esquina ali.

    Cruzar outro carreador é só relatado, não imposto: uma estrada que corre de borda a borda e
    não encontra nada além do perímetro é mais fina que o projeto quer, e a estrada sul do
    citrus_farm_90 é uma, mas ela está conectada e não está errada do jeito que as três acima
    estão.
    """
    pts = {tag: p.points(1.0) for tag, p in net}
    bound = pts["perimeter"]
    roads = [t for t in pts if t != "perimeter"]
    bad = []

    def off(q):
        if in_polygon(q[0], q[1], bound):
            return 0.0
        return min(math.dist(q, b) for b in bound)

    def heading(line, i):
        a, b = line[max(0, i - 1)], line[min(len(line) - 1, i + 1)]
        return math.atan2(b[1] - a[1], b[0] - a[0])

    def square_at(line, i, other):
        """O ângulo que este ponto de `line` faz com o trecho mais próximo de `other`."""
        j = min(range(len(other)), key=lambda k: math.dist(line[i], other[k]))
        d = abs((heading(line, i) - heading(other, j) + math.pi)
                % (2 * math.pi) - math.pi)
        return math.degrees(min(d, math.pi - d))

    def meets(a, b):
        for i in range(len(a) - 1):
            for j in range(len(b) - 1):
                if segments_cross(a[i], a[i + 1], b[j], b[j + 1]):
                    return True
        return False

    for tag in roads:
        line = pts[tag]
        stray = max(off(q) for q in line)
        if stray > 6.0:
            bad.append(f"{tag} leaves the property by {stray:.0f} m")
        for end, i in (("starts", 0), ("ends", len(line) - 1)):
            # No limite, ou noutro carreador. Uma estrada que termina em T contra uma estrada que
            # segue para os dois lados não terminou no meio de um campo, entrou na rede; só uma
            # estrada que para sem nada no fim terminou.
            gap = min(math.dist(line[i], b) for b in bound)
            joined = min((math.dist(line[i], q) for o in roads if o != tag
                          for q in pts[o]), default=1e9)
            if min(gap, joined) > 4.0:
                bad.append(f"{tag} {end} {min(gap, joined):.0f} m from "
                           f"anything it could join")
            met = square_at(line, i, bound)
            if oblique and abs(met - 90) < 15:
                bad.append(f"{tag} {end} on the boundary at {met:.0f} degrees, "
                           f"which is a corner nobody painted a number on")

    for i, a in enumerate(roads):
        for b in roads[i + 1:] + ["perimeter"]:
            side = 0
            for k, q in enumerate(pts[a]):
                j = min(range(len(pts[b])), key=lambda m: math.dist(q, pts[b][m]))
                if math.dist(q, pts[b][j]) < 32.0:
                    d = abs((heading(pts[a], k) - heading(pts[b], j) + math.pi)
                            % (2 * math.pi) - math.pi)
                    if math.degrees(min(d, math.pi - d)) < 25.0:
                        side += 1
            if side > 25:
                bad.append(f"{a} and {b} run alongside each other for {side} m")

    # A origem, e o que o veículo encontra quando nasce ali.
    on_grid = [(t, p) for t, p in ((t, pts[t]) for t in roads)
               if min(math.hypot(q[0], q[1]) for q in p) < 3.5]
    if not on_grid:
        near = min(min(math.hypot(q[0], q[1]) for q in pts[t]) for t in roads)
        bad.append(f"the origin is {near:.0f} m from the nearest carreador")
    else:
        # Um deles tem de correr ao longo de x, não todos: um cruzamento na origem põe duas
        # estradas passando por ela em ângulo reto, e o veículo está sobre as duas.
        along = []
        for tag, line in on_grid:
            i = min(range(len(line)), key=lambda k: math.hypot(line[k][0], line[k][1]))
            off = abs(math.degrees(heading(line, i)))
            along.append((tag, min(off, abs(off - 180))))
        if not any(off <= 3.0 for _, off in along):
            worst = ", ".join(f"{t} at {o:.0f} degrees" for t, o in along)
            bad.append(f"nothing through the origin runs along the x axis ({worst}), "
                       f"so a vehicle spawned there is across its own carreador")

    if pairs:
        # O limite conta. Uma propriedade é cortada em esquadro nos cantos muito mais vezes que
        # um carreador, então os ângulos retos de uma fazenda ficam na borda e os ângulos que
        # cortam os blocos ficam por dentro: o mundo misto tira os dois 90 do perímetro, um
        # abaulado e um cortado em esquadro, e os 30 e 60 das diagonais.
        seen = {}
        for tag, path in net:
            for deg, radius in path.turns():
                seen.setdefault(abs(round(deg)), set()).add(radius > 0)
        for want in (30, 60, 90):
            got = seen.get(want, set())
            if got != {True, False}:
                have = ("nothing" if not got else
                        "only rounded ones" if got == {True} else "only sharp ones")
                bad.append(f"{want} degrees: {have}, and it wants one of each")

    if bend_spacing:
        for tag, path in net:
            if tag == "perimeter":
                continue
            marks = path.marks()
            for (_, _, _, da, ra, _), (_, _, _, db, rb, run) in zip(marks, marks[1:]):
                clear = (run - ra * math.tan(abs(math.radians(da)) / 2)
                         - rb * math.tan(abs(math.radians(db)) / 2))
                if clear < bend_spacing:
                    bad.append(f"{tag}: the {abs(round(da))} and the "
                               f"{abs(round(db))} leave {clear:.0f} m of straight "
                               f"between them, under the {bend_spacing:.0f} asked for")

    if bad:
        raise ValueError(f"{name}: " + "; ".join(bad))

    for tag in roads:
        line = pts[tag]
        joined = any(meets(line, pts[o]) for o in roads if o != tag)
        # Um T conta. Uma estrada que termina em outra a encontrou, e se o último ponto da
        # poligonal cai um milímetro antes da outra linha central é aritmética, não geografia.
        if not joined:
            joined = any(math.dist(line[i], q) < 4.0
                         for i in (0, len(line) - 1)
                         for o in roads if o != tag for q in pts[o])
        if not joined:
            print(f"  note: {name}: {tag} meets no other carreador")


def segments_cross(p, q, r, s):
    """Verdadeiro se pq e rs se encontram. As pontas contam: estradas são unidas, não soldadas."""
    d = (q[0] - p[0]) * (s[1] - r[1]) - (q[1] - p[1]) * (s[0] - r[0])
    if abs(d) < 1e-12:
        return False
    t = ((r[0] - p[0]) * (s[1] - r[1]) - (r[1] - p[1]) * (s[0] - r[0])) / d
    u = ((r[0] - p[0]) * (q[1] - p[1]) - (r[1] - p[1]) * (q[0] - p[0])) / d
    return 0 <= t <= 1 and 0 <= u <= 1


def _build_farm(name, network, blocks, opening, loops_note, step=0.002,
                bend_spacing=0.0, oblique=False, pairs=False, extent=260.0,
                woods=None, wood_trees=30):
    """A propriedade inteira: terreno, estradas, talhões, mata, máquinas.

    `network` é o leiaute de estradas, como (tag, Path). Uma tag tem de ser "perimeter", o
    limite, que se fecha sobre si mesmo; uma tem de ser "main", o carreador em que o pátio
    está, que tem de passar pela origem. `blocks` é o plantio, como (tag, x do centro, y do
    centro, meio comprimento, meia largura, direção das fileiras), tentados em ordem e vencendo
    a primeira reivindicação.

    Todo o resto é compartilhado entre os mundos construídos a partir disto: o terreno, a mata,
    o jeito de as fileiras serem cortadas no que as estradas deixam. Mude só o leiaute das
    estradas e nada mais, e o que um robô faz de diferente se deve às estradas.

    Com qual copa cada talhão é plantado está em `PLANTING`, e é a mesma tabela nos três mundos:
    onze blocos de sequências de malha assada, dois de elipsoides redondos, um de citros com
    frutos, e toda mata de `forest_tree`. Quatro modelos de árvore numa propriedade, para dois
    deles poderem ser comparados sem sair do mundo -- e para o custo dos caros ser limitado por
    um bloco e não espalhado por uma propriedade.
    """
    check_network(name, network, bend_spacing, oblique, pairs)

    ROW_SPACING = 6.5
    TREE_SPACING = 3.0
    CANOPY_W = 3.5

    # Uma largura de carreador para a propriedade inteira: 7 m de superfície de rolagem dentro de
    # 11 m de terra nua, de sebe a sebe. Uniforme, porque a coisa mais larga que usa uma estrada
    # de fazenda é a mesma em toda parte.
    ROAD_W, SHOULDER = 7.0, 2.0
    CLEAR = ROAD_W / 2 + SHOULDER + 0.5 + CANOPY_W / 2

    # --- o relevo ------------------------------------------------------------
    # Um perfil de leste a oeste, em nós por onde o terreno passa: uma encosta caindo 11% a
    # partir do limite oeste, um fundo de vale perto de x = -90, depois uma subida de 10% até
    # uma crista em x = 160 e descendo além da borda leste. Dezesseis metros de relevo, que é o
    # que é preciso para ser inconfundível neste tamanho; a primeira tentativa, com oito, estava
    # lá e não dava para ver. Nada aqui é íngreme para citros, que são plantados em pior, e o
    # pátio em volta da origem é nivelado, como um pátio é.
    # O mesmo perfil seja qual for a propriedade, esticado para alcançá-la: `extent` é quanto o
    # terreno tem de ir a partir da origem, e um mundo maior que os 260 m para os quais o perfil
    # foi escrito recebe os declives das pontas prolongados e não um relevo novo. Terreno que
    # para antes do limite é um buraco por onde uma máquina cai.
    #
    # citrus_farm pede 460 m para uma propriedade que chega a 332, e o motivo é a vista aérea.
    # Terreno que termina oito metros depois da última árvore é um penhasco na borda do mundo, e
    # qualquer foto aérea do lugar mostra a fazenda toda flutuando sobre o nada. Corrigir não
    # custa entidades -- as lajes são as mesmas caixas, mais compridas -- e uma fazenda deve
    # ficar numa paisagem e não sobre uma mesa.
    profile = [(-260, 14.0), (-195, 7.0), (-140, 1.0), (-90, -2.5),
               (-46, -1.2), (-25, 0.0), (25, 0.0), (46, 1.4),
               (110, 8.0), (160, 10.5), (195, 7.0), (260, 2.0)]
    if extent > 260.0:
        west_grade = (profile[1][1] - profile[0][1]) / (profile[1][0] - profile[0][0])
        east_grade = (profile[-1][1] - profile[-2][1]) / (profile[-1][0] - profile[-2][0])
        profile = ([(-extent, profile[0][1] - west_grade * (extent - 260.0))]
                   + profile
                   + [(extent, profile[-1][1] + east_grade * (extent - 260.0))])
    terrain = Terrain(profile, y_half=max(250.0, extent))

    # --- as estradas ---------------------------------------------------------
    # Nada pode terminar no meio de um campo: toda estrada vai de uma borda da propriedade a
    # outra ou de uma estrada a outra, e o perímetro se fecha sobre si mesmo, então não há onde
    # uma máquina tenha de sair de ré.
    by_tag = {tag: path for tag, path in network}
    perimeter, main = by_tag["perimeter"], by_tag["main"]
    bound = perimeter.points(2.0)
    roads = [(tag, path, ROAD_W, SHOULDER, CLEAR) for tag, path in network]

    # --- a mata --------------------------------------------------------------
    # Onde ficam as matas. O padrão é uma mancha em cada bloco, no terreno sobrando que um laço
    # deixa no canto, em torno do que os mundos de estrada passante foram desenhados.
    #
    # `woods` o sobrescreve, e o mundo do cruzamento usa isso para pôr a mata onde uma mata de
    # fato fica: fora da cerca, em volta de tudo. Um bloco de citros com um mato no meio é um
    # bloco que alguém plantou em volta de um mato, o que acontece, mas quatro deles são quatro
    # vezes uma coincidência. O que as fotografias aéreas mostram é o terreno plantado parando
    # na divisa da propriedade e as árvores tomando conta do lado de fora.
    #
    # `wood_trees` é quantas entram em cada mancha. Um cinturão são muitas manchas e quer menos
    # em cada: trinta em cada uma de doze manchas são 360 árvores a três visuais cada, que é
    # metade do mundo de novo em cenário por onde nada passa.
    matas = woods if woods is not None else [
        blob(150, 150, 26, 20, seed=11),
        blob(-150, 148, 24, 19, seed=22),
        blob(150, -150, 25, 18, seed=33),
        blob(-146, -150, 23, 20, seed=44)]

    # --- o pátio -------------------------------------------------------------
    # Escolhido aqui, antes de qualquer coisa ser plantada, porque a clareira que mantém as
    # fileiras fora dele é derivada de onde ele cai. A primeira versão procurava o pátio depois
    # de o plantio já estar posto, então o pátio mudava e a clareira não: um galpão com citros a
    # quatro metros da parede, o que não é um pátio, é um galpão que alguém largou num bloco.
    #
    # A forma é fixa -- a carreta engatada atrás do trator, o galpão ao norte dos dois, o tanque
    # no lado oposto, as caixas junto ao portão -- e só a âncora é procurada, perto da origem,
    # longe das estradas.
    YARD = {"tractor": (4.0, 0.5), "trailer": (-1.9, 0.5), "shed": (0.0, 8.0),
            "tank": (-16.0, 0.0), "bin0": (13.0, -3.0), "bin1": (13.0, -1.4),
            "bin2": (11.3, -3.0)}
    YARD_CLEAR = 30.0

    all_roads = NearGrid([q for _, path in network for q in path.points(2.0)])

    def yard_at(ax, ay):
        return {k: (ax + dx, ay + dy) for k, (dx, dy) in YARD.items()}

    anchor = None
    for radius_out in (0, 20, 40, 60, 80):
        for ax, ay in ((-24, 22 + radius_out), (-24, 22 - radius_out),
                       (-24 - radius_out, 22), (-24 + radius_out, 22),
                       (-24 - radius_out, 22 + radius_out),
                       (-24 + radius_out, 22 - radius_out)):
            spots = yard_at(ax, ay)
            if any(all_roads.within(x, y, 17.0) for x, y in spots.values()):
                continue
            if any(not in_polygon(x, y, bound) for x, y in spots.values()):
                continue
            if any(in_polygon(x, y, poly) for x, y in spots.values() for poly in matas):
                continue
            anchor = (ax, ay)
            break
        if anchor:
            break
    if anchor is None:
        raise ValueError(f"{name}: nowhere near the origin to put the yard")
    yard = yard_at(*anchor)

    # A clareira é o terreno do próprio pátio: nada é plantado dentro dela, e ela é centrada onde
    # o pátio realmente foi parar.
    clearings = [(anchor[0], anchor[1], YARD_CLEAR)]

    road_grids = [(NearGrid(p.points(0.75)), clear)
                  for _, p, _, _, clear in roads]

    parts = [terrain.slabs("terrain", C_GRASS)]

    # --- a mata, plantada ----------------------------------------------------
    rng = random.Random(5150)
    mata_pts: list[tuple[float, float]] = []
    wood: list[tuple[str, float, float, float, float]] = []
    for m, poly in enumerate(matas):
        xs, ys = [q[0] for q in poly], [q[1] for q in poly]
        placed = 0
        for _ in range(8000):
            if placed >= wood_trees:
                break
            x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
            if not in_polygon(x, y, poly):
                continue
            if any(math.hypot(x - a, y - b) < 6.0 for a, b in mata_pts):
                continue
            if any(g.within(x, y, cl - CANOPY_W / 2 + 2.0) for g, cl in road_grids):
                continue
            mata_pts.append((x, y))
            wood.append(("forest_tree", x, y, terrain.z(x),
                         rng.uniform(0, 2 * math.pi)))
            placed += 1
    mata_grid = NearGrid(mata_pts)

    MATA_CLEAR = 2.6 + CANOPY_W / 2 + 1.0
    BLOCK_GAP = 4.5

    def plantable(x, y, mine):
        if not in_polygon(x, y, bound):
            return False                        # fora do limite da propriedade
        for tag, bx, by, hl, hw, yaw in blocks:
            if tag == mine:
                break
            dx, dy = x - bx, y - by
            c, s = math.cos(-yaw), math.sin(-yaw)
            if (abs(dx * c - dy * s) <= hl + BLOCK_GAP
                    and abs(dx * s + dy * c) <= hw + BLOCK_GAP):
                return False
        for grid, clear in road_grids:
            if grid.within(x, y, clear):
                return False
        for poly in matas:
            if in_polygon(x, y, poly):
                return False
        if mata_grid.within(x, y, MATA_CLEAR):
            return False
        for cx, cy, r in clearings:
            if math.hypot(x - cx, y - cy) < r:
                return False
        return True

    row_segments = []
    hedge_shapes: dict[str, list[str]] = {"ne": [], "nw": [], "se": [], "sw": []}
    # ordenado, não na ordem do conjunto: um conjunto de strings itera na ordem que a semente de
    # hash deste processo dá, e os modelos saem deste dicionário nessa ordem. Duas execuções do
    # gerador escrevem então o mesmo mundo com os blocos em lugares diferentes do arquivo, o que
    # é um diff a cada vez e um gerador que não pode ser conferido com o que está versionado.
    special_shapes: dict[str, list[str]] = {k: [] for k in sorted(set(PLANTING.values()))}
    seed = 0
    for tag, bx, by, hl, hw, yaw in blocks:
        c, s = math.cos(yaw), math.sin(yaw)
        n_rows = int(2 * hw / ROW_SPACING)
        for r in range(n_rows):
            across = -hw + ROW_SPACING * (r + 0.5)
            run_start = None
            run_strip = None
            sample_step = 0.5                      # não `step`: esse é o passo da física
            samples = int(2 * hl / sample_step) + 1
            for k in range(samples + 1):
                along = -hl + k * sample_step
                x = bx + along * c - across * s
                y = by + along * s + across * c
                # Uma fileira é cortada numa dobra do terreno e também numa estrada: uma sebe, um
                # plano. Fileiras que terminam na crista de uma elevação são, de qualquer jeito,
                # o que um campo em terreno ondulado parece.
                here = terrain.strip(x)
                ok = plantable(x, y, tag) if k < samples else False
                if ok and run_strip is not None and here != run_strip:
                    ok = False
                if ok and run_start is None:
                    run_start, run_strip = (x, y), here
                elif not ok and run_start is not None:
                    seed += 1
                    kind = PLANTING.get(tag, "mesh")
                    shapes = hedgerow_shapes(f"{tag}_{r}_{seed}", *run_start,
                                             x, y, seed, TREE_SPACING,
                                             CANOPY_W, terrain=terrain,
                                             kind=kind)
                    if shapes:
                        if kind == "mesh":
                            # agrupado pelo lugar da fileira, não pelo bloco que a reivindicou:
                            # as tags de bloco diferem entre leiautes, os quadrantes não
                            quad = (("n" if y >= 0 else "s")
                                    + ("e" if x >= 0 else "w"))
                            hedge_shapes[quad].extend(shapes)
                        else:
                            # Um modelo por copa que não seja a barata, para o que ela custa
                            # poder ser contado, medido e retirado sem tocar o quadrante em que
                            # ela está.
                            special_shapes[kind].extend(shapes)
                        row_segments.append((*run_start, x, y, kind))
                    run_start, run_strip = None, None
                    if ok:                       # uma dobra, não um obstáculo
                        run_start, run_strip = (x, y), here

    for quad, shapes in hedge_shapes.items():
        if shapes:
            parts.append(hedges(f"hedges_{quad}", shapes))
    for kind, shapes in special_shapes.items():
        if shapes:
            parts.append(hedges(f"hedges_{kind}", shapes))

    # --- as estradas, desenhadas --------------------------------------------
    # Uma malha para todas elas, uma faixa nomeada dentro dela por camada por estrada. O que vai
    # acima do quê está em `ROAD_LAYERS`: por camada em todas as estradas, para a superfície de
    # nenhum carreador ficar jamais sobre a linha de outro carreador.
    road_groups = {}
    # As faixas só se sustentam se a estrada mais alta da faixa mais baixa ainda ficar abaixo da
    # faixa seguinte. Com sete estradas são 3 mm de escalonamento contra 4 mm de faixa, o que
    # funciona e não sobreviveria a nove.
    top = ROAD_LAYER_STEP * (len(roads) - 1)
    bands = sorted(ROAD_LAYERS.values()) + [PAINT_Z]
    assert all(b - a > top for a, b in zip(bands, bands[1:])), (
        f"{len(roads)} carreadores do not fit the layers in ROAD_LAYERS")
    for n, (tag, path, width, shoulder, clear) in enumerate(roads):
        models, groups = road_parts(name, tag, path, width,
                                    clear - CANOPY_W / 2,
                                    ROAD_LAYER_STEP * n, terrain)
        parts += models
        road_groups.update(groups)

    # --- o ângulo de cada curva, pintado na própria curva -------------------
    # As placas se foram; isto é o que as substituiu. Um número no chão não pode ser derrubado,
    # não precisa de um poste no acostamento e é legível diretamente de cima, que é de onde olha
    # quem examina um bloco inteiro.
    #
    # Ele vai no meio do arco, e antes ia 28 m atrás, ao longo da reta que levava até ele.
    # Recuá-lo é o que uma placa de trânsito faz, porque o motorista precisa de aviso; um número
    # pintado no chão não é um aviso, é um rótulo, e um rótulo numa reta vazia tem de ser
    # associado a uma curva a olho. Metade das vezes com a errada: numa propriedade de ângulos
    # mistos o número antes de uma curva e o depois dela estão ambos visíveis do mesmo lugar, e
    # nada diz qual pertence a qual. No arco não há dúvida -- a cifra está deitada na curva que
    # ela nomeia.
    #
    # O arco também é o trecho de estrada mais largo e plano disponível: a terra nua corre 11 m
    # pela curva como em toda parte, e a placa tem 7,8 por 6,0 m, então cabe dentro dela em
    # qualquer rumo.
    paint = []
    others = {tag: path.points(2.0) for tag, path in network}
    for tag, path in network:
        elsewhere = [q for t, pts_ in others.items() if t != tag for q in pts_]
        for m, (mx, my, mh, deg, radius, run) in enumerate(path.marks()):
            turn = math.radians(deg)
            cx, cy, sign = path._arc_centre(mx, my, mh, radius, turn)

            def on_arc(frac):
                """Um ponto ao longo da curva: 0 na entrada, 1 na saída."""
                a = mh + turn * frac
                return cx + sign * radius * math.sin(a), cy - sign * radius * math.cos(a)

            def past(end, metres):
                """`metres` além de uma ponta do arco, ao longo da reta."""
                if end < 0:
                    return (mx - metres * math.cos(mh), my - metres * math.sin(mh))
                h = mh + turn
                ex = cx + sign * radius * math.sin(h)
                ey = cy - sign * radius * math.cos(h)
                return ex + metres * math.cos(h), ey + metres * math.sin(h)

            def clear_of_roads(spot):
                return min((math.hypot(spot[0] - qx, spot[1] - qy)
                            for qx, qy in elsewhere), default=999.0)

            def on_fold(spot):
                return any(abs(spot[0] - f) < 6.0 for f, _ in terrain.nodes)

            # No meio da curva, depois de cada lado dela, e só então um pouco além das pontas. Um
            # número deitado atravessando um cruzamento é a única posição que não se lê de jeito
            # nenhum, então a busca desiste dela por último: 22 m de folga são os 5 m da própria
            # placa mais os 11 m de terra nua que a outra estrada desgasta, mais espaço para ver
            # a qual estrada o número pertence.
            # Primeiro o meio da curva, depois cada lado dele ao longo do arco, e só então alguns
            # metros além das pontas -- nunca longe o bastante para deixar de parecer o número
            # daquela curva. Doze metros é o teto, que no arco mais curto daqui é o próprio
            # comprimento da placa: além disso pareceria pertencer à reta.
            spots = ([on_arc(f) for f in (0.5, 0.42, 0.58, 0.34, 0.66,
                                          0.26, 0.74, 0.18, 0.82)]
                     + [past(e, m) for m in (6.0, 12.0, 18.0) for e in (1, -1)])
            # Todo número do mesmo tamanho, sempre. A placa encolhia para 68% quando a busca não
            # achava um ponto livre de dobra, e uma propriedade cujos números têm dois tamanhos
            # parece dizer que os menores significam outra coisa. Tamanho não é informação aqui;
            # os dígitos são. Então a busca move a placa e nunca a redimensiona, e tem mais
            # lugares para tentar.
            side = 0.0
            best = next((q for q in spots
                         if clear_of_roads(q) > 22.0 and not on_fold(q)), None)
            if best is None:
                # Uma curva com uma dobra passando por todos esses pontos. Mantém a regra da
                # junção, abre mão da regra da dobra e põe a placa ao lado do vinco e não
                # atravessada nele, para que ela toda ainda fique num plano só. Nenhum ponto desta
                # propriedade precisa mais disso -- a lista ampliada acima sempre acha um --, mas
                # uma curva posta num vinco mais tarde precisaria.
                best = next((q for q in spots if clear_of_roads(q) > 22.0), None)
                if best is not None:
                    near = min((f for f, _ in terrain.nodes),
                               key=lambda f: abs(best[0] - f))
                    gap = best[0] - near
                    side = math.copysign(2.85, gap if abs(gap) > 0.05 else 1.0)
            if best is None:
                # Nada perto desta curva está livre da estrada que ela cruza. Toma o ponto mais
                # livre que houver: um número ao lado de uma junção ainda se lê, e um número
                # ausente não.
                best = max(spots, key=clear_of_roads)
            px = best[0] - math.sin(mh + turn / 2) * side
            py = best[1] + math.cos(mh + turn / 2) * side
            parts.append(ground_number(f"paint_{tag}_{m}", px, py,
                                       f"{abs(int(round(deg)))}", terrain=terrain))
            paint.append((px, py))

    # --- máquinas, nos carreadores e em nenhum outro lugar -------------------
    def beside(path, distance, offset):
        x, y, h = path.at(distance)
        return x - math.sin(h) * offset, y + math.cos(h) * offset, h

    def standing(uri, name, x, y, yaw=0.0):
        """Qualquer coisa que repousa no chão em vez de crescer dele: acompanha a encosta, do
        jeito que uma máquina estacionada faz."""
        return include(uri, name, *terrain.pose_on(x, y, yaw))

    # O trator e a carreta, estacionados no pátio ao lado do galpão e não num carreador. Uma
    # máquina parada no pátio é uma entre serviços; uma máquina parada na estrada é uma que
    # quebrou ali, e o pátio é onde um motorista de fato deixa as duas. O galpão vai de y = 21 a
    # y = 31, então o par fica a 1,4 m da parede sul, com o nariz para leste ao longo das
    # fileiras, na clareira nivelada e longe do tanque de água em (-36, 18) e das caixas junto
    # ao portão.
    #
    # O trator simulado é este mesmo adereço -- `autonomous_tractor_description` copia a
    # carroceria dele painel por painel --, então os dois são a mesma máquina em dois estados:
    # um estacionado junto ao galpão, outro na origem com o grafo ROS ligado a ele. É uma coisa
    # justa de um mundo mostrar, desde que o dirigido seja o que está no carreador.
    # O pátio fica recuado da estrada, e quanto recuado é uma regra e não um visual: 15 m de
    # qualquer linha central, que são os 6 m de terra nua que o carreador desgasta mais espaço
    # para uma máquina passar por uma estacionada sem sair da estrada. `check_props` abaixo
    # recusa qualquer coisa mais perto. A primeira versão deste mundo tinha o tanque de água a
    # 1 m da linha central e as caixas de campo a 4,8 m, que são um tanque e duas caixas na
    # estrada que um trator sem motorista deveria ter só para si.
    tractor_x, tractor_y = yard["tractor"]
    trailer_x, trailer_y = yard["trailer"]
    parts.append(standing("model://tractor", "tractor", tractor_x, tractor_y))
    parts.append(standing("model://bin_trailer", "trailer",
                          trailer_x, trailer_y))
    props = [(tractor_x, tractor_y), (trailer_x, trailer_y)]
    standing_props = [("the tractor", tractor_x, tractor_y),
                      ("its trailer", trailer_x, trailer_y)]

    sx, sy = yard["shed"]
    parts.append(box("shed", (16.0, 10.0, 4.4), (sx, sy, terrain.z(sx) + 2.2), C_SHED))
    tx2, ty2 = yard["tank"]
    parts.append(standing("model://water_tank", "yard_tank", tx2, ty2))
    for i, k in enumerate(("bin0", "bin1", "bin2")):
        parts.append(standing("model://harvest_bin", f"bin_yard{i}", *yard[k]))
    props += [(sx, sy), (tx2, ty2)]
    standing_props += [("the shed", sx, sy), ("the water tank", tx2, ty2)]
    standing_props += [("a yard bin", *yard[k]) for k in ("bin0", "bin1", "bin2")]

    # Caixas esperando na ponta de uma fileira. Elas ficavam a 4,8 m da linha central, que é
    # terra nua do carreador: perto o bastante para carregar da estrada, e bem no caminho de
    # quem a percorre. Dezesseis metros as põem na cabeceira, no fim das fileiras, que é onde
    # um produtor de fato deixa uma caixa que não vai encher hoje.
    for i, dist in enumerate((reach(bound, 180) + 150.0, reach(bound, 180) + 300.0)):
        # Dezesseis metros da linha central está livre da estrada ao lado da qual ela fica, e
        # nada diz que está livre da próxima estrada adiante. Segue andando até estar livre de
        # todas, o que numa propriedade deste tamanho leva no máximo algumas dezenas de metros.
        main_pts = main.points(1.0)
        main_len = sum(math.dist(a, b) for a, b in zip(main_pts, main_pts[1:]))
        bx = by = bh = None
        for delta in (0, -40, 40, -80, 80, -120, 120, -160, 160, -200, 200):
            at = min(max(30.0, dist + delta), main_len - 30.0)
            for side in (-16.0, 16.0):
                qx, qy, qh = beside(main, at, side)
                if not all_roads.within(qx, qy, 15.0):
                    bx, by, bh = qx, qy, qh
                    break
            if bx is not None:
                break
        if bx is None:
            raise ValueError(f"{name}: nowhere clear to leave a field bin")
        parts.append(standing("model://harvest_bin", f"bin_field{i}", bx, by, bh))
        standing_props.append((f"a field bin", bx, by))

    check_props(name, network, standing_props)

    # --- o que cresce em tudo o que sobra ------------------------------------
    # Uma fazenda não é plantada de ponta a ponta. O que não é talhão, estrada ou mancha de mata
    # é pasto, e pasto nesta parte do país é capim com árvores em pé no meio, sozinhas ou em
    # duplas, deixadas onde estavam quando a terra foi desmatada. Preencher esse terreno não é
    # decoração: um campo aberto sem nada é o único lugar onde uma pilha de navegação nunca
    # precisa funcionar, e também o que menos se parece com as fotografias.
    #
    # Espalhado a partir de uma semente fixa e rejeitado contra tudo o que já foi posto, então o
    # arranjo é irregular mas igual em toda execução.
    open_rng = random.Random(90210)
    taken = NearGrid([(sx, sy) for sx, sy, ex, ey, _ in row_segments] +
                     [(ex, ey) for sx, sy, ex, ey, _ in row_segments] +
                     mata_pts + props + [(p[0], p[1]) for p in
                                         ((-24, 26), (-36, 18))])
    hedge_pts = []
    for sx, sy, ex, ey, _ in row_segments:
        n = max(2, int(math.hypot(ex - sx, ey - sy) / 3.0))
        hedge_pts += [(sx + (ex - sx) * i / n, sy + (ey - sy) * i / n)
                      for i in range(n + 1)]
    hedge_grid = NearGrid(hedge_pts)
    scattered: list[tuple[float, float]] = []

    def open_ground(x, y, spacing, spread):
        if hedge_grid.within(x, y, 6.0) or taken.within(x, y, 10.0):
            return False
        # `spread` é quanto a própria coisa tem de ficar afastada de uma estrada além dos 1,5 m
        # de margem que um objeto pontual precisa. Nada quer mais que isso hoje; o parâmetro
        # está aqui porque a primeira tentativa de pôr madeira em pé aqui quis, e descobrir
        # por quê é o comentário abaixo do laço.
        if any(g.within(x, y, cl - CANOPY_W / 2 + 1.5 + spread)
               for g, cl in road_grids):
            return False
        if any(in_polygon(x, y, poly) for poly in matas):
            return False
        if mata_grid.within(x, y, 5.0):
            return False
        if any(math.hypot(x - cx, y - cy) < r + 4 for cx, cy, r in clearings):
            return False
        return not any(math.hypot(x - a, y - b) < spacing for a, b in scattered)

    # O que está solto na cabeceira: madeira em pé que a fazenda nunca limpou, e uma pedra de vez
    # em quando. Havia cupinzeiros aqui, quarenta deles, e saíram -- um monte de 1 m de elipsoides
    # marrons junto a um carreador parece algo largado e não algo crescido, e neste tamanho
    # ninguém olhando o mundo saberia dizer o que devia ser.
    # Uma árvore fica onde ficava um monte: a 7,5 m da linha central do carreador, o que põe o
    # tronco 1,5 m fora da terra nua e deixa a copa inclinada um metro sobre o acostamento, do
    # jeito que uma árvore à beira de uma estrada de fazenda faz. O que ela não pode é pender
    # sobre os 7 m de superfície de rolagem, e com 2,6 m de copa a 7,5 m ela passa 1,4 m longe.
    # Os dois são postos alternadamente e não um tipo e depois o outro, e em citrus_farm isso
    # importa. Eles disputam a mesma cabeceira, e ela é pequena: citrus_farm, plantada rente aos
    # carreadores, tem lugar para vinte e duas coisas soltas, onde os mundos cujos blocos deixam
    # folga levam todas as cinquenta e oito. Quem rodasse primeiro levava tudo -- pedras
    # primeiro deixavam três árvores, árvores primeiro deixavam uma pedra --, então eles se
    # revezam, e cada um fica com a sua parte do que há: 14 árvores e 8 pedras aqui, as 40 e 18
    # completas nos outros três.
    scatter = (("forest_tree", 40, 12.0, 0.0), ("boulder", 18, 14.0, 0.0))
    placed = {uri: 0 for uri, _, _, _ in scatter}
    for n in range(18000):
        wanted = [k for k in scatter if placed[k[0]] < k[1]]
        if not wanted:
            break
        uri, count, spacing, spread = wanted[n % len(wanted)]
        x = open_rng.uniform(-244.0, 244.0)
        y = open_rng.uniform(-236.0, 236.0)
        if not open_ground(x, y, spacing, spread):
            continue
        scattered.append((x, y))
        wood.append((uri, x, y, terrain.z(x), open_rng.uniform(0, 2 * math.pi)))
        placed[uri] += 1

    quadrant: dict[str, list] = {"ne": [], "nw": [], "se": [], "sw": []}
    for item in wood:
        _, wx, wy = item[0], item[1], item[2]
        quadrant[("n" if wy >= 0 else "s") + ("e" if wx >= 0 else "w")].append(item)
    for tag, items in quadrant.items():
        if items:
            parts.append(scenery(f"scenery_{tag}", items))

    the_map = plan_map([p.points(3.0) for _, p, _, _, _ in roads], matas,
                       row_segments, props, paint,
                       wood=[(w[1], w[2]) for w in wood], cell=8.0)
    comment = FARM_COMMENT.format(map=the_map, opening=opening.strip("\n"),
                                  loops_note=loops_note)
    meshes = {f"{name}.obj": obj_mesh(road_groups,
                                      f"The carreadores of {name}.",
                                      f"{name}.mtl"),
              f"{name}.mtl": mtl_text(road_groups,
                                      f"The carreadores of {name}.")}
    return name, emit(name, comment, parts, step=step), meshes


def dogleg(corners, radius):
    """Uma estrada por uma lista de cantos, com um arco de um raio em cada.

    Declarativo onde `block_loop` era procedural: diga onde a estrada vira e as retas entre os
    arcos são calculadas para cada arco ser tangente às duas. Um canto come raio * tan(metade da
    curva) da reta de cada lado, que é toda a aritmética e exatamente a parte tediosa de manter
    certa à mão quando um canto se move.
    """
    def bearing(a, b):
        return math.atan2(b[1] - a[1], b[0] - a[0])

    turns, setbacks = [], [0.0] * len(corners)
    for i in range(1, len(corners) - 1):
        t = bearing(corners[i], corners[i + 1]) - bearing(corners[i - 1], corners[i])
        t = (t + math.pi) % (2 * math.pi) - math.pi
        turns.append(t)
        setbacks[i] = radius * abs(math.tan(t / 2))

    p = Path(corners[0][0], corners[0][1], math.degrees(bearing(corners[0], corners[1])))
    for i in range(len(corners) - 1):
        span = math.hypot(corners[i + 1][0] - corners[i][0],
                          corners[i + 1][1] - corners[i][1])
        run = span - setbacks[i] - setbacks[i + 1]
        if run < 1.0:
            raise ValueError(f"corners {i} and {i + 1} are too close for radius "
                             f"{radius}: {run:.1f} m of straight left between them")
        p.straight(run)
        if i < len(turns):
            p.turn(math.degrees(turns[i]), radius)
    return p


def polygon_perimeter(angle, radius, inradius):
    """Um limite de propriedade que vira um ângulo e se fecha sobre si mesmo.

    360 graus de giro divididos pelo ângulo dão o número de lados: quatro com 90, seis com 60,
    doze com 30. Dimensionado pelo raio inscrito, que é o número útil aqui porque é a distância
    do cruzamento ao trecho de limite mais próximo, e portanto quanto espaço os talhões têm.
    """
    n = round(360 / angle)
    side = 2 * inradius * math.tan(math.pi / n)
    leg = side - 2 * radius * math.tan(math.radians(angle) / 2)
    p = Path(-inradius, 0.0, 90).straight(leg / 2)
    for i in range(n):
        # a última perna corre dois metros além do início, então o laço visivelmente fecha e a
        # última curva é limitada por uma reta como as outras
        p.turn(-angle, radius).straight(leg if i < n - 1 else leg / 2 + 2)
    return p


def mixed_perimeter(turns, free, radius, h0=90.0, radii=None):
    """Um limite que vira um ângulo diferente em cada canto e ainda se fecha.

    `polygon_perimeter` desenha um polígono regular: todo canto vira o mesmo ângulo, então todo
    canto pinta o mesmo número, e uma propriedade com quatro ângulos retos também encontra toda
    estrada que a cruza num ângulo reto. As duas coisas parecem erradas no mundo misto, onde um
    90 pintado deve significar algo: se os quatro cantos dizem 90 e todo carreador encontra o
    limite num ângulo reto sem rótulo, os números deixam de ser informação.

    Então este toma as curvas que deve fazer -- dois 90, dois 60 e dois 30 aqui -- e os
    primeiros n-2 comprimentos de lado, e resolve os dois últimos para o anel se fechar. Girar
    360 graus no total é a única coisa que não é livre: o fechamento são duas equações, a soma
    de L*cos(rumo) e a soma de L*sin(rumo) ambas zero, que é exatamente o bastante para fixar
    dois comprimentos.

    `radii` dá a cada canto o seu raio, e 0 ali é um canto sem arco nenhum. Um limite de campo é
    cortado em esquadro muito mais vezes que abaulado -- as fotografias aéreas são todas de
    cantos vivos --, então o mundo misto toma dois de cada nos seus quatro ângulos retos, que é
    também como ele ganha um 90 vivo e um arredondado sem pôr nenhum dos dois dentro da
    propriedade.
    """
    n = len(turns)
    h, heads = math.radians(h0), []
    for t in turns:
        heads.append(h)
        h += math.radians(t)
    sx = sum(free[i] * math.cos(heads[i]) for i in range(n - 2))
    sy = sum(free[i] * math.sin(heads[i]) for i in range(n - 2))
    a, b = math.cos(heads[-2]), math.cos(heads[-1])
    c, d = math.sin(heads[-2]), math.sin(heads[-1])
    det = a * d - b * c
    if abs(det) < 1e-9:
        raise ValueError("the last two sides are parallel: the ring cannot close")
    sides = list(free) + [(-sx * d + sy * b) / det, (-a * sy + c * sx) / det]
    if min(sides) < 60.0:
        raise ValueError(f"a side came out at {min(sides):.0f} m")

    # Os cantos, depois o centroide, para a propriedade ficar em torno da origem e não onde o
    # primeiro lado por acaso começou.
    pts, x, y = [(0.0, 0.0)], 0.0, 0.0
    for leg, hh in zip(sides, heads):
        x += leg * math.cos(hh)
        y += leg * math.sin(hh)
        pts.append((x, y))
    ox = -sum(q[0] for q in pts[:-1]) / n
    oy = -sum(q[1] for q in pts[:-1]) / n

    rr = list(radii) if radii is not None else [radius] * n
    back = [r * math.tan(abs(math.radians(t)) / 2) for r, t in zip(rr, turns)]
    p = Path(pts[0][0] + ox + sides[0] / 2 * math.cos(heads[0]),
             pts[0][1] + oy + sides[0] / 2 * math.sin(heads[0]), h0)
    p.straight(sides[0] / 2 - back[0]).turn(turns[0], rr[0])
    for i in range(1, n):
        p.straight(sides[i] - back[i - 1] - back[i]).turn(turns[i], rr[i])
    # dois metros além do início, então o laço visivelmente fecha
    p.straight(sides[0] / 2 - back[-1] + 2)
    return p


def reach(points, heading, start=60.0, stop=520.0):
    """Até onde uma estrada pode correr a partir da origem antes de encontrar o limite."""
    c, s = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    grid = NearGrid(points)
    d = start
    while d < stop:
        x, y = c * d, s * d
        if grid.within(x, y, 3.0):
            return d
        d += 0.5
    return stop


THROUGH_BLOCKS = [
    ("a", -148.0, 148.0, 56.0, 56.0, math.radians(0)),
    ("b", -26.0, 148.0, 62.0, 56.0, math.radians(90)),
    ("c", 124.0, 168.0, 80.0, 36.0, math.radians(0)),
    ("d", 148.0, 64.0, 56.0, 62.0, math.radians(90)),
    ("e", -148.0, 45.0, 56.0, 44.0, math.radians(0)),
    ("f", -26.0, 45.0, 62.0, 44.0, math.radians(45)),
    ("g", -148.0, -64.0, 56.0, 62.0, math.radians(0)),
    ("h", 30.0, -92.0, 62.0, 42.0, math.radians(90)),
    ("i", -118.0, -170.0, 82.0, 34.0, math.radians(0)),
    ("j", 120.0, -128.0, 30.0, 76.0, math.radians(0)),
    ("k", 178.0, -128.0, 26.0, 76.0, math.radians(90)),
    ("l", -60.0, -92.0, 42.0, 42.0, math.radians(0)),
    ("m", 0.0, -170.0, 42.0, 34.0, math.radians(90)),
    ("n", -20.0, -25.0, 62.0, 22.0, math.radians(0)),
    ("z", 0.0, 0.0, 210.0, 210.0, math.radians(0)),
]


# Os talhões do mundo do cruzamento, que não são uma lista de retângulos que alguém escolheu:
# são as quatro células que as estradas deixam. Um bloco de citros é delimitado por carreadores
# em todos os lados, porque é assim que um produtor leva uma máquina até ele e tira a fruta, e
# um grupo de árvores sem estrada num dos lados é um grupo que ninguém colhe. Então os blocos
# aqui são os quadrantes, e a estrada do limite e os dois carreadores que se cruzam são os
# quatro lados deles.
#
# As fileiras correm de leste a oeste em dois deles e de norte a sul nos outros dois. Blocos
# adjacentes com direções de fileira diferentes é o que as fotografias aéreas mostram, e é
# também a única coisa que torna visível do ar o limite de um bloco depois que as copas se
# fecham.
CROSS_BLOCKS = [
    ("ne", 141.0, 141.0, 133.0, 133.0, math.radians(0)),
    ("nw", -141.0, 141.0, 133.0, 133.0, math.radians(90)),
    ("se", 141.0, -141.0, 133.0, 133.0, math.radians(90)),
    ("sw", -141.0, -141.0, 133.0, 133.0, math.radians(0)),
]


def through_network(angle, radius, inradius, spec):
    """Estradas que cruzam uma propriedade, cada uma virando um ângulo e nada mais.

    `spec` é (tag, rumo de origem, direção, movimentos): onde no limite a estrada começa, para
    que lado parte, e depois os movimentos como pares (reta, sinal da curva). A reta final é
    calculada, então uma estrada sempre termina no limite e não no meio de um campo.
    """
    per = polygon_perimeter(angle, radius, inradius)
    bound = per.points(2.0)
    net = [("perimeter", per)]
    for tag, bearing, heading, moves in spec:
        d = reach(bound, bearing)
        p = Path(d * math.cos(math.radians(bearing)),
                 d * math.sin(math.radians(bearing)), heading)
        for run, sign in moves:
            p.straight(run)
            if sign:
                p.turn(sign * angle, radius)
        net.append((tag, p.close_to_boundary(bound)))
    return net


def mixed_network(radius, per, spec):
    """O mesmo que `through_network`, com os ângulos de curva escritos por extenso.

    Para a fazenda cujas curvas são deliberadamente de tamanhos diferentes: cada movimento é uma
    reta e depois uma curva com sinal em graus, ou zero para nenhuma curva. As irmãs dela tomam
    um ângulo e o impõem em toda curva; esta recebe cada ângulo porque essa mistura é o ponto do
    mundo. Recebe também o limite, porque esse limite não é um polígono regular.

    Um sexto item na tupla de uma estrada é um valor de y em que terminar em vez do limite: essa
    estrada termina num T contra o carreador que correr ao longo dessa linha, que é como uma
    estrada dentro de um bloco se junta à rede da propriedade sem correr até a cerca.
    """
    bound = per.points(2.0)
    net = [("perimeter", per)]
    for entry in spec:
        tag, bearing, heading, moves = entry[:4]
        close_on = entry[4] if len(entry) > 4 else None
        d = reach(bound, bearing)
        p = Path(d * math.cos(math.radians(bearing)),
                 d * math.sin(math.radians(bearing)), heading)
        # (reta, curva) abaula a curva no raio do mundo; um terceiro número o sobrescreve, e 0
        # ali é uma esquina viva.
        for move in moves:
            run, turn = move[0], move[1]
            r = move[2] if len(move) > 2 else radius
            p.straight(run)
            if turn:
                p.turn(turn, r)
        net.append((tag, p.close_to_y(close_on) if close_on is not None
                    else p.close_to_boundary(bound)))
    return net


def _through_farm(angle, radius, spec, opening_note, loops_note, step=0.003):
    return _build_farm(
        f"citrus_farm_{angle}", through_network(angle, radius, 205, spec),
        THROUGH_BLOCKS,
        f"""
  Citrus farm, {angle} degree bends and no others. The same ground, the same
  planting and the same mata as citrus_farm_90.sdf, on a road network of its
  own. Generated by scripts/generate_agriculture_worlds.py, edit there.

  Carreadores cross the property and a boundary encloses it. Every one of them
  runs from one edge to another, and they cross each other on the way; the
  talhoes are whatever the crossings leave over. {opening_note}
        """,
        loops_note, step=step)


def build_citrus_farm_60() -> tuple[str, str, dict[str, str]]:
    """O 60: três carreadores, uma curva de ida e uma de volta em cada.

    Um limite que se fecha sobre si mesmo girou 360 graus, então com 60 é um hexágono, e essa é
    a única coisa neste mundo que não é uma escolha.
    """
    return _through_farm(
        60, 26,
        [("main", 180, 0, [(250, -1), (80, 1)]),
         ("cross", 95, -90, [(110, 1), (60, -1)]),
         # Da borda sudeste à noroeste, atravessando tudo. Ela começava no ponto de baixo do
         # hexágono e partia para leste, onde a propriedade tem 35 m de largura: 180 dos seus
         # 270 m corriam fora do limite, e uma estrada que cruza o próprio limite duas vezes
         # desenha um oito na planta. `check_network` recusa isso agora.
         ("south", 300, 120, [(120, 1), (60, -1)])],
        "A 60 is enough of a turn to send a road somewhere else, so each of "
        "these takes one out and one back, and three of them divide the "
        "property between them.",
        "Every bend turns 60 degrees: two on each of the three carreadores and "
        "six around the boundary, because a boundary that closes on itself "
        "turns 360.")


def build_citrus_farm_30() -> tuple[str, str, dict[str, str]]:
    return _through_farm(
        30, 30,
        [("main", 180, 0, [(235, 1), (18, 1), (18, 1), (70, 0)]),
         ("cross", 90, -90, [(200, -1), (18, -1), (18, -1), (70, 0)])],
        "A 30 barely changes where a road is pointing, so turning a corner "
        "takes three of them in a row, and two roads is all the property will "
        "hold: any more and they run alongside each other instead of crossing. "
        "That is the angle showing through the layout rather than a compromise "
        "in it.",
        "Every bend turns 30 degrees: three to a corner, three corners in all, "
        "and twelve around the boundary.")


SQUARE = 560.0                  # a propriedade, de lado a lado

# O cinturão de mata, em volta de todo o lado de fora do quadrado. A mata mantém o lado de fora
# do mundo onde sempre esteve -- nada aqui passa de 332 m da origem, que é aonde o cinturão
# chegava quando a propriedade era um quadrado de 400 m -- e tudo o que ela cede por dentro vai
# para os talhões. A faixa tem 18 m de largura agora onde começou com 40, 24 000 m2 de copa
# contra os 122 000 da primeira versão, e a cerca foi de 200 m para 280 no mesmo terreno.
#
# Um cinturão é cenário por onde ninguém passa, um talhão é para o que a máquina está ali, e
# quando os dois querem os mesmos metros o talhão leva. Comprido ao longo do lado e estreito
# através dele, para a mata ainda fechar o horizonte sem comer o campo.
CROSS_WOODS = [
    *[blob(x, side * 306, 40, 18, seed=11 + i)
      for i, (x, side) in enumerate(
          [(x, s) for s in (1, -1) for x in (-140, 140)])],
    *[blob(side * 306, y, 18, 40, seed=31 + i)
      for i, (y, side) in enumerate(
          [(y, s) for s in (1, -1) for y in (-140, 140)])],
    *[blob(sx * 310, sy * 310, 22, 22, seed=51 + i)
      for i, (sx, sy) in enumerate([(1, 1), (-1, 1), (1, -1), (-1, -1)])],
]


def build_citrus_farm() -> tuple[str, str, dict[str, str]]:
    """A propriedade de que este workspace realmente trata.

    Um quadrado de 560 m, dois carreadores que se cruzam em esquadro na origem, quatro talhões
    com uma copa cada, um cinturão de mata fora da cerca e quatro estradas oblíquas cortadas
    pelos blocos: uma curva de 60 graus em cada talhão de cima e uma de 30 em cada de baixo.

    Sessenta graus nos três pontos de cada estrada da metade de cima, e trinta nos três de cada
    estrada da metade de baixo, que é para o que servem os formatos. Ela sai da estrada do
    limite no alto do seu bloco a 60, não em esquadro com ela; vira 60 no meio do bloco; e
    encontra o carreador principal embaixo a 60. Um V raso, descendo para um lado e para o
    outro, e uma máquina que o percorre toma o mesmo ângulo três vezes em três tipos de junção:
    um T saindo do limite, uma curva em campo aberto, um T sobre o principal.

    Sair do limite norte em esquadro, que é onde isto começou, fazia a primeira das três um
    ângulo reto numa estrada cujo assunto inteiro é 60 graus. Entrar a 60 corrige isso, e a
    aritmética então corrige todo o resto: 60 a partir de uma estrada horizontal, vira 60, e a
    estrada está a -120, que encontra o principal horizontal a 60 de novo.

    A curva começa na parte mais central do talhão, que é a outra metade do ponto: uma máquina
    que entra no bloco pelo norte encontra a curva com o bloco todo em volta, e não contra uma
    cerca, e as fileiras dos dois lados são cortadas por ela.

    Elas são espelhadas, então o mundo tem a mesma curva duas vezes em lados opostos, em terreno
    da mesma inclinação. As quatro são abauladas no raio de 24 m do mundo; qualquer uma vira
    esquina viva pondo um 0 como terceiro número do seu movimento.

    As duas estradas de baixo fazem o mesmo serviço a 30 graus e não conseguem fazê-lo na mesma
    forma, e a aritmética diz por quê. Uma perna a 30 graus da horizontal encontra uma estrada
    vertical a 60; uma perna a 60 encontra uma estrada horizontal a 60. Com uma curva de 30 as
    duas pernas ficam em -30 e -60: a primeira tem de terminar num carreador horizontal, o que
    pede um bloco alto, e a segunda num vertical, o que pede um bloco largo. Nenhum retângulo é
    os dois. Sessenta graus escapa disso porque as duas pernas, -60 e -120, estão ambas a 60 da
    horizontal, então as duas podem terminar no principal e no limite.

    Então as estradas de baixo tomam os 30 graus nas duas junções -- saindo do principal a 30,
    chegando ao limite lateral a 30 -- e abrem mão do centro exato: a curva cai a 72 m do meio
    de um bloco de 266 m, no terço central. A alternativa, uma curva no centro exato com uma
    junção a 60, troca o ângulo de que este mundo trata por uma posição.
    """
    return _build_farm(
        "citrus_farm",
        mixed_network(24, mixed_perimeter([-90, -90, -90, -90],
                                          [SQUARE, SQUARE], 24), [
            ("main", 180, 0, [(0, 0)]),
            ("cross", 90, -90, [(0, 0)]),
            ("northeast", 77.76, -60, [(160.5, -60), (0, 0)], 0.0),
            ("northwest", 102.24, -120, [(160.5, 60), (0, 0)], 0.0),
            ("southeast", 317.60, 120, [(202.9, 30), (0, 0)], 0.0),
            ("southwest", 222.40, 60, [(202.9, -30), (0, 0)], 0.0),
        ]),
        CROSS_BLOCKS,
        """
  Citrus farm: the square and its crossroads, with a 60 degree bend in each of
  the two upper talhoes. Generated by scripts/generate_agriculture_worlds.py,
  edit there.

  The frame is a 560 m square with four 90 degree corners, two carreadores
  crossing square at the origin, four talhoes one crown apiece, and a belt of
  woods outside the fence.

  Four roads are added here, two above the main and two below it. The upper
  pair is 60 degrees at all three of its points: off the boundary road at 60
  rather than square to it, a 60 degree bend in the middle of the block, and
  onto the main at 60. A shallow V, and the same angle on a T off the
  boundary, on a bend in open ground, and on a T onto the main.

  The lower pair does the same at 30 degrees, off the main and onto the side
  boundary, and its bend is 72 m from the middle of its block rather than at
  it. That is geometry and not laziness: a leg at 30 degrees to the horizontal
  meets a vertical road at 60, so of the two legs of a 30 degree bend, one
  wants a tall block to finish on a horizontal road and the other a wide one
  to finish on a vertical road, and no rectangle is both. Sixty degrees is the
  angle that escapes it.

  Each pair mirrors itself. All four are painted with their angle.
        """,
        "Eight bends: the boundary's four 90s, a 60 in each upper talhao and "
        "a 30 in each lower one, each pair turning opposite ways. Those four "
        "roads also meet the carreadores they start and finish on at their "
        "own angle, so a 60 road is 60 at every junction it has and a 30 road "
        "is 30 at every one of its own. The crossroads is still cut square "
        "and still carries no number.",
        step=0.003, extent=460.0, wood_trees=12, woods=CROSS_WOODS)


def build_citrus_farm_90() -> tuple[str, str, dict[str, str]]:
    """O mundo de 90 graus, disposto como estradas que cruzam a propriedade.

    A primeira versão cercava cada quarto da fazenda com um laço, e o resultado era um
    retângulo dentro de um retângulo dentro de um retângulo: toda rota dava a volta em alguma
    coisa, e nenhuma ia a lugar nenhum. Uma rede de estradas de fazenda não é feita de anéis. É
    feita de estradas que correm de uma borda da propriedade a outra e se cruzam no caminho, e
    os talhões são o que os cruzamentos deixam, que é o que as fotografias aéreas mostram e o
    que isto agora faz.

    Quatro rotas passantes e um limite, dez curvas, todas de 90 graus.
    """
    R, EDGE = 24, 205
    per = polygon_perimeter(90, R, EDGE)
    network = [
        # da borda oeste à leste, pela origem, subindo para o norte no caminho
        ("main", dogleg([(-EDGE, 0), (90, 0), (90, 130), (EDGE, 130)], R)),
        # da borda norte à sul, indo para leste sob o meio da fazenda
        ("cross", dogleg([(-90, EDGE), (-90, -50), (150, -50), (150, -EDGE)], R)),
        # da borda oeste à borda sul, ao longo da base da propriedade
        ("south", dogleg([(-EDGE, -135), (40, -135), (40, -EDGE)], R)),
        # da borda norte à borda oeste, por cima
        ("north", dogleg([(40, EDGE), (40, 90), (-EDGE, 90)], R)),
        ("perimeter", per),
    ]
    blocks = THROUGH_BLOCKS
    return _build_farm(
        "citrus_farm_90", network, blocks,
        """
  Citrus farm, 90 degree bends and no others. The same ground, the same
  planting and the same mata as its 60 and 30 degree siblings, on a road
  network of its own. Generated by scripts/generate_agriculture_worlds.py,
  edit there.

  Four carreadores cross the property and a boundary encloses it. Every one of
  them runs from one edge to another or from one road to another, and they
  cross each other on the way; the talhoes are whatever the crossings leave
  over. An earlier cut of this ringed each quarter of the farm with a loop and
  the result was a rectangle inside a rectangle inside a rectangle, where every
  route went round something and none of them went anywhere.
        """,
        "Ten bends, all of them 90 degrees: two on the main carreador, two on "
        "the cross, one on each of the other two, and four around the boundary.",
        # 3 ms, e as outras duas fazendas tomam o mesmo. Estradas que cruzam uma propriedade
        # deixam muito mais dela plantável que estradas que cercam partes dela -- 5421 árvores
        # aqui, a maior contagem das três --, e o passo é o que compensa isso. Conferido e não
        # presumido, no robô que tem de conviver com ele: a 3 ms o rover se assenta imóvel na
        # origem e anda 8,5 m pelo carreador sem o yaw ou a posição lateral saírem de zero.
        step=0.003)


# ------------------------------------------------------------------ principal --

# ------------------------------------------------------ colisões das plantas --

PLANT_MODELS = re.compile(r'<model name="(?:hedges|scenery)_[^"]*">.*?</model>', re.S)
COLLISION_BLOCK = re.compile(r"[ \t]*<collision[^>]*>.*?</collision>\n?", re.S)


def without_plant_collisions(text: str) -> str:
    """Tira as formas de colisão de todo modelo de sebe e de mata.

    Um pomar simulado para percepção não precisa de o motor de física saber onde as árvores
    estão: a câmera as vê porque são visuais, e o solver gastava o tempo em 480 cilindros de
    tronco em que nada nunca tocava. Medido em citrus_farm_60 com o trator criado, em 20 s de
    tempo simulado: 32,8 s de relógio com elas, 29,0 s sem.

    O que NÃO é tocado, e a distinção é o ponto todo: o terreno (as lajes), as estruturas que
    decidem aonde uma máquina pode ir (o galpão) e os adereços incluídos de models/ mantêm toda
    colisão que tinham. Atravessar uma árvore é uma troca deliberada; cair pelo chão tornaria
    os três mundos sem sentido.

    Passe `--collisions full` para ter de volta os mundos estritos.
    """
    return PLANT_MODELS.sub(lambda m: COLLISION_BLOCK.sub("", m.group(0)), text)


BUILDERS = (build_citrus_farm_90, build_citrus_farm_60, build_citrus_farm_30,
            build_citrus_farm)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=FilePath, default=DEFAULT_OUT,
                        help="directory to write the worlds into")
    parser.add_argument("--models", type=FilePath, default=HERE.parent / "models",
                        help="model library the baked road meshes are written into")
    parser.add_argument("--collisions", choices=("ground", "full"), default="ground",
                        help=("ground (default): hedgerows and mata are visuals only, so the "
                              "solver stops paying for trunks nothing touches. full: every "
                              "plant collides, as in the first cut of these worlds."))
    args = parser.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    # As malhas de estrada são escritas do zero a cada execução, inclusive as obsoletas: este
    # diretório não guarda nada que um humano tenha editado, e um mundo renomeado deixaria os
    # carreadores dele para trás para sempre.
    meshes = args.models / "carreador" / "meshes"
    meshes.mkdir(parents=True, exist_ok=True)
    for stale in sorted(meshes.iterdir()):
        if stale.suffix in (".obj", ".mtl"):
            stale.unlink()
    for builder in BUILDERS:
        name, text, baked = builder()
        if args.collisions == "ground":
            text = without_plant_collisions(text)
        path = args.out / f"{name}.sdf"
        path.write_text(text)
        for filename, body in baked.items():
            (meshes / filename).write_text(body)
        tris = sum(body.count("\nf ") for body in baked.values())
        print(f"{path}  ({text.count('<model name=') + text.count('<include>')} models, "
              f"{text.count('<collision')} collisions, {tris} road triangles)")


if __name__ == "__main__":
    main()
