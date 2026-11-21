#!/usr/bin/env python3
"""Gera as malhas de copa de citros que `models/citrus_tree` e os talhões usam.

    python3 agri_gz_worlds/scripts/generate_citrus_meshes.py            # reescreve no lugar
    python3 agri_gz_worlds/scripts/generate_citrus_meshes.py --out /tmp  # para ver antes

Seis variantes de uma árvore, em OBJ, mais uma nuvem de frutos para cada uma. Os arquivos
são versionados; o build nunca roda este script e o Gazebo só lê o OBJ, do mesmo jeito que só
lê os mundos gerados.

Por que uma malha, se todas as outras plantas desta biblioteca são primitivas.

Um citros adulto não é um elipsoide e não parece um. Nas fotografias de docs/citrus/ a copa é
uma massa irregular de tufos de folhas, e o que a torna legível como árvore é a luz: os tufos
voltados para o sol são claros, os vãos entre eles são quase pretos e a silhueta contra a rua
é recortada. Um elipsoide não tem nada disso. É uma superfície lisa, então recebe um único
gradiente suave do sol e parece um balão verde, e duzentos deles numa fileira parecem papel de
parede. Nenhum ajuste de material resolve, porque o problema é a geometria.

Uma superfície irregular com sombreamento chapado (`flat shading`) tem o efeito todo de graça,
com o sol que já está no mundo: cada face recebe a própria luz, então o agrupamento é iluminado
e não pintado. Por isso as normais aqui são por face e não médias por vértice, o oposto do que
um exportador de malhas faz por padrão.

E não custa quase nada. As medições dos próprios mundos de fazenda dizem que o Gazebo cobra por
*entidade*, não por triângulo: 300 objetos como modelos separados custam 0,8 ms por passo,
como elos de um modelo 0,5, como formas simples num único elo 0,3. Uma malha é uma só forma,
tenha ela 8 triângulos ou 8000, então trocar um elipsoide por uma copa de 1296 triângulos deixa
o passo onde estava e passa o trabalho para a GPU, que está ociosa. O fruto é o único custo
real, porque é um segundo visual por árvore.

O tronco faz parte da malha da copa, e não é uma segunda forma, pelo mesmo motivo: uma árvore
com saia mal mostra o tronco, e um cilindro separado dobraria o número de entidades para
desenhar 20 cm de casca.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "models/citrus_tree/meshes"

VARIANTS = "abcdef"

# Uma sequência (`run`) são oito árvores a 3 m, assadas numa única malha, e ela existe por um
# motivo: cada `<visual>` num mundo é uma entidade, e o servidor percorre todas a cada passo,
# quer algo se mova ou seja renderizado, quer não. Medido em citrus_farm_60, que tem 7255
# delas: o mundo inteiro roda a 312 passos de física por segundo, e a 1428 sem os visuais.
# Só as árvores são 4966 dessa conta.
#
# Oito por malha leva a contagem de árvores de 4966 visuais para cerca de 620.
#
# Por que não assar uma fileira inteira, o que levaria a 339: um OBJ de uma árvore tem 87 KB,
# então uma fileira tem em média 3,5 MB e os três mundos de fazenda carregariam 1,3 GB de
# geometria única. Uma biblioteca de seis sequências tem 4 MB e chega na maior parte do caminho.
# Oito copas também dão 24 m, o comprimento em que um trecho padrão de fileira sempre foi cortado.
# Três comprimentos, porque uma fileira não é múltipla de nada. Aqui as fileiras são cortadas
# por estradas e pelas dobras do terreno, então saem com todos os tamanhos de três a quarenta
# árvores, e com só as de oito as sobras eram um terço de todas as árvores da propriedade.
# Colocar 8, depois 4, depois 2, depois avulsas deixa no máximo uma árvore de cada fileira
# sem assar.
RUNS = "abcdef"
RUN_SIZES = (8, 4, 2)
RUN_SPACING = 3.0

# Estes são os raios da esfera antes de os lóbulos serem empurrados para fora dela, então a
# copa que resulta é maior que o dobro de um deles: medido no OBJ, 3,95 m de largura e 2,83 m
# até o topo. Plantada a 3 m numa fileira a 6,5 m da próxima, as copas ainda se sobrepõem ao
# longo da fileira e deixam 2,55 m de rua através dela: uma sebe, e não uma linha de obstáculos
# separados.
#
# Eram 14% maiores (1,75/1,90/1,42 num centro de 1,53, medindo 4,60 m de largura e 3,29 m de
# altura) e duas coisas estavam erradas nisso. O bloco parecia mais pesado do que um bloco de
# citros parece, e 4,60 m de copa numa carreira de 6,5 m deixam 1,90 m de rua, onde um trator
# de 2,15 m não passa de jeito nenhum: a conta em que tudo o mais foi escrito supunha uma copa
# de 3,5 m e ninguém tinha medido a malha. Com 3,95 m a rua tem 2,55 m e a máquina fica com
# 20 cm de cada lado.
#
# Encolher a copa em vez de abrir a carreira é a metade deliberada: o espaçamento é agronomia
# e não cabe a mim inventar; a copa é o que uma árvore de determinada idade acaba sendo.
CANOPY_RX = 1.50      # ao longo da fileira
CANOPY_RY = 1.63      # através da fileira
CANOPY_RZ = 1.22
CANOPY_Z = 1.32       # altura do centro, para o topo ficar em 2,54 m
SKIRT = 0.78          # quanto a parte de baixo é achatada
TRUNK_R = 0.115
FRUIT_N = 60
RUN_FRUIT = 18        # por árvore dentro de uma sequência assada; ver `fruit()`
FRUIT_R = 0.038

# O nível 3 tem 1296 triângulos, o 2 tem 336 e o 4 tem 5136. Os três são uma entidade e custam
# o mesmo pela medida que importa aqui, e o do meio ainda ganha, que é a parte que vale
# registrar: com 336 as faces têm 22 cm numa copa de 3,3 m e parecem uma gema lapidada, e com
# 5136 têm 6 cm, pequenas o bastante para o olho tirar a média e devolvê-las à massa lisa que
# este arquivo inteiro existe para evitar. Doze centímetros é mais ou menos o tamanho de um
# tufo de folhas de citros, e é o tamanho em que a luz deixa de ser um gradiente e passa a ser
# folhagem. Renderizei os três e olhei.
#
# O fruto precisa da própria subdivisão por um motivo mais simples: um icosaedro puro é
# visivelmente um dado de vinte faces ao alcance da mão.
CANOPY_SUBDIV = 3
FRUIT_SUBDIV = 1


# ----------------------------------------------------------------- esfera --

def icosphere(subdiv: int) -> tuple[list[tuple], list[tuple]]:
    """Esfera unitária como um icosaedro subdividido `subdiv` vezes.

    Uma icosfera e não uma esfera UV: os triângulos têm todos o mesmo tamanho, então os
    lóbulos deformam por igual e não há polo onde vinte lascas se encontram.
    """
    t = (1 + math.sqrt(5)) / 2
    verts = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
             (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
             (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]

    def norm(v):
        d = math.sqrt(sum(c * c for c in v))
        return (v[0] / d, v[1] / d, v[2] / d)

    verts = [norm(v) for v in verts]

    for _ in range(subdiv):
        mid: dict[tuple[int, int], int] = {}

        def midpoint(a, b):
            key = (min(a, b), max(a, b))
            if key not in mid:
                va, vb = verts[a], verts[b]
                verts.append(norm(((va[0] + vb[0]) / 2, (va[1] + vb[1]) / 2,
                                   (va[2] + vb[2]) / 2)))
                mid[key] = len(verts) - 1
            return mid[key]

        faces = [f for a, b, c in faces
                 for f in ((a, midpoint(a, b), midpoint(a, c)),
                           (b, midpoint(b, c), midpoint(b, a)),
                           (c, midpoint(c, a), midpoint(c, b)),
                           (midpoint(a, b), midpoint(b, c), midpoint(c, a)))]
    return verts, faces


# ------------------------------------------------------------------- copa --

def canopy(seed: int) -> tuple[list[tuple], list[tuple]]:
    """Uma copa: uma esfera empurrada em tufos de folhagem e com saia.

    Duas escalas de deformação, porque uma copa de citros tem duas. De seis a nove lóbulos
    largos são os galhos principais, cada um com a própria massa de folhas de cerca de um metro;
    por cima disso, cada vértice ganha alguns centímetros de ruído, o que impede os próprios
    lóbulos de parecerem bolhas lisas. O ruído é aplicado nos vértices compartilhados antes de
    as faces serem separadas, então faces vizinhas continuam se encontrando: um buraco numa copa
    aparece como um ponto preto visto da rua e é muito difícil de achar depois.
    """
    rng = random.Random(seed)
    verts, faces = icosphere(CANOPY_SUBDIV)

    lobes = []
    for _ in range(rng.randint(6, 9)):
        # Inclinados para cima: a massa de um citros com saia fica nos dois terços de cima, e
        # lóbulos pendurados embaixo da árvore preencheriam o vão por onde o robô realmente
        # passa.
        z = rng.uniform(-0.35, 0.95)
        a = rng.uniform(0, 2 * math.pi)
        r = math.sqrt(max(0.0, 1 - z * z))
        lobes.append(((r * math.cos(a), r * math.sin(a), z),
                      rng.uniform(0.10, 0.26), rng.uniform(0.45, 0.85)))

    out = []
    for v in verts:
        r = 1.0
        for c, amp, width in lobes:
            dot = max(-1.0, min(1.0, v[0] * c[0] + v[1] * c[1] + v[2] * c[2]))
            r += amp * math.exp(-(math.acos(dot) / width) ** 2)
        r += rng.uniform(-0.055, 0.055)
        x, y, z = v[0] * r * CANOPY_RX, v[1] * r * CANOPY_RY, v[2] * r * CANOPY_RZ
        if z < 0:
            # A parte de baixo de uma sebe é cortada reta pelo cortador e pelos colhedores que
            # trabalham embaixo dela, e não arredondada como a de cima.
            z *= SKIRT
        out.append((x, y, z + CANOPY_Z))
    return out, faces


def plain_canopy(seed: int) -> tuple[list[tuple], list[tuple]]:
    """A mesma copa sem os lóbulos: um elipsoide liso, com saia.

    Esta é a copa simples -- a forma com que estes mundos foram plantados no início -- e existe
    como malha por um único motivo: uma malha pode ser assada oito por sequência e uma
    primitiva não. Como `<ellipsoid>` do SDF ela custa um visual por árvore; um talhão de nove
    hectares delas chegou a 2986 visuais, contra os 461 que o mesmo bloco custa em sequências
    assadas. A forma é idêntica nos dois casos, então só se abre mão do número de entidades.

    Ainda com alguns por cento próprios em cada eixo, para a fileira não ser uma bola repetida,
    e com saia embaixo como a irregular.
    """
    rng = random.Random(seed)
    verts, faces = icosphere(CANOPY_SUBDIV)
    sx = 1 + rng.uniform(-0.04, 0.04)
    sy = 1 + rng.uniform(-0.04, 0.04)
    sz = 1 + rng.uniform(-0.05, 0.05)
    out = []
    for v in verts:
        x, y, z = (v[0] * CANOPY_RX * sx, v[1] * CANOPY_RY * sy,
                   v[2] * CANOPY_RZ * sz)
        if z < 0:
            z *= SKIRT
        out.append((x, y, z + CANOPY_Z))
    return out, faces


def trunk(bottom: float) -> tuple[list[tuple], list[tuple]]:
    """Os 20 e poucos cm de tronco que uma árvore com saia mostra, como um toco de oito lados."""
    verts, faces = [], []
    n = 8
    for i in range(n):
        a = 2 * math.pi * i / n
        # O tronco de um citros se alarga onde encontra o chão; 35% a mais na base é o que as
        # fotografias mostram, e é o que impede o toco de parecer um pedaço de cano.
        verts.append((math.cos(a) * TRUNK_R * 1.35, math.sin(a) * TRUNK_R * 1.35, 0.0))
        verts.append((math.cos(a) * TRUNK_R, math.sin(a) * TRUNK_R, bottom))
    for i in range(n):
        b0, t0 = 2 * i, 2 * i + 1
        b1, t1 = 2 * ((i + 1) % n), 2 * ((i + 1) % n) + 1
        faces += [(b0, b1, t1), (b0, t1, t0)]
    return verts, faces


def fruit(seed: int, crown: list[tuple], count: int = None) -> tuple[list[tuple], list[tuple]]:
    """Frutos pendurados na copa a que pertencem, como uma única malha de esferas pequenas.

    Só por fora da copa e em nenhum outro lugar: fruto dentro da copa é invisível da rua, e um
    robô que procura frutos olha para a saia externa de qualquer jeito. Também um pouco abaixo
    do equador, onde o colhedor encontra a maior parte deles e para onde aponta uma câmera num
    robô de 40 cm.

    `count` é quantos pendurar. A árvore avulsa leva o `FRUIT_N` inteiro, porque é a única de
    que algo chega perto; uma sequência de oito leva menos, porque uma sequência é com o que um
    talhão inteiro é plantado, e 60 esferas por árvore num bloco de nove hectares dão 130 MB de
    OBJ para laranjas de quatro pixels na distância em que o bloco é visto.
    """
    rng = random.Random(seed + 977)
    ball_v, ball_f = icosphere(FRUIT_SUBDIV)
    verts, faces = [], []
    picked = rng.sample(range(len(crown)), min(count or FRUIT_N, len(crown)))
    for idx in picked:
        x, y, z = crown[idx]
        if z > CANOPY_Z + 1.0 or z < 0.35:
            continue                      # nem na crista, nem na poeira
        # Empurrado um pouco mais para fora que a superfície das folhas, para ficar saliente
        # em relação à folhagem e não meio afundado nela.
        k = 1.03
        cx, cy = x * k, y * k
        cz = (z - CANOPY_Z) * k + CANOPY_Z
        base = len(verts)
        verts += [(cx + v[0] * FRUIT_R, cy + v[1] * FRUIT_R, cz + v[2] * FRUIT_R)
                  for v in ball_v]
        faces += [(a + base, b + base, c + base) for a, b, c in ball_f]
    return verts, faces


# --------------------------------------------------------------- material --

# Uma cor de folha por variante. Os verdes diferem por alguns centésimos, que é mais ou menos
# o quanto um bloco de uma variedade difere de árvore para árvore: o bastante para seis delas
# embaralhadas numa fileira não parecerem um objeto repetido, e não tanto que o bloco deixe de
# parecer um único plantio.
#
# O termo ambiente importa mais aqui que o difuso, e é o número a mexer primeiro se estas
# cores parecerem erradas. É o que o lado da árvore sem sol vira, e metade de cada fileira é
# esse lado: baixo demais e a face na sombra vai a quase preto, que é a causa mais comum de um
# objeto verde no Gazebo parecer um buraco no mundo.
LEAF = [(0.13, 0.29, 0.10, 0.15, 0.33, 0.11),
        (0.12, 0.27, 0.09, 0.14, 0.31, 0.10),
        (0.14, 0.31, 0.11, 0.16, 0.35, 0.12),
        (0.12, 0.26, 0.09, 0.13, 0.29, 0.10),
        (0.13, 0.30, 0.10, 0.15, 0.34, 0.11),
        (0.14, 0.29, 0.11, 0.17, 0.32, 0.12)]
BARK = (0.14, 0.11, 0.08, 0.30, 0.23, 0.15)
PEEL = (0.34, 0.17, 0.02, 0.93, 0.52, 0.07)


def write_mtl(path: Path, materials: dict[str, tuple]) -> None:
    """As cores, ao lado do OBJ que as nomeia.

    Ficam na malha e não num `<material>` do SDF porque uma malha traz duas coisas que não têm
    a mesma cor: uma copa verde e um toco marrom. Um visual só consegue sobrescrever uma malha
    com um único material, então pôr a cor no SDF foi o que deixou o tronco verde da primeira
    vez.
    """
    lines = ["# Generated by scripts/generate_citrus_meshes.py."]
    for name, (ar, ag, ab, dr, dg, db) in materials.items():
        lines += [f"newmtl {name}",
                  f"Ka {ar:.3f} {ag:.3f} {ab:.3f}",
                  f"Kd {dr:.3f} {dg:.3f} {db:.3f}",
                  "Ks 0.040 0.050 0.035",   # folhagem é fosca; uma folha de citros é
                  "Ns 8.0",                 # cerosa, mas não a 20 m
                  "d 1.0", "illum 2", ""]
    path.write_text("\n".join(lines))


# ------------------------------------------------------------------- OBJ --

def write_obj(path: Path, groups: list[tuple[str, list[tuple], list[tuple], str]],
              note: str, mtl: str | None = None, smooth: frozenset = frozenset()) -> None:
    """Um OBJ, com sombreamento chapado: uma normal por face e nenhuma normal de vértice.

    Normais de vértice médias são o que um exportador escreve, e aqui estão erradas. Elas suavizam
    a luz pela copa toda, que é justamente o efeito que uma malha irregular existe para evitar;
    com normais por face cada faceta pega o sol por conta própria e o agrupamento é iluminado e
    não pintado.

    Exceto os grupos nomeados em `smooth`, que recebem normais médias de propósito. A copa
    simples é um elipsoide e deve parecer uma superfície lisa: com sombreamento chapado nesta
    subdivisão ela sai como uma gema lapidada, o oposto do que se quer.
    """
    lines = [f"# {note}", "# Generated by scripts/generate_citrus_meshes.py.",
             "# Flat shaded on purpose: one normal per face.", ""]
    if mtl:
        lines += [f"mtllib {mtl}", ""]
    v_off, n_off = 1, 1
    for name, verts, faces, material_name in groups:
        lines.append(f"o {name}")
        lines.append(f"usemtl {material_name}")
        lines += [f"v {x:.4f} {y:.4f} {z:.4f}" for x, y, z in verts]
        if name in smooth:
            # Uma normal por vértice, média das faces que se encontram ali.
            acc = [[0.0, 0.0, 0.0] for _ in verts]
            for a, b, c in faces:
                ax, ay, az = verts[a]
                bx, by, bz = verts[b]
                cx, cy, cz = verts[c]
                ux, uy, uz = bx - ax, by - ay, bz - az
                wx, wy, wz = cx - ax, cy - ay, cz - az
                nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
                for k in (a, b, c):
                    acc[k][0] += nx
                    acc[k][1] += ny
                    acc[k][2] += nz
            for nx, ny, nz in acc:
                d = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
                lines.append(f"vn {nx / d:.4f} {ny / d:.4f} {nz / d:.4f}")
            for a, b, c in faces:
                lines.append(f"f {a + v_off}//{a + n_off} {b + v_off}//{b + n_off} "
                             f"{c + v_off}//{c + n_off}")
            lines.append("")
            v_off += len(verts)
            n_off += len(verts)
            continue
        normals, idx = [], []
        for a, b, c in faces:
            ax, ay, az = verts[a]
            bx, by, bz = verts[b]
            cx, cy, cz = verts[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            wx, wy, wz = cx - ax, cy - ay, cz - az
            nx, ny, nz = uy * wz - uz * wy, uz * wx - ux * wz, ux * wy - uy * wx
            d = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            normals.append((nx / d, ny / d, nz / d))
            idx.append((a, b, c))
        lines += [f"vn {x:.4f} {y:.4f} {z:.4f}" for x, y, z in normals]
        for i, (a, b, c) in enumerate(idx):
            n = n_off + i
            lines.append(f"f {a + v_off}//{n} {b + v_off}//{n} {c + v_off}//{n}")
        lines.append("")
        v_off += len(verts)
        n_off += len(normals)
    path.write_text("\n".join(lines))
    tris = sum(len(g[2]) for g in groups)
    print(f"{path}  ({tris} triangles)")


def placed(verts, scale, spin, dx, dy):
    """Os vértices de uma árvore, postos onde ela fica na sequência."""
    c, s = math.cos(spin), math.sin(spin)
    sx, sy, sz = scale
    out = []
    for x, y, z in verts:
        x, y, z = x * sx, y * sy, z * sz
        out.append((x * c - y * s + dx, x * s + y * c + dy, z))
    return out


def run(seed, trees, plain=False, fruited=False):
    """`trees` árvores numa linha ao longo de +X, centradas no meio da sequência.

    Cada árvore tem a própria copa, o próprio giro, os próprios poucos por cento de escala e os
    próprios poucos centímetros fora da linha -- a mesma variação que o gerador de mundos
    aplicava por visual, assada aqui. Seis sequências de oito dão 48 árvores distintas na
    biblioteca, o bastante para uma fileira de quarenta não parecer uma árvore carimbada
    várias vezes.
    """
    rng = random.Random(seed)
    crown_v, crown_f, trunk_v, trunk_f = [], [], [], []
    fruit_v, fruit_f = [], []
    for i in range(trees):
        cv, cf = (plain_canopy(seed * 131 + i * 17) if plain
                  else canopy(seed * 131 + i * 17))
        tv, tf = trunk(min(z for _, _, z in cv) + 0.16)
        scale = (1 + rng.uniform(-0.08, 0.08),
                 1 + rng.uniform(-0.08, 0.08),
                 1 + rng.uniform(-0.10, 0.13))
        spin = rng.uniform(0, 2 * math.pi)
        dx = -(trees - 1) / 2 * RUN_SPACING + i * RUN_SPACING
        dy = rng.uniform(-0.14, 0.14)
        base = len(crown_v)
        crown_v += placed(cv, scale, spin, dx, dy)
        crown_f += [(a + base, b + base, c + base) for a, b, c in cf]
        base = len(trunk_v)
        trunk_v += placed(tv, scale, spin, dx, dy)
        trunk_f += [(a + base, b + base, c + base) for a, b, c in tf]
        if fruited:
            fv, ff = fruit(seed * 271 + i * 13, cv, count=RUN_FRUIT)
            base = len(fruit_v)
            fruit_v += placed(fv, scale, spin, dx, dy)
            fruit_f += [(a + base, b + base, c + base) for a, b, c in ff]
    return crown_v, crown_f, trunk_v, trunk_f, fruit_v, fruit_f


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help="directory to write the meshes into")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    for i, tag in enumerate(VARIANTS):
        seed = 4100 + i * 17
        crown_v, crown_f = canopy(seed)
        bottom = min(z for _, _, z in crown_v) + 0.16
        trunk_v, trunk_f = trunk(bottom)
        write_mtl(args.out / f"citrus_{tag}.mtl",
                  {"leaf": LEAF[i % len(LEAF)], "bark": BARK})
        write_obj(args.out / f"citrus_{tag}.obj",
                  [("canopy", crown_v, crown_f, "leaf"),
                   ("trunk", trunk_v, trunk_f, "bark")],
                  f"Grown citrus, variant {tag}: crown and the stump it stands on.",
                  mtl=f"citrus_{tag}.mtl")
        fv, ff = fruit(seed, crown_v)
        write_mtl(args.out / f"citrus_{tag}_fruit.mtl", {"peel": PEEL})
        write_obj(args.out / f"citrus_{tag}_fruit.obj", [("fruit", fv, ff, "peel")],
                  f"Fruit for citrus_{tag}.obj, hung on that crown's own surface.",
                  mtl=f"citrus_{tag}_fruit.mtl")

    # Uma copa simples por variante, para a árvore que sobra no fim de uma fileira num talhão
    # plantado com copa simples.
    for i, tag in enumerate(VARIANTS):
        seed = 6100 + i * 17
        crown_v, crown_f = plain_canopy(seed)
        trunk_v, trunk_f = trunk(min(z for _, _, z in crown_v) + 0.16)
        write_mtl(args.out / f"citrus_plain_{tag}.mtl",
                  {"leaf": LEAF[i % len(LEAF)], "bark": BARK})
        write_obj(args.out / f"citrus_plain_{tag}.obj",
                  [("canopy", crown_v, crown_f, "leaf"),
                   ("trunk", trunk_v, trunk_f, "bark")],
                  f"Plain citrus, variant {tag}: one smooth crown on its stump.",
                  mtl=f"citrus_plain_{tag}.mtl", smooth=frozenset({"canopy"}))
        fv, ff = fruit(seed, crown_v)
        write_mtl(args.out / f"citrus_plain_{tag}_fruit.mtl", {"peel": PEEL})
        write_obj(args.out / f"citrus_plain_{tag}_fruit.obj",
                  [("fruit", fv, ff, "peel")],
                  f"Fruit for citrus_plain_{tag}.obj, on that crown's surface.",
                  mtl=f"citrus_plain_{tag}_fruit.mtl")

    # Quatro bibliotecas de sequências, que são as duas copas vezes fruto ou sem fruto:
    # irregular, irregular com fruto, simples, simples com fruto. Um talhão é plantado com
    # exatamente uma delas -- um bloco tem uma idade e uma variedade -- e cada uma das quatro
    # custa o mesmo, um visual por oito árvores.
    for size in RUN_SIZES:
        for i, tag in enumerate(RUNS):
            cv, cf, tv, tf, _, _ = run(9200 + size * 7 + i * 31, size)
            write_mtl(args.out / f"citrus_run{size}_{tag}.mtl",
                      {"leaf": LEAF[i % len(LEAF)], "bark": BARK})
            write_obj(args.out / f"citrus_run{size}_{tag}.obj",
                      [("canopy", cv, cf, "leaf"), ("trunk", tv, tf, "bark")],
                      f"Run {tag}: {size} grown citrus at {RUN_SPACING:g} m, "
                      "one mesh, centred on the middle of the run.",
                      mtl=f"citrus_run{size}_{tag}.mtl")

            cv, cf, tv, tf, fv, ff = run(9200 + size * 7 + i * 31, size,
                                         fruited=True)
            write_mtl(args.out / f"citrus_run{size}_{tag}_fruit.mtl",
                      {"leaf": LEAF[i % len(LEAF)], "bark": BARK, "peel": PEEL})
            write_obj(args.out / f"citrus_run{size}_{tag}_fruit.obj",
                      [("canopy", cv, cf, "leaf"), ("trunk", tv, tf, "bark"),
                       ("fruit", fv, ff, "peel")],
                      f"Run {tag}: {size} grown citrus at {RUN_SPACING:g} m "
                      "carrying fruit, one mesh.",
                      mtl=f"citrus_run{size}_{tag}_fruit.mtl")

            cv, cf, tv, tf, _, _ = run(9200 + size * 7 + i * 31, size, plain=True)
            write_mtl(args.out / f"citrus_plain{size}_{tag}.mtl",
                      {"leaf": LEAF[i % len(LEAF)], "bark": BARK})
            write_obj(args.out / f"citrus_plain{size}_{tag}.obj",
                      [("canopy", cv, cf, "leaf"), ("trunk", tv, tf, "bark")],
                      f"Run {tag}: {size} plain crowns at {RUN_SPACING:g} m, "
                      "one mesh.",
                      mtl=f"citrus_plain{size}_{tag}.mtl",
                      smooth=frozenset({"canopy"}))

            cv, cf, tv, tf, fv, ff = run(9200 + size * 7 + i * 31, size,
                                         plain=True, fruited=True)
            write_mtl(args.out / f"citrus_plain{size}_{tag}_fruit.mtl",
                      {"leaf": LEAF[i % len(LEAF)], "bark": BARK, "peel": PEEL})
            write_obj(args.out / f"citrus_plain{size}_{tag}_fruit.obj",
                      [("canopy", cv, cf, "leaf"), ("trunk", tv, tf, "bark"),
                       ("fruit", fv, ff, "peel")],
                      f"Run {tag}: {size} plain crowns at {RUN_SPACING:g} m "
                      "carrying fruit, one mesh.",
                      mtl=f"citrus_plain{size}_{tag}_fruit.mtl",
                      smooth=frozenset({"canopy"}))


if __name__ == "__main__":
    main()
