#!/usr/bin/env python3
"""Write the citrus canopy meshes that models/citrus_tree and the talhões use.

    python3 agro_gz_worlds/scripts/generate_citrus_meshes.py            # rewrite in place
    python3 agro_gz_worlds/scripts/generate_citrus_meshes.py --out /tmp  # look first

Six variants of one tree, as OBJ, plus a matching cloud of fruit for each. The
files are committed; the build never runs this and Gazebo only ever reads the
OBJ, exactly as it only ever reads the generated worlds.

Why a mesh at all, when every other plant in this library is primitives.

A grown citrus is not an ellipsoid and does not read as one. In the photographs
in docs/citrus/ the crown is a lumpy mass of foliage clumps, and what makes it
legible as a tree is the light: the clumps facing the sun are pale, the pockets
between them are almost black, and the silhouette against the alley is ragged.
An ellipsoid gets none of that. It is a smooth surface, so it takes one smooth
gradient from the sun and reads as a green balloon, and two hundred of them in
a row read as wallpaper. No amount of tuning the material fixes that, because
the problem is the geometry.

A lumpy, flat shaded surface gets the whole effect for free from the sun that
is already in the world: every facet takes its own light, so the clumping is
lit rather than painted. That is why the normals here are per face and not
averaged per vertex, which is the opposite of what a mesh exporter does by
default.

And it costs nothing to speak of. The farm worlds' own measurements say Gazebo
charges per *entity*, not per triangle: 300 objects as separate models cost
0.8 ms a step, as links in one model 0.5, as plain shapes in a single link 0.3.
A mesh visual is one shape whether it carries 8 triangles or 8000, so replacing
one ellipsoid with one 1296 triangle canopy leaves the step where it was and
moves the work onto the GPU, which is idle. The fruit is the one real cost,
because it is a second visual per tree.

The trunk is part of the canopy mesh rather than a second shape for the same
reason: a skirted tree barely shows one, and a separate cylinder would double
the entity count to draw 20 cm of bark.
"""

from __future__ import annotations

import argparse
import math
import random
from pathlib import Path

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "models/citrus_tree/meshes"

VARIANTS = "abcdef"

# A run is eight trees at 3 m, baked into one mesh, and it exists for one
# reason: every <visual> in a world is an entity, and the server walks all of
# them on every step whether or not anything moves or renders. Measured on
# citrus_farm_60, which carries 7255 of them: the whole world runs at 312
# physics steps a second, and 1428 with the visuals taken out. The trees alone
# are 4966 of that count.
#
# Eight to a mesh takes the tree count from 4966 visuals to about 620.
#
# Why not bake a whole row, which would take it to 339: an OBJ of one tree is
# 87 KB, so a row averages 3.5 MB and the three farm worlds would carry 1.3 GB
# of unique geometry. A library of six runs is 4 MB and gets most of the way.
# Eight crowns is also 24 m, the length a stock section of row was always cut to.
# Three lengths, because a row is not a multiple of anything. Rows here are cut
# by roads and by the folds in the ground, so they come out at every length
# from three trees to forty, and with only the eight the leftovers were a third
# of all the trees on the property. Laying 8, then 4, then 2, then singles
# leaves at most one tree of each row unbaked.
RUNS = "abcdef"
RUN_SIZES = (8, 4, 2)
RUN_SPACING = 3.0

# These are the radii of the sphere before the lobes are pushed out of it, so
# the crown that comes out is bigger than twice one of them: measured on the
# OBJ, 3.95 m across and 2.83 m to the top. Planted at 3 m in a row 6.5 m from
# the next, so the canopies still overlap along the row and leave 2.55 m of
# alley across it — a hedge, not a line of separate obstacles.
#
# They were 14% larger (1.75/1.90/1.42 on a 1.53 centre, measuring 4.60 m
# across and 3.29 m tall) and two things were wrong with that. The block read
# as heavier than a citrus block looks, and 4.60 m of canopy in a 6.5 m
# carreira leaves 1.90 m of alley, which a 2.15 m tractor does not fit down at
# all — the arithmetic everything else was written against assumed a 3.5 m
# canopy and nobody had measured the mesh. At 3.95 m the alley is 2.55 m and
# the machine has 20 cm a side.
#
# Shrinking the crown rather than opening the carreira is the deliberate half:
# the spacing is agronomy and not mine to invent, the crown is whatever a tree
# of a given age happens to be.
CANOPY_RX = 1.50      # along the row
CANOPY_RY = 1.63      # across the row
CANOPY_RZ = 1.22
CANOPY_Z = 1.32       # centre height, so the top lands at 2.54 m
SKIRT = 0.78          # how much the underside is flattened
TRUNK_R = 0.115
FRUIT_N = 60
RUN_FRUIT = 18        # per tree inside a baked run; see fruit()
FRUIT_R = 0.038

# Level 3 is 1296 triangles, level 2 is 336 and level 4 is 5136. All three are
# one entity and so all three cost the same by the measure that matters here,
# and the middle one still wins, which is the part worth writing down: at 336
# the facets are 22 cm across on a 3.3 m crown and read as a cut gemstone, and
# at 5136 they are 6 cm, small enough that the eye averages them back into the
# smooth blob this whole file exists to avoid. Twelve centimetres is about the
# size of a clump of citrus leaves, and it is the size at which the light stops
# being a gradient and starts being foliage. Rendered all three and looked.
#
# The fruit needs its own subdivision for a plainer reason: a bare icosahedron
# is visibly a twenty sided die at arm's length.
CANOPY_SUBDIV = 3
FRUIT_SUBDIV = 1


# ------------------------------------------------------------------ sphere --

def icosphere(subdiv: int) -> tuple[list[tuple], list[tuple]]:
    """Unit sphere as an icosahedron subdivided `subdiv` times.

    An icosphere and not a UV sphere: its triangles are all the same size, so
    the lumps deform evenly and there is no pole where twenty slivers meet.
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


# ------------------------------------------------------------------ canopy --

def canopy(seed: int) -> tuple[list[tuple], list[tuple]]:
    """One crown: a sphere pushed out into foliage clumps and skirted.

    Two scales of deformation, because a citrus crown has two. Six to nine
    broad lobes are the scaffold branches, each carrying its own mass of leaves
    a metre or so across; on top of that every vertex gets a few centimetres of
    jitter, which is what stops the lobes themselves from looking like smooth
    blisters. The jitter is applied to the shared vertices before the faces are
    split, so neighbouring facets still meet — a hole in a canopy shows up as a
    black speck from the alley and is very hard to find afterwards.
    """
    rng = random.Random(seed)
    verts, faces = icosphere(CANOPY_SUBDIV)

    lobes = []
    for _ in range(rng.randint(6, 9)):
        # Biased upwards: the mass of a skirted citrus sits in the top two
        # thirds, and lobes hung under the tree would fill the gap the robot
        # actually drives past.
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
            # The underside of a hedge is cut flat by the mower and by the
            # pickers working under it, not rounded like the top.
            z *= SKIRT
        out.append((x, y, z + CANOPY_Z))
    return out, faces


def plain_canopy(seed: int) -> tuple[list[tuple], list[tuple]]:
    """The same crown with none of the lumps: one smooth ellipsoid, skirted.

    This is the plain crown -- the shape these worlds were first planted with
    -- and it exists as a mesh for one reason only, which is that a mesh can
    be baked eight to a run and a primitive cannot. As an SDF <ellipsoid> it
    costs a visual a tree; a nine hectare talhao of them came to 2986 visuals
    against the 461 the same block costs in baked runs. The shape is identical
    either way, so nothing is given up but the entity count.

    Still a few per cent of its own on each axis, so a row is not one ball
    repeated, and skirted underneath like the lumpy one.
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
    """The 20-odd cm of trunk a skirted tree shows, as an eight sided stump."""
    verts, faces = [], []
    n = 8
    for i in range(n):
        a = 2 * math.pi * i / n
        # A citrus trunk flares where it meets the ground; 35% more at the base
        # is what the photographs show and it is what stops the stump from
        # reading as a piece of pipe.
        verts.append((math.cos(a) * TRUNK_R * 1.35, math.sin(a) * TRUNK_R * 1.35, 0.0))
        verts.append((math.cos(a) * TRUNK_R, math.sin(a) * TRUNK_R, bottom))
    for i in range(n):
        b0, t0 = 2 * i, 2 * i + 1
        b1, t1 = 2 * ((i + 1) % n), 2 * ((i + 1) % n) + 1
        faces += [(b0, b1, t1), (b0, t1, t0)]
    return verts, faces


def fruit(seed: int, crown: list[tuple], count: int = None) -> tuple[list[tuple], list[tuple]]:
    """Fruit hung on the crown that goes with it, as one mesh of small spheres.

    On the outside of the canopy and nowhere else: fruit inside the crown is
    invisible from the alley, and a robot looking for it is looking at the
    outer skirt anyway. Slightly below the equator too, which is where a
    picker finds most of it and where a camera on a 40 cm robot is pointed.

    `count` is how many to hang. The single tree takes the full FRUIT_N,
    because it is the one anything gets close to; a run of eight takes fewer,
    because a run is what a whole talhao is planted from and 60 spheres a tree
    across a nine hectare block is 130 MB of OBJ for oranges that are four
    pixels across at the distance the block is ever seen from.
    """
    rng = random.Random(seed + 977)
    ball_v, ball_f = icosphere(FRUIT_SUBDIV)
    verts, faces = [], []
    picked = rng.sample(range(len(crown)), min(count or FRUIT_N, len(crown)))
    for idx in picked:
        x, y, z = crown[idx]
        if z > CANOPY_Z + 1.0 or z < 0.35:
            continue                      # not on the crest, not in the dust
        # Pushed a little further out than the leaf surface, so it sits proud
        # of the foliage rather than half sunk in it.
        k = 1.03
        cx, cy = x * k, y * k
        cz = (z - CANOPY_Z) * k + CANOPY_Z
        base = len(verts)
        verts += [(cx + v[0] * FRUIT_R, cy + v[1] * FRUIT_R, cz + v[2] * FRUIT_R)
                  for v in ball_v]
        faces += [(a + base, b + base, c + base) for a, b, c in ball_f]
    return verts, faces


# ---------------------------------------------------------------- material --

# A leaf colour per variant. The greens differ by a few hundredths, which is
# about what one block of one variety differs by tree to tree — enough that six
# of them shuffled along a row do not read as one object repeated, and not so
# much that the block stops looking like a single planting.
#
# The ambient term matters more here than the diffuse one, and it is the number
# to reach for first if these look wrong. It is what the side of the tree the
# sun is not on comes out as, and half of every row is that side: too low and
# the shaded face goes to near black, which is the most common way a green
# object in Gazebo ends up looking like a hole in the world.
LEAF = [(0.13, 0.29, 0.10, 0.15, 0.33, 0.11),
        (0.12, 0.27, 0.09, 0.14, 0.31, 0.10),
        (0.14, 0.31, 0.11, 0.16, 0.35, 0.12),
        (0.12, 0.26, 0.09, 0.13, 0.29, 0.10),
        (0.13, 0.30, 0.10, 0.15, 0.34, 0.11),
        (0.14, 0.29, 0.11, 0.17, 0.32, 0.12)]
BARK = (0.14, 0.11, 0.08, 0.30, 0.23, 0.15)
PEEL = (0.34, 0.17, 0.02, 0.93, 0.52, 0.07)


def write_mtl(path: Path, materials: dict[str, tuple]) -> None:
    """The colours, beside the OBJ that names them.

    They live in the mesh rather than in an SDF <material> because one mesh
    carries two things that are not the same colour: a green crown and a brown
    stump. A visual can only override a mesh with a single material, so putting
    the colour in the SDF is what turned the trunk green the first time round.
    """
    lines = ["# Generated by scripts/generate_citrus_meshes.py."]
    for name, (ar, ag, ab, dr, dg, db) in materials.items():
        lines += [f"newmtl {name}",
                  f"Ka {ar:.3f} {ag:.3f} {ab:.3f}",
                  f"Kd {dr:.3f} {dg:.3f} {db:.3f}",
                  "Ks 0.040 0.050 0.035",   # foliage is matt; a citrus leaf is
                  "Ns 8.0",                 # waxy but not at 20 m
                  "d 1.0", "illum 2", ""]
    path.write_text("\n".join(lines))


# --------------------------------------------------------------------- OBJ --

def write_obj(path: Path, groups: list[tuple[str, list[tuple], list[tuple], str]],
              note: str, mtl: str | None = None, smooth: frozenset = frozenset()) -> None:
    """One OBJ, flat shaded: a normal per face and no vertex normals.

    Averaged vertex normals are what an exporter writes and they are wrong
    here. They smooth the lighting across the whole crown, which is precisely
    the effect a lumpy mesh exists to avoid; with per face normals each facet
    catches the sun on its own and the clumping is lit rather than painted.

    Except for the groups named in `smooth`, which get averaged normals on
    purpose. The plain crown is an ellipsoid and is supposed to read as one
    smooth surface: flat shaded at this subdivision it comes out a cut gem,
    which is the opposite of the point of it.
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
            # One normal per vertex, averaged over the faces that meet there.
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
    """One tree's vertices, put where it stands in the run."""
    c, s = math.cos(spin), math.sin(spin)
    sx, sy, sz = scale
    out = []
    for x, y, z in verts:
        x, y, z = x * sx, y * sy, z * sz
        out.append((x * c - y * s + dx, x * s + y * c + dy, z))
    return out


def run(seed, trees, plain=False, fruited=False):
    """`trees` trees in a line along +X, centred on the middle of the run.

    Every tree gets its own crown, its own spin, its own few per cent of scale
    and its own few centimetres off the line — the same variation the world
    generator used to apply per visual, baked in here instead. Six runs of
    eight is 48 distinct trees in the library, which is enough that a row of
    forty does not read as one tree stamped over and over.
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

    # One plain crown a variant, for the tree left over at the end of a row
    # in a talhao planted plain.
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

    # Four libraries of runs, which is the two crowns times fruit or no fruit:
    # lumpy, lumpy with fruit, plain, plain with fruit. A talhao is planted
    # from exactly one of them -- a block is one age and one variety -- and
    # every one of the four costs the same, one visual per eight trees.
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
