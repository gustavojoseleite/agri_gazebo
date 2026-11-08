#!/usr/bin/env python3
"""Generate the three citrus farm worlds in worlds/.

    python3 scripts/generate_agriculture_worlds.py            # rewrite them in place
    python3 scripts/generate_agriculture_worlds.py --out /tmp  # try a change first

The worlds are committed and the build never runs this script: Gazebo reads the
SDF, not the generator. It exists because the interesting part of a field is a
loop -- four thousand trees cut to whatever shape the roads leave over -- and
because the numbers worth editing are the agronomic ones at the top of each
builder (row spacing, lane width, bend radius), not the poses they expand
into.

One property, three road networks: citrus_farm_90, _60 and _30 share their
ground, their planting and their mata, and differ only in the angle every bend
on them turns.

Conventions shared by the three worlds, and worth keeping if you add a fourth:

  +X runs along the rows, +Y across them, +Z up (SDF's usual frame).

  The world origin sits on a carreador, on flat ground, at z = 0. That is what
  lets `sim.launch.py` spawn the rover at z = 0.15 in any of these worlds
  without a per-world spawn pose.

  Ground markings -- the worked soil of a talhão, the surface of a carreador --
  are visuals with no collision. A 2 cm lip is nothing to a tractor and an
  immovable wall to a rover with 4 cm casters, so the lanes are painted on the
  ground rather than built up on it.
"""

from __future__ import annotations

import argparse
import re
import math
import random
from pathlib import Path as FilePath
from textwrap import indent

# Imported under another name because a Path down in this file is a road, and
# the road wins: it is what most of the code here is about. Left as `Path`,
# argparse resolved `type=Path` to the road class at call time and every
# `--out` ended in "invalid Path value".
HERE = FilePath(__file__).resolve().parent
DEFAULT_OUT = HERE.parent / "worlds"

# ----------------------------------------------------------------- palette --

C_GRASS = (0.30, 0.40, 0.20)
C_BARK = (0.29, 0.23, 0.17)     # and its trunk, and the mata's
C_LATOSOL = (0.62, 0.31, 0.18)  # the red earth of a citrus grove
C_SHED = (0.70, 0.68, 0.64)


# --------------------------------------------------------------------- XML --

def _n(v: float) -> str:
    """Numbers as SDF wants them: no exponents, no trailing noise."""
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


# Explicit friction, for the same reason empty.sdf sets it: the default
# is slippery enough that a differential drive spins its wheels in place. On
# the slope and the ramps it is the difference between climbing and sliding.
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
    """A static box. collide=False leaves a pure marking on the ground."""
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
    """Turn a position given in a block's own frame into world coordinates."""
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
    """Assemble a world. step is the physics step size, 1 ms unless the world
    is big enough to need otherwise: the cost of a step here is nearly all
    fixed overhead per entity, so a 2 ms step buys back half the real-time
    factor at a cost in contact accuracy that no slow ground robot driving
    among static obstacles will notice."""
    # A double dash is illegal inside an XML comment. Gazebo's parser lets it
    # through, ElementTree and xmllint do not, so a header drawn with ASCII
    # rules made of dashes produces a world that only the simulator can read.
    # Draw the plans with '=' and ':' instead.
    comment = comment.strip()
    inner = comment.removeprefix("<!--").removesuffix("-->")
    if "--" in inner:
        raise ValueError(f"{name}: '--' inside the header comment")
    body = "\n\n".join(indent(p, "    ") for p in parts)
    return WORLD.format(name=name, comment=comment, body=body, step=_n(step),
                        step_name=f"{_n(step * 1000)}ms")


# ----------------------------------------------------------------- scenery --

# The same plants the model library holds, written out as bare shapes so that
# hundreds of them can share one link.
#
# Measured, because it is not obvious: 300 static objects as 300 models cost
# 0.8 ms a step; the same 300 as links inside one model, 0.5 ms; as plain
# shapes inside a single link, 0.3 ms. Gazebo charges for entities, and a model
# is several. Over the six hundred trees, mounds and rocks scattered across
# each farm that is the difference between a world that keeps up with the
# clock and one that runs at a third of it, so out here the include goes and
# the geometry is inlined. What is written below is therefore the only
# definition of a mata tree and a rock. Where the count is small
# model:// stays, and models/ holds exactly those: the tank, the bins, the
# tractor and its trailer, five of a kind rather than six hundred.
#
# (kind: [(collides, geometry, (dx, dy, dz), rgb)])
SCENERY = {
    # model://forest_tree, inlined. Two ellipsoids, one inside the other,
    # on a trunk: the outer one is the canopy and the lighter inner one is the
    # dense crown around the trunk, which is what keeps standing timber from
    # reading as an oversized citrus. Same numbers as models/forest_tree, and
    # that file is where they are explained.
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
    """One model, one link, every scattered plant and rock in it.

    items: (kind, x, y, z, yaw). Yaw turns the off-centre pieces, which is what
    keeps a hundred copies of the same tree from looking like a hundred copies
    of the same tree.
    """
    out = [f'<model name="{name}">', "  <static>true</static>",
           '  <link name="link">']
    for n, (kind, x, y, z, yaw) in enumerate(items):
        for k, (collides, geom, (dx, dy, dz), rgb) in enumerate(SCENERY[kind]):
            ox, oy = rotate(dx, dy, yaw)
            at = pose(x + ox, y + oy, z + dz, 0, 0, yaw)
            # Collision and visual are separate questions, and reading them as
            # one is how a hundred trees ended up as canopies floating over
            # nothing: a trunk both collides and has to be seen, a mound's
            # collision cylinder is a stand-in for visuals it already has.
            if collides:
                out += [f'    <collision name="c{n}_{k}">', f"      {at}",
                        f"      <geometry>{geom}</geometry>", "    </collision>"]
            if rgb is not None:
                out += [f'    <visual name="v{n}_{k}">', f"      {at}",
                        f"      <geometry>{geom}</geometry>",
                        f"{indent(material(rgb), '      ')}", "    </visual>"]
    out += ["  </link>", "</model>"]
    return "\n".join(out)


# ------------------------------------------------------------------ ground --

class Terrain:
    """Rolling ground, built from tilted slabs.

    Gazebo Harmonic's DART will collide with neither a heightmap nor a mesh in
    this build. Both were tried and measured: a sphere dropped on a heightmap
    falls to z = -1686, and on a trimesh to z = -1093, in the second case after
    the mesh loads without complaint. So the ground here is made of the one
    thing that certainly does collide, which is boxes, and the vineyard proves
    it: a rover left on a tilted slab there settles at exactly its 8 degrees
    and stays put.

    A box's top face is a plane, so the surface has to be piecewise planar, and
    for the slabs to tile without leaving a lip at the joins each one may tilt
    about a single axis only. Tilt about two and the top faces project to
    parallelograms that no longer meet at the corners: at these sizes that is
    ten to twenty centimetres of gap, which is nothing to look at and
    everything to a rover with 4 cm casters.

    Hence relief that varies along X and is level along Y. Ridges and valleys
    running north to south, which is a real landform and not a compromise, and
    slabs that meet exactly along each fold because both planes are defined to
    pass through the same node height there.
    """

    def __init__(self, nodes, y_half=250.0, thickness=8.0):
        self.nodes = sorted(nodes)          # [(x, z)] along the profile
        self.y_half = y_half
        self.thickness = thickness

    def _span(self, x):
        for i in range(len(self.nodes) - 1):
            if self.nodes[i][0] <= x <= self.nodes[i + 1][0]:
                return i
        return 0 if x < self.nodes[0][0] else len(self.nodes) - 2

    def strip(self, x):
        """Which planar stretch a point is on. Rows are not allowed to cross
        one of these, so that every hedge lies in a single plane."""
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
        """Where and how a flat thing lies on the ground here.

        The rotation wanted is Ry(pitch) . Rz(yaw): turn the object to its
        heading, then lay the whole thing on the slope. SDF wants that as roll,
        pitch and yaw in the other order, so it is composed and read back.
        """
        p = -math.atan(self.grade(x))
        c, s = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(p), math.sin(p)
        yaw_o = math.atan2(s, cp * c)
        pitch_o = math.asin(max(-1.0, min(1.0, sp * c)))
        roll_o = math.atan2(sp * s, cp)
        return x, y, self.z(x), roll_o, pitch_o, yaw_o

    def slabs(self, name, rgb):
        """The ground itself: one model, one slab per stretch of the profile."""
        out = [f'<model name="{name}">', "  <static>true</static>",
               '  <link name="link">']
        for i in range(len(self.nodes) - 1):
            (x0, z0), (x1, z1) = self.nodes[i], self.nodes[i + 1]
            grade = (z1 - z0) / (x1 - x0)
            theta = math.atan(grade)
            length = (x1 - x0) / math.cos(theta)
            t = self.thickness
            # the top face has to pass through both nodes, so the box centre
            # sits half a thickness back along the slab's own normal
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


# --------------------------------------------------------- roads and curves -

class Path:
    """A road centreline, written the way it is driven.

        Path(0, 0, 90).straight(50).turn(-45, 22).straight(30)

    turn() takes the angle in degrees, positive to the left, and the radius of
    the centreline. The radius is the number that matters. A tractor with a
    loaded trailer on a rigid drawbar cuts the corner, so the inside edge of a
    bend is what its wheels find first, and a curve that looks generous on a
    map is where a driver has to shunt.
    """

    def __init__(self, x, y, heading_deg):
        self.x0, self.y0, self.h0 = x, y, math.radians(heading_deg)
        self.segments: list[tuple[str, float, float]] = []

    def straight(self, length):
        self.segments.append(("s", length, 0.0))
        return self

    def turn(self, degrees, radius):
        """Bend the road. radius=0 is a sharp corner: no arc, just a vertex.

        A graded farm road turns both ways. Where there is room it is bladed
        round, and where two blocks meet square it is simply cut, and the
        aerial photographs are full of both. A vertex costs no straight either
        -- an arc eats radius * tan(half the turn) off the road at each end,
        a corner eats nothing -- which is why a sharp 90 can sit where a
        rounded one would not fit.
        """
        self.segments.append(("t", radius, math.radians(degrees)))
        return self

    def _arc_centre(self, x, y, h, radius, sweep):
        sign = 1.0 if sweep > 0 else -1.0
        return (x - sign * radius * math.sin(h), y + sign * radius * math.cos(h), sign)

    def points(self, step=1.0):
        """The centreline as a dense polyline, for measuring distances to it."""
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
                    h += sweep      # a corner is one point, already in pts
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
        """Position and heading a given distance along the road."""
        pts = self.points(0.25)
        i = min(int(distance / 0.25), len(pts) - 2)
        (px, py), (qx, qy) = pts[i], pts[i + 1]
        return px, py, math.atan2(qy - py, qx - px)

    def turns(self):
        """(degrees, radius) of every bend, for the checks to look at."""
        return [(math.degrees(b), a) for kind, a, b in self.segments if kind == "t"]

    def end(self):
        """Where the road has got to, and which way it is pointing."""
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
        """Run on until the road reaches a given line, and stop there.

        Every block road in citrus_farm leaves the main carreador and comes
        back to it. Working out the length of that last straight by hand means
        redoing the arithmetic every time a radius changes; this just asks.
        """
        x, y, h = self.end()
        d = (y_target - y) / math.sin(h)
        if d <= 0:
            raise ValueError(f"road points away from y={y_target}")
        return self.straight(d)

    def close_to_boundary(self, points, reach_max=700.0):
        """Run on until the road meets the property boundary, and stop there.

        The counterpart of close_to_y for a boundary that is not a straight
        line: a hexagon or a twelve sided ring. Same purpose, which is that no
        road may end in the middle of a field.
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
        """(x, y, heading, degrees, radius, run) at the mouth of every bend.

        run is the straight that leads into it, which is what decides where the
        angle can be written and how big the figures can be. On a road of six
        30s there are twenty metres between bends and on one of two 90s there
        are a hundred and thirty; a number placed a fixed distance back from
        the curve lands on the previous curve in the first case.
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
    """Bucketed points, for asking 'is anything within r of here' ten thousand
    times. A linear scan over every road point turns the generator into a
    minute of waiting; a 10 m grid keeps it under a second."""

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
    """Ray casting, so the mata can be any shape it likes."""
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
    """An irregular closed polygon: what a patch of mata looks like on a map."""
    rng = random.Random(seed)
    return [(cx + rx * (1 + rng.uniform(-wobble, wobble)) * math.cos(2 * math.pi * i / n),
             cy + ry * (1 + rng.uniform(-wobble, wobble)) * math.sin(2 * math.pi * i / n))
            for i in range(n)]


# ----------------------------------------------------------- citrus farms ---

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
  photographs this was built from (agro_gz_worlds/docs/citrus/).

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
    """Turn an offset given in an object's own frame into a world offset."""
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

# Trees baked eight, four and two to a mesh by
# scripts/generate_citrus_meshes.py, and the spacing they were baked at. A row
# is laid in runs, longest first, and the last tree or two planted singly.
#
# This is the entity budget, not a rendering trick. Every <visual> is an entity
# and the server walks all of them on every step, moving or not, rendered or
# not: citrus_farm_60 with 7255 visuals steps 312 times a second, and 1428 with
# the visuals removed. Its 4966 tree visuals become about 620 this way.
CITRUS_RUNS = "abcdef"
RUN_SIZES = (8, 4, 2)
RUN_SPACING = 3.0


# Which crown each talhao is planted with. Three of the four tree models are
# citrus and are named here; the fourth, model://forest_tree, is what a mata is
# made of and is never planted in rows.
#
#   "mesh"         the lumpy polygon crown, baked eight to a run
#   "fruit"        the same crown, carrying oranges
#   "round"        the smooth crown of model://citrus_round
#   "round_fruit"  the smooth crown, carrying oranges
#
# A block not named here is planted "mesh", which is the only one of the three
# that can use the baked runs and therefore the only one that is nearly free.
# The other two cost one visual a tree, and "fruit" costs two, so what this
# table really sets is how much of the entity budget goes on looking at things
# rather than on driving past them. Three blocks of fourteen is the split these
# worlds are tuned around.
#
# They are spread rather than adjacent on purpose: one round block in the
# north west, one south of the yard, and the fruiting one at the crossroads,
# so that no route across the property misses them and none of them can be
# confused with a change in the ground.
PLANTING = {"e": "round", "l": "round", "n": "fruit",
            "nw": "round", "sw": "round_fruit", "se": "fruit"}


def hedgerow_shapes(tag, x0, y0, x1, y1, seed, spacing=3.0, canopy=3.5,
                    terrain=None, kind="mesh"):
    """One run of closed citrus hedge, as shapes already in world coordinates.

    Cut to length rather than fixed at a stock 24 m, because a row that stops
    where a carreador curves cannot be a stock part. That is why a hedge is
    built here and not kept in models/ as a ready-made section.

    Every tree is one visual, whichever crown it wears: a mesh from
    model://citrus_tree with its own trunk inside it, or the single ellipsoid
    of model://citrus_round. Only fruit costs a second one. What these worlds
    used to draw was a bare crown floating with nothing under it — an
    ellipsoid at 1.75 m and no trunk at all — defended on cost, and the
    defence was wrong: a trunk as a *second* shape would indeed have doubled
    the visual count, but a trunk inside the crown's own mesh costs nothing,
    because Gazebo charges per entity and not per triangle. The round crown is
    skirted to the ground for the same reason: it is free to stand it on
    something.

    It returns loose shapes rather than a model so that a whole block's worth
    of them can share one link. Two hundred hedgerows as two hundred models is
    a measurable amount of a simulation step, on the same accounting.

    Trees are planted from runs baked by scripts/generate_citrus_meshes.py,
    eight at a time and then four and two, with at most one tree of any row
    left over to plant singly. That is what keeps the entity count down; see CITRUS_RUNS above.

    `kind` picks which of the three citrus models the row is planted with,
    and the choice is an entity budget as much as a look:

      "mesh"         the lumpy polygon crown of model://citrus_tree.
      "fruit"        the same crown carrying oranges.
      "round"        model://citrus_round's shape: one smooth crown, and the
                     stump it stands on.
      "round_fruit"  the smooth crown carrying oranges.

    Two crowns and fruit or no fruit, which is four libraries and four ways a
    talhao can look. One to a block and no more: a block is planted at one age
    with one variety, and a block of two kinds of tree is two blocks.

    All three are baked eight, four and two crowns to a mesh, so all three
    cost the same, about one visual for every eight trees. One talhao takes
    one of them and no more: a block is planted at one age with one variety,
    and a block of two kinds of tree is two blocks.

    Which talhão gets which is PLANTING, near the top of this file.
    """
    length = math.hypot(x1 - x0, y1 - y0)
    n = int(length // spacing)
    if n < 3:
        return []
    span = (n - 1) * spacing
    yaw = math.atan2(y1 - y0, x1 - x0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    rng = random.Random(seed)
    # A row never crosses a fold in the ground, so one plane describes all of
    # it and the whole run can simply be laid on that plane.
    px, py, pz, roll, pitch, yaw_w = (terrain.pose_on(cx, cy, yaw) if terrain
                                      else (cx, cy, 0.0, 0.0, 0.0, yaw))

    def at(local, spin=0.0):
        ox, oy, oz = rot_apply(roll, pitch, yaw_w, local)
        # A tree's own spin goes on the yaw of the row's plane. On ground this
        # flat that is the same thing as turning it about its own trunk to
        # within a degree, and it costs no second rotation.
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

    # The row is laid in baked runs, longest first, and whatever will not fill
    # a run of two is planted tree by tree. A run only fits if the row is at
    # the spacing the runs were baked at.
    #
    # Which library the runs come from is the whole of what `kind` does. All
    # three are baked eight, four and two crowns to a mesh, so a talhao of
    # fruit and a talhao of plain crowns cost what a talhao of the ordinary
    # ones costs. Before they were baked the fruit went on one tree at a time
    # with a second visual for the oranges, and two nine hectare blocks came to
    # 6180 visuals against the 922 the ordinary crown cost for the same trees.
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
        # Six crowns, shuffled, spun to any angle and scaled a few per cent
        # each. The variation has to be there or a row reads as one tree
        # stamped forty times; inside a run the same variation is baked in.
        v = CITRUS_VARIANTS[rng.randrange(len(CITRUS_VARIANTS))]
        spin = rng.uniform(0, 2 * math.pi)
        sx = 1 + rng.uniform(-0.08, 0.08)
        sy = 1 + rng.uniform(-0.08, 0.08)
        sz = 1 + rng.uniform(-0.10, 0.13)
        scale = f"<scale>{_n(sx)} {_n(sy)} {_n(sz)}</scale>"
        # Off the row line by a few centimetres, the way a planted tree
        # stands: dead straight rows are the other half of why a generated
        # block looks generated.
        across = rng.uniform(-0.14, 0.14)

        # Whatever is left over at the end of a row, planted one at a time.
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
    """A block's worth of hedgerows, in one model and one link."""
    return "\n".join([f'<model name="{name}">', "  <static>true</static>",
                      '  <link name="link">'] + shapes + ["  </link>", "</model>"])


# ------------------------------------------------------ roads, as a ribbon --
#
# A carreador used to be drawn as boxes: a flat slab per straight, a fan of
# them per arc, and then more of them, because the ground folds underneath and
# a box is one plane. Where a road crossed a fold at an angle the fold ran
# diagonally across the rectangle, which no single cut fixes, so the code
# chopped the length every 3 m near the crease and split each chop across its
# width. It worked and it was expensive: the four oblique roads of
# citrus_farm came to 1139 visuals between them, against 190 for the
# whole 2240 m perimeter, and cost the world 0.16 of its real time factor.
#
# So a road is now one mesh. The advantage is not only the entity count: a
# mesh has no obligation to be planar, so the fold does not have to be cut
# around -- it can be an edge of the geometry, which is what it is on the
# ground. The strip is clipped by each fold it crosses and every piece that
# comes out lies in exactly one plane of the terrain. Exact, with nothing to
# tune, and cheaper than the approximation it replaces.

ROAD_COLOURS = {
    "latosol": C_LATOSOL,
    "surface": (0.66, 0.36, 0.22),
}


def road_spine(path, arc_step=2.5):
    """The centreline as stations: (x, y, heading in, heading out).

    A straight gives its two ends, an arc a station every arc_step, and a
    sharp corner one station whose two headings differ. That last one is why
    the spine is shaped like this rather than as a plain polyline: a vertex is
    not a point on a curve, it is a point where the road points two ways at
    once, and the edge of a ribbon laid over it needs both directions to know
    where its own corner goes.
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
            out[-1][3] = h + sweep          # the vertex, in on h, out past it
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
    """Which way, and how far, a road edge steps out at one station.

    On a straight or an arc this is just the left-hand normal. At a vertex it
    is where the two offset lines cross, which is that normal stretched by
    1 / cos(half the turn) -- 1.41 at a right angle. That is not a flourish:
    it is exactly the square corner a grader leaves, and it comes out of the
    same two lines of arithmetic as every other station, which is what let the
    hand-laid patch over each vertex go. The patch was a square of road laid
    over the notch, and it had to be cut around the folds like everything else.
    """
    nx_i, ny_i = -math.sin(h_in), math.cos(h_in)
    nx_o, ny_o = -math.sin(h_out), math.cos(h_out)
    k = 1.0 + nx_i * nx_o + ny_i * ny_o
    if k < 0.2:                    # a turn near 180 degrees has no honest mitre
        return nx_o, ny_o
    return (nx_i + nx_o) / k, (ny_i + ny_o) / k


def _cut_x(poly, f):
    """A convex polygon cut by the vertical line x = f, as up to two."""
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
    """Split a polygon until no piece of it straddles a fold in the ground."""
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
    """One strip of ground-hugging surface, as a list of triangles.

    lo and hi are lateral offsets in metres, left of travel positive: a
    carreador is (-w/2, +w/2) and a wheel rut is a narrow band well off the
    middle of it. Every vertex sits on the ground at its own x and z above it,
    and every triangle lies in one plane of the terrain, because the quads are
    clipped by the folds before they are triangulated.
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
    """One OBJ holding every road of a world, a named object per strip.

    The SDF picks them out with <submesh>, so the models keep the names they
    always had and each one still costs exactly one visual.
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
    """The colours the OBJ names, written once each in the order first used."""
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


# What is laid over what, in metres above the ground. The order is the order a
# road is actually built in, and it has to run by LAYER and not by road, which
# is a lesson this file learned the hard way.
#
# It used to stack each road as a unit -- earth, surface, line, then the next
# road a millimetre higher -- and that buries the line of every carreador
# under the surface of any carreador that joins it. Seven metres of running
# surface crossing at 30 degrees covers fourteen metres of the line it
# crosses, so the reference line down the middle of the main road simply
# stopped for fourteen metres at each junction. A tractor following that line
# by camera followed what was left of it, which was the joining road's line,
# and drove off down the 30 degree road into the talhao.
#
# The half millimetre inside a band separates the roads from each other,
# because coplanar faces are what a renderer cannot choose between -- two
# carreadores crossing at the same height flicker as the camera moves.
ROAD_LAYERS = {"earth": 0.010, "surface": 0.014}
ROAD_LAYER_STEP = 0.0005
PAINT_Z = 0.018          # the painted numbers, above every road surface


def road_parts(world, tag, path, width, earth_half, lift, terrain):
    """One carreador: the SDF models for it, and the mesh groups they name.

    Two strips, each one visual: the bare earth the road wears, and the
    running surface graded on top of it. Which height each goes to is
    ROAD_LAYERS above; `lift` is this road's place within its band.

    A carreador carries nothing else. Two things were tried down the middle of
    it and both came out again -- a painted white line, because nobody paints a
    farm track, and then a pair of wheel ruts, because hard edged stripes
    2.1 m apart read as paint too and match the bitola of no machine on the
    property. What is left is what a graded road is: bare earth, and the
    narrower strip down it that the traffic keeps smooth.
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


# Seven segments, and which of them each digit lights. Only the six digits that
# appear in an angle on this farm are here; a KeyError is the right answer for
# anything else, because the sign would be unreadable rather than wrong.
SEGMENTS = {
    "0": "abcdef", "3": "abcdg", "4": "bcfg",
    "5": "acdfg", "6": "acdefg", "9": "abcdfg",
}

def ground_number(name, x, y, text, digit_h=4.2, digit_w=2.5, stroke=0.7, gap=1.0,
                  margin=0.9, board=(0.92, 0.90, 0.80), ink=(0.16, 0.08, 0.06),
                  z=PAINT_Z, terrain=None):
    """A number painted flat on the ground: digits side by side, north up.

    The first version of this wrote the figures the way a carriageway writes
    SLOW -- stretched, stacked one after another along the direction of travel,
    each one read as you drive over it. On a road that is being looked at from
    above rather than driven, that is unreadable: the digits arrive one at a
    time and at whatever angle the curve happens to be pointing, so a 30 and a
    90 look much the same.

    So: digits side by side in one number, and every number on the property
    laid out north up, whatever direction its road runs. Consistent orientation
    is what makes a set of numbers scannable; a reader looking down at the farm
    does not have to work out which way round each one is. The figures sit on a
    painted board, because dark on pale reads at a glance and pale on red
    latosol does not.

    The whole thing is under 10 m across the diagonal, which is what lets it
    sit inside the 11 m of bare earth at any road heading.
    """
    n = len(text)
    span = n * digit_w + (n - 1) * gap
    # One model pose, with the slabs as offsets inside it. That only works
    # because the board is never put over a fold in the ground: the placement
    # search below refuses those spots, so the whole number always lies on one
    # plane.
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
    """Rasterise what was just built, so the layout can be read without a GUI.

    This goes into the world's own header. A field of this shape is not
    something anyone can check by reading six hundred poses, and it is not
    something I can check by looking at a screen either.
    """
    xs = [p[0] for r in roads for p in r] + [x for row in rows for x in (row[0], row[2])]
    ys = [p[1] for r in roads for p in r] + [y for row in rows for y in (row[1], row[3])]
    x0, x1, y0, y1 = min(xs) - 6, max(xs) + 6, min(ys) - 6, max(ys) + 6
    nx, ny = int((x1 - x0) / cell) + 1, int((y1 - y0) / cell) + 1
    grid = [[" "] * nx for _ in range(ny)]

    # Priority is this string's own order, so a road drawn over a row wins and
    # the origin wins over everything. The three row characters sit together
    # because they never share a cell: one talhão, one crown.
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
    """Nothing a machine would have to drive round may stand in a carreador.

    The roads are the one part of these worlds a driverless vehicle is
    entitled to: the talhoes are obstacles by definition and the headland is
    where things get left, but a carreador with a water tank in it is a road
    that does not go anywhere. Fifteen metres from the centreline is the 6 m
    of bare earth the road wears, plus room to pass whatever is parked without
    putting a wheel on the crop.

    props is (name, x, y) for everything placed on the ground: the yard, the
    machinery, the bins, the shed.
    """
    lines = [q for _, path in net for q in path.points(2.0)]
    grid = NearGrid(lines)
    bad = [f"{what} at ({x:.0f}, {y:.0f})" for what, x, y in props
           if grid.within(x, y, clear)]
    if bad:
        raise ValueError(f"{name}: standing in a carreador: " + "; ".join(bad))


def check_network(name, net, bend_spacing=0.0, oblique=False, pairs=False):
    """Refuse to build a world whose roads are wrong where it does not show.

    Every one of these was a real fault before it was a check. citrus_farm_60's
    third carreador left the property through the southern boundary, ran 180 of
    its 270 m outside the hexagon and came back in: on the plan that is a road
    and a boundary crossing twice with a lens between them, which reads as a
    figure of eight, and in the world it is a road through nobody's field. It
    survived because close_to_boundary only ever looks at where a road *ends*.
    Nothing looked at where it had been.

    The three hard rules, in the order they are cheapest to break:

      inside      a road's centreline stays within the boundary. Six metres of
                  tolerance, because the last point of a road sits on the
                  boundary by construction and floating point puts it either
                  side of the line.
      ends on it  both ends are on the boundary, or in a T against another
                  carreador. A road that stops with nothing at the end of it
                  is a road a machine has to reverse out of; a road that ends
                  on another one has joined the network.
      not beside  no road runs within 32 m of another one, or of the
                  boundary, while pointing the same way, for more than 25 m.
                  Crossing is a junction and junctions are the point; running
                  alongside is one wide mess with a strip of hedge stranded
                  down the middle. The boundary counts: a carreador laid 20 m
                  inside the perimeter is the same mess, and the first cut of
                  this check let that through by comparing carreadores only.
      not square  optional, and only the mixed world asks for it: a road may
                  not meet the boundary within 15 degrees of a right angle. A carreador ending square on the boundary draws
                  a corner exactly like a 90 degree bend, and that corner
                  carries no number, so a world that paints 90 on its bends is
                  telling the reader that the unpainted right angles are
                  something else. They are not. On the mixed world this is
                  what drove the boundary away from being a square: an
                  irregular ring has edges at several headings and a road can
                  always meet one of them at 30 or 60. The single angle worlds
                  do not take this rule either, and for the same reason they
                  do not take the spacing one: in a world where every bend is
                  a 90 and every corner is painted 90, a road meeting the
                  boundary square is the world being consistent with itself.
      on the grid  the carreador that carries the origin runs along x, to
                  within 3 degrees. The vehicle spawns at 0 0 pointing down
                  +x with no per-world yaw, so the road under it has to point
                  that way too: a machine that starts 20 degrees across its
                  own carreador is a machine whose first move is a correction.
      both forms  optional: every angle appears both rounded and as a sharp
                  corner. A farm turns both ways -- bladed round where there
                  is room, cut square where two blocks meet -- and a world
                  meant to ask what a bend costs has to carry both of each,
                  or it is only ever asking about one of them.
      bends apart  optional, and only the mixed world asks for it: consecutive
                  bends must leave `bend_spacing` metres of real straight
                  between them, real meaning after both have eaten their
                  radius * tan(half the turn) off the straight that joins them.
                  A 60 and a 90 of radius 24 that are 60 m apart on paper are
                  22 m apart on the ground, and two arcs that close together
                  read as one long sweep — which is exactly what a 90 must not
                  look like in the world whose point is telling angles apart.
                  The single angle worlds do not take this rule: three 30s back
                  to back is how a road of 30s turns a corner there.

    Crossing another carreador is only reported, not enforced: a road that runs
    edge to edge and meets nothing but the perimeter is thinner than the design
    wants, and citrus_farm_90's south road is one, but it is connected and it
    is not wrong the way the three above are wrong.
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
        """The angle this point of `line` makes with the nearest bit of `other`."""
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
            # On the boundary, or on another carreador. A road that finishes
            # in a T against a road that carries on both ways has not ended in
            # the middle of a field, it has joined the network; only a road
            # that stops with nothing at the end of it has.
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

    # The origin, and what the vehicle finds when it spawns there.
    on_grid = [(t, p) for t, p in ((t, pts[t]) for t in roads)
               if min(math.hypot(q[0], q[1]) for q in p) < 3.5]
    if not on_grid:
        near = min(min(math.hypot(q[0], q[1]) for q in pts[t]) for t in roads)
        bad.append(f"the origin is {near:.0f} m from the nearest carreador")
    else:
        # One of them has to run along x, not all of them: a crossroads at the
        # origin puts two roads through it at right angles, and the vehicle is
        # standing on both.
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
        # The boundary counts. A property is squared off at its corners far
        # more often than a carreador is, so the right angles of a farm live
        # out on the edge and the angles that cut its blocks up live inside:
        # the mixed world takes its two 90s off the perimeter, one bladed
        # round and one cut square, and its 30s and 60s off the diagonals.
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
        # A T counts. A road that finishes on another one has met it, and
        # whether the polyline's last point lands a millimetre short of the
        # other centreline is arithmetic, not geography.
        if not joined:
            joined = any(math.dist(line[i], q) < 4.0
                         for i in (0, len(line) - 1)
                         for o in roads if o != tag for q in pts[o])
        if not joined:
            print(f"  note: {name}: {tag} meets no other carreador")


def segments_cross(p, q, r, s):
    """True if pq and rs meet. Endpoints count: roads are joined, not welded."""
    d = (q[0] - p[0]) * (s[1] - r[1]) - (q[1] - p[1]) * (s[0] - r[0])
    if abs(d) < 1e-12:
        return False
    t = ((r[0] - p[0]) * (s[1] - r[1]) - (r[1] - p[1]) * (s[0] - r[0])) / d
    u = ((r[0] - p[0]) * (q[1] - p[1]) - (r[1] - p[1]) * (q[0] - p[0])) / d
    return 0 <= t <= 1 and 0 <= u <= 1


def _build_farm(name, network, blocks, opening, loops_note, step=0.002,
                bend_spacing=0.0, oblique=False, pairs=False, extent=260.0,
                woods=None, wood_trees=30):
    """The whole property: ground, roads, talhões, mata, machinery.

    network is the road layout, as (tag, Path). One tag must be
    "perimeter", the boundary, which closes on itself; one must be "main", the
    carreador the yard stands on, which has to pass through the origin. blocks
    is the planting, as (tag, centre x, centre y, half length, half width, row
    direction), tried in order with the first claim winning.

    Everything else is shared between the worlds built from this: the ground,
    the mata, the way rows are cut to whatever the roads leave. Change the road
    layout and nothing else, and what a robot does differently is down to the
    roads.

    Which crown each talhão is planted with is PLANTING, and it is the same
    table in all three worlds: eleven blocks of baked mesh runs, two of round
    ellipsoids, one of fruiting citrus, and every mata of forest_tree. Four
    tree models in one property, so that two of them can be compared without
    leaving the world — and so that the cost of the expensive ones is bounded
    by a block rather than spread over a property.
    """
    check_network(name, network, bend_spacing, oblique, pairs)

    ROW_SPACING = 6.5
    TREE_SPACING = 3.0
    CANOPY_W = 3.5

    # One carreador width for the whole property: 7 m of running surface inside
    # 11 m of bare earth, hedge to hedge. Uniform, because the widest thing
    # that uses a farm road is the same everywhere.
    ROAD_W, SHOULDER = 7.0, 2.0
    CLEAR = ROAD_W / 2 + SHOULDER + 0.5 + CANOPY_W / 2

    # --- the lie of the land ------------------------------------------------
    # A profile east to west, in nodes the ground passes through: a face
    # falling 11% off the western boundary, a valley floor around x = -90, then
    # a 10% climb to a ridge at x = 160 and away off the eastern edge. Sixteen
    # metres of relief, which is what it takes to be unmistakable at this size;
    # the first attempt at eight was there and could not be seen. Nothing here
    # is steep for citrus, which is planted on worse, and the yard around the
    # origin is levelled, as a yard is.
    # The same profile whatever the property is, stretched to reach across
    # it: `extent` is how far the ground has to go from the origin, and a
    # world larger than the 260 m the profile was written for gets the end
    # grades carried on rather than a new landform. Ground that stops before
    # the boundary does is a hole a machine falls through.
    #
    # citrus_farm asks for 460 m against a property that reaches 332, and the
    # reason is the view from the air. Ground that stops eight metres past the
    # last tree is a cliff at the edge of the world, and any aerial shot of
    # the place shows the whole farm floating on nothing. It costs no entities
    # to fix -- the slabs are the same boxes, longer -- and a farm ought to sit
    # in a landscape rather than on a table.
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

    # --- the roads ----------------------------------------------------------
    # Nothing is allowed to end in the middle of a field: every road runs from
    # one edge of the property to another or from one road to another, and the
    # perimeter closes on itself, so there is nowhere a machine has to reverse
    # out of.
    by_tag = {tag: path for tag, path in network}
    perimeter, main = by_tag["perimeter"], by_tag["main"]
    bound = perimeter.points(2.0)
    roads = [(tag, path, ROAD_W, SHOULDER, CLEAR) for tag, path in network]

    # --- the mata -----------------------------------------------------------
    # Where the woods are. The default is a patch in each block, in the slack
    # ground a loop leaves in its corner, which is what the through-road
    # worlds were laid out around.
    #
    # `woods` overrides it, and the crossroads world uses that to put the mata
    # where a mata actually is: outside the fence, all the way round. A block
    # of citrus with a wood in the middle of it is a block somebody planted
    # around a wood, which happens, but four of them is four times a
    # coincidence. What the aerial photographs show is the planted ground
    # stopping at the property line and the trees taking over outside it.
    #
    # `wood_trees` is how many go in each patch. A belt is many patches and
    # wants fewer in each: thirty apiece across twelve patches is 360 trees at
    # three visuals each, which is half the world again for scenery nothing
    # drives through.
    matas = woods if woods is not None else [
        blob(150, 150, 26, 20, seed=11),
        blob(-150, 148, 24, 19, seed=22),
        blob(150, -150, 25, 18, seed=33),
        blob(-146, -150, 23, 20, seed=44)]

    # --- the yard -----------------------------------------------------------
    # Chosen here, before anything is planted, because the clearing that keeps
    # the rows out of it is derived from where it lands. The first cut of this
    # searched for the yard after the planting was already laid, so the yard
    # moved and the clearing did not: a shed with citrus four metres from its
    # wall, which is not a yard, it is a shed somebody dropped in a block.
    #
    # The shape is fixed -- the trailer hitched behind the tractor, the shed
    # north of both, the tank at the far side, the bins by the gate -- and only
    # the anchor is searched for, near the origin, out of the roads.
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

    # The clearing is the yard's own ground: nothing is planted inside it, and
    # it is centred on where the yard actually went.
    clearings = [(anchor[0], anchor[1], YARD_CLEAR)]

    road_grids = [(NearGrid(p.points(0.75)), clear)
                  for _, p, _, _, clear in roads]

    parts = [terrain.slabs("terrain", C_GRASS)]

    # --- the mata, planted --------------------------------------------------
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
            return False            # outside the property boundary
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
    # sorted, not set order: a set of strings iterates in whatever order this
    # process's hash seed gives it, and the models come out of this dict in
    # that order. Two runs of the generator then write the same world with its
    # blocks in different places in the file, which is a diff every time and a
    # generator that cannot be checked against what is committed.
    special_shapes: dict[str, list[str]] = {k: [] for k in sorted(set(PLANTING.values()))}
    seed = 0
    for tag, bx, by, hl, hw, yaw in blocks:
        c, s = math.cos(yaw), math.sin(yaw)
        n_rows = int(2 * hw / ROW_SPACING)
        for r in range(n_rows):
            across = -hw + ROW_SPACING * (r + 0.5)
            run_start = None
            run_strip = None
            sample_step = 0.5          # not `step`: that is the physics step
            samples = int(2 * hl / sample_step) + 1
            for k in range(samples + 1):
                along = -hl + k * sample_step
                x = bx + along * c - across * s
                y = by + along * s + across * c
                # A row is broken at a fold in the ground as well as at a road:
                # one hedge, one plane. Rows ending on the crest of a rise is
                # what a field on rolling ground looks like anyway.
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
                            # grouped by where the row is, not by which
                            # block claimed it: block tags differ between
                            # layouts, quadrants do not
                            quad = (("n" if y >= 0 else "s")
                                    + ("e" if x >= 0 else "w"))
                            hedge_shapes[quad].extend(shapes)
                        else:
                            # One model per crown that is not the cheap one, so
                            # what it costs can be counted, timed and taken out
                            # again without touching the quadrant it lies in.
                            special_shapes[kind].extend(shapes)
                        row_segments.append((*run_start, x, y, kind))
                    run_start, run_strip = None, None
                    if ok:                       # a fold, not an obstacle
                        run_start, run_strip = (x, y), here

    for quad, shapes in hedge_shapes.items():
        if shapes:
            parts.append(hedges(f"hedges_{quad}", shapes))
    for kind, shapes in special_shapes.items():
        if shapes:
            parts.append(hedges(f"hedges_{kind}", shapes))

    # --- the roads, drawn ---------------------------------------------------
    # One mesh for all of them, a named strip inside it per layer per road.
    # What goes above what is ROAD_LAYERS: by layer across every road, so that
    # no carreador's surface is ever laid over another carreador's line.
    road_groups = {}
    # The bands only hold if the tallest road in the lowest band still clears
    # the next band up. With seven roads that is 3 mm of stagger against 4 mm
    # of band, which works and would not survive nine.
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

    # --- the angle of every bend, painted on the bend itself ---------------
    # The signposts are gone; this is what replaced them. A number on the
    # ground cannot be knocked over, does not need a post standing in the
    # shoulder, and is legible from directly above, which is where anyone
    # looking at a whole block is looking from.
    #
    # It goes in the middle of the arc, and used to go 28 m back along the
    # straight that led into it. Setting it back is what a road sign does,
    # because a driver needs warning; a number painted on the ground is not a
    # warning, it is a label, and a label sitting on an empty straight has to
    # be matched to a bend by eye. Half the time the wrong one: on a property
    # of mixed angles the number before a bend and the number after it are
    # both visible from the same place, and nothing says which belongs to
    # which. On the arc there is no question -- the figure is lying in the
    # curve it names.
    #
    # The arc is also the widest, flattest piece of road available: the bare
    # earth runs 11 m through the bend as everywhere else, and the board is
    # 7.8 by 6.0 m, so it lies inside it at any heading.
    paint = []
    others = {tag: path.points(2.0) for tag, path in network}
    for tag, path in network:
        elsewhere = [q for t, pts_ in others.items() if t != tag for q in pts_]
        for m, (mx, my, mh, deg, radius, run) in enumerate(path.marks()):
            turn = math.radians(deg)
            cx, cy, sign = path._arc_centre(mx, my, mh, radius, turn)

            def on_arc(frac):
                """A point along the bend: 0 at its mouth, 1 at its exit."""
                a = mh + turn * frac
                return cx + sign * radius * math.sin(a), cy - sign * radius * math.cos(a)

            def past(end, metres):
                """`metres` beyond one end of the arc, along the straight."""
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

            # In the middle of the bend, and then either side of it, and only
            # then a little past its ends. A number lying across a crossroads
            # is the one placement that cannot be read at all, so the search
            # gives that up last: 22 m of clearance is the board's own 5 m plus
            # the 11 m of bare earth the other road is wearing, plus room to
            # see which road the number belongs to.
            # The middle of the bend first, then either side of it along the
            # arc, and only then a few metres past its ends -- never far enough
            # to stop looking like the number of that bend. Twelve metres is
            # the cap, which on the shortest arc here is the board's own
            # length: past that it would read as belonging to the straight.
            spots = ([on_arc(f) for f in (0.5, 0.42, 0.58, 0.34, 0.66,
                                          0.26, 0.74, 0.18, 0.82)]
                     + [past(e, m) for m in (6.0, 12.0, 18.0) for e in (1, -1)])
            # Every number the same size, always. The board used to shrink to
            # 68% when the search could not find a spot clear of a fold, and a
            # property whose numbers are two different sizes reads as though
            # the smaller ones meant something else. Size is not information
            # here; the digits are. So the search moves the board and never
            # resizes it, and it has more places to try instead.
            side = 0.0
            best = next((q for q in spots
                         if clear_of_roads(q) > 22.0 and not on_fold(q)), None)
            if best is None:
                # A bend with a fold through every one of those spots. Keep
                # the junction rule, give up the fold rule, and set the board
                # beside the crease rather than across it, so that all of it
                # still lies on one plane. No spot on this property needs
                # this any more -- the widened list above always finds one --
                # but a bend put on a crease later would.
                best = next((q for q in spots if clear_of_roads(q) > 22.0), None)
                if best is not None:
                    near = min((f for f, _ in terrain.nodes),
                               key=lambda f: abs(best[0] - f))
                    gap = best[0] - near
                    side = math.copysign(2.85, gap if abs(gap) > 0.05 else 1.0)
            if best is None:
                # Nowhere near this bend is clear of the road it crosses.
                # Take the clearest spot there is: a number beside a junction
                # is still readable, and a missing one is not.
                best = max(spots, key=clear_of_roads)
            px = best[0] - math.sin(mh + turn / 2) * side
            py = best[1] + math.cos(mh + turn / 2) * side
            parts.append(ground_number(f"paint_{tag}_{m}", px, py,
                                       f"{abs(int(round(deg)))}", terrain=terrain))
            paint.append((px, py))

    # --- machinery, on the carreadores and nowhere else ---------------------
    def beside(path, distance, offset):
        x, y, h = path.at(distance)
        return x - math.sin(h) * offset, y + math.cos(h) * offset, h

    def standing(uri, name, x, y, yaw=0.0):
        """Anything that rests on the ground rather than growing out of it: it
        takes the slope with it, the way a parked machine does."""
        return include(uri, name, *terrain.pose_on(x, y, yaw))

    # The tractor and its trailer, parked in the yard alongside the shed rather
    # than out on a carreador. A machine standing in the yard is one between
    # jobs; a machine standing on the road is one that broke down there, and
    # the yard is where a driver actually leaves both of them. The shed runs
    # from y = 21 to y = 31, so the pair sits 1.4 m off its south wall, nose
    # east along the rows, in the levelled clearing and clear of the water tank
    # at (-36, 18) and of the bins by the gate.
    #
    # The simulated tractor is this same prop -- autonomous_tractor_description
    # copies its bodywork panel for panel -- so the two are the same machine in
    # two states: one parked by the shed, one out at the origin with the ROS
    # graph attached to it. That is a fair thing for a world to show, as long
    # as the driven one is the one on the carreador.
    # The yard sits back from the road, and how far back is a rule rather
    # than a look: 15 m from any centreline, which is the 6 m of bare earth the
    # carreador wears plus room for a machine to pass a parked one without
    # leaving the road. check_props below refuses anything closer. The first
    # cut of this world had the water tank 1 m off the centreline and the field
    # bins 4.8 m off, which is a tank and two bins standing in the road a
    # driverless tractor is supposed to have to itself.
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

    # Bins waiting at a row end. They used to sit 4.8 m off the centreline,
    # which is on the bare earth of the carreador: near enough to load from the
    # road, and squarely in the way of anything driving it. Sixteen metres puts
    # them in the headland at the end of the rows, which is where a grower
    # actually leaves a bin he is not filling today.
    for i, dist in enumerate((reach(bound, 180) + 150.0, reach(bound, 180) + 300.0)):
        # Sixteen metres off the centreline is clear of the road it stands
        # beside, and nothing says it is clear of the next road along. Walk on
        # until it is clear of all of them, which on a property this size takes
        # a few tens of metres at most.
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

    # --- what grows in everything that is left ------------------------------
    # A farm is not planted edge to edge. What is not a talhao, a road or a
    # patch of mata is pasture, and pasture in this part of the country is
    # grass with trees standing about in it, singly or in twos, left where they
    # were when the land was cleared. Filling that ground is not decoration: an
    # open field with nothing in it is the one place a navigation stack never
    # has to work, and it is also the least like the photographs.
    #
    # Scattered from a fixed seed and rejected against everything already
    # placed, so the arrangement is irregular but the same on every run.
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
        # `spread` is how far the thing itself has to be held off a road
        # beyond the 1.5 m of margin a point object needs. Nothing wants more
        # than that today; the parameter is here because the first attempt at
        # standing timber out here did, and finding out why is the comment
        # below the loop.
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

    # What is loose in the headland: standing timber the farm never cleared,
    # and the odd rock. There were cupinzeiros here, forty of them, and they
    # came out -- a 1 m mound of brown ellipsoids beside a carreador reads as
    # something dropped rather than something grown, and at this size nobody
    # looking at the world could tell what it was meant to be.
    # A tree stands where a mound stood: 7.5 m from the carreador's centreline,
    # which puts the trunk 1.5 m outside the bare earth and leaves the canopy
    # leaning a metre over the shoulder, the way a tree beside a farm road
    # does. What it must not lean over is the 7 m of running surface, and at
    # 2.6 m of canopy from 7.5 m out it clears that by 1.4 m.
    # The two are placed alternately rather than one kind and then the other,
    # and in citrus_farm that matters. They compete for the same headland, and
    # that headland is small: citrus_farm, planted right up to its
    # carreadores, has room for twenty-two loose things where the worlds whose
    # blocks leave slack take all fifty-eight. Whichever kind ran first took
    # the lot -- rocks first left three trees, trees first left one rock -- so
    # they take turns, and each ends up with its share of what there is: 14
    # trees and 8 rocks here, the full 40 and 18 in the other three.
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
    """A road through a list of corners, with an arc of one radius at each.

    Declarative where block_loop was procedural: say where the road turns and
    the straights between the arcs are worked out so that each arc is tangent
    to both. A corner eats radius * tan(half the turn) off the straight on
    either side of it, which is the whole of the arithmetic and exactly the
    part that is tedious to keep right by hand when a corner moves.
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
    """A property boundary that turns one angle and closes on itself.

    360 degrees of turning divided by the angle gives the number of sides: four
    at 90, six at 60, twelve at 30. Sized by the inradius, which is the useful
    number here because it is the distance from the crossroads to the nearest
    stretch of boundary, and therefore how much room the talhoes have.
    """
    n = round(360 / angle)
    side = 2 * inradius * math.tan(math.pi / n)
    leg = side - 2 * radius * math.tan(math.radians(angle) / 2)
    p = Path(-inradius, 0.0, 90).straight(leg / 2)
    for i in range(n):
        # the last leg runs two metres past the start, so the loop visibly
        # shuts and the final corner is bounded by a straight like the others
        p.turn(-angle, radius).straight(leg if i < n - 1 else leg / 2 + 2)
    return p


def mixed_perimeter(turns, free, radius, h0=90.0, radii=None):
    """A boundary that turns a different angle at each corner and still closes.

    polygon_perimeter draws a regular polygon: every corner turns the same
    angle, so every corner paints the same number, and a property with four
    right angles meets every road that crosses it at a right angle too. Both
    of those read wrong on the mixed world, where a painted 90 is supposed to
    mean something: if the four corners all say 90 and every carreador joins
    the boundary in an unlabelled right angle, the numbers stop being
    information.

    So this one takes the turns it should make -- two 90s, two 60s and two 30s
    here -- and the first n-2 side lengths, and solves the last two so that the
    ring closes. Turning 360 degrees in total is the only thing not free: the
    closure is two equations, sum of L*cos(heading) and sum of L*sin(heading)
    both zero, which is exactly enough to pin two lengths.

    `radii` gives each corner its own radius, and 0 there is a corner with no
    arc at all. A field boundary is squared off far more often than it is
    bladed round -- the aerial photographs are all sharp corners -- so the
    mixed world takes two of each on its four right angles, which is also how
    it gets a sharp 90 and a rounded one without putting either inside the
    property.
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

    # Corners, then the centroid, so the property sits around the origin
    # rather than wherever the first side happened to start.
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
    # two metres past the start, so the loop visibly shuts
    p.straight(sides[0] / 2 - back[-1] + 2)
    return p


def reach(points, heading, start=60.0, stop=520.0):
    """How far a road can run from the origin before it meets the boundary."""
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


# The talhoes of the crossroads world, and they are not a list of rectangles
# somebody chose: they are the four cells the roads leave. A block of citrus is
# bounded by carreadores on every side, because that is how a grower gets a
# machine to it and how he gets the fruit out, and a group of trees with no
# road down one side of it is a group nobody can pick. So the blocks here are
# the quadrants, and the boundary road and the two crossing carreadores are
# their four sides.
#
# Rows run east-west in two of them and north-south in the other two. Adjacent
# blocks at different row directions is what the aerial photographs show, and
# it is also the only thing that makes a block boundary visible from the air
# once the canopies have closed.
CROSS_BLOCKS = [
    ("ne", 141.0, 141.0, 133.0, 133.0, math.radians(0)),
    ("nw", -141.0, 141.0, 133.0, 133.0, math.radians(90)),
    ("se", 141.0, -141.0, 133.0, 133.0, math.radians(90)),
    ("sw", -141.0, -141.0, 133.0, 133.0, math.radians(0)),
]


def through_network(angle, radius, inradius, spec):
    """Roads that cross a property, each turning one angle and nothing else.

    spec is (tag, bearing, heading, moves): where on the boundary the
    road starts, which way it sets off, and then the moves as (straight, turn
    sign) pairs. The closing straight is computed, so a road always finishes on
    the boundary rather than in the middle of a field.
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
    """The same as through_network, with the turn angles written out.

    For the farm whose bends are deliberately of different sizes: each move is
    a straight and then a signed turn in degrees, or zero for no turn. Its
    siblings take one angle and imply it on every bend; this one is handed
    each angle because that mixture is the whole point of the world. It is
    handed its boundary too, because that boundary is not a regular polygon.

    A sixth item in a road's tuple is a y value to finish on instead of the
    boundary: that road ends in a T against whichever carreador runs along
    that line, which is how a road inside one block joins the property's
    network without running out to the fence.
    """
    bound = per.points(2.0)
    net = [("perimeter", per)]
    for entry in spec:
        tag, bearing, heading, moves = entry[:4]
        close_on = entry[4] if len(entry) > 4 else None
        d = reach(bound, bearing)
        p = Path(d * math.cos(math.radians(bearing)),
                 d * math.sin(math.radians(bearing)), heading)
        # (straight, turn) rounds the bend at the world's radius; a third
        # number overrides it, and 0 there is a sharp corner.
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
    """The 60: three carreadores, one turn out and one back on each.

    A boundary that closes on itself has turned 360 degrees, so at 60 it is a
    hexagon, and that is the one thing about this world that is not a choice.
    """
    return _through_farm(
        60, 26,
        [("main", 180, 0, [(250, -1), (80, 1)]),
         ("cross", 95, -90, [(110, 1), (60, -1)]),
         # From the south east edge to the north west one, across everything.
         # It used to start at the bottom point of the hexagon and set off due
         # east, where the property is 35 m wide: 180 of its 270 m ran outside
         # the boundary, and a road crossing its own boundary twice draws a
         # figure of eight on the plan. check_network refuses that now.
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


SQUARE = 560.0                  # the property, side to side

# The belt of woods, right round the outside of the square. The wood keeps the
# outside of the world where it has always been -- nothing here reaches past
# 332 m from the origin, which is where the belt reached when the property was
# a 400 m square -- and everything it gives up on the inside goes to the
# talhoes. The strip is 18 m across now where it started at 40, 24 000 m2 of
# canopy against the first cut's 122 000, and the fence has moved out from
# 200 m to 280 in the same ground.
#
# A belt is scenery nobody drives through, a talhao is what the machine is
# there for, and when the two want the same metres the talhao takes them. Long
# along the side and narrow through it, so the wood still closes the horizon
# without eating the field.
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
    """The property this workspace is really about.

    A 560 m square, two carreadores crossing square at the origin, four
    talhoes one crown apiece, a belt of woods outside the fence, and four
    oblique roads cut through the blocks: a 60 degree bend in each upper
    talhao and a 30 degree one in each lower.

    Sixty degrees at all three points of each road in the upper half, and
    thirty at all three of each road in the lower half, which is what the
    shapes are for. It leaves the boundary road at the top of its block at 60, not
    square to it; it turns 60 in the middle of the block; and it meets the
    main carreador at the bottom at 60. A shallow V, down one way and down the
    other, and a machine driving it takes the same angle three times on three
    different kinds of junction: a T off the boundary, a bend in open ground,
    a T onto the main.

    Coming off the north boundary square, which is where this started, made
    the first of those three a right angle on a road whose whole subject is
    60 degrees. Entering at 60 fixes it, and the arithmetic then fixes
    everything else: 60 off a horizontal road, turn 60, and the road is at
    -120, which meets the horizontal main at 60 again.

    The bend starts in the most central part of the talhao, which is the other
    half of the point: a machine that enters the block from the north meets
    the curve with the whole block around it, not against a fence, and the
    rows on both sides are cut to it.

    They mirror each other, so the world holds the same bend twice in opposite
    hands on ground of the same slope. All four are bladed round at the world's
    24 m; any of them becomes a sharp corner by putting a 0 in as the third
    number of its move.

    The two lower roads do the same job at 30 degrees and cannot do it in the
    same shape, and the arithmetic says why. A leg at 30 degrees to the
    horizontal meets a vertical road at 60; a leg at 60 meets a horizontal
    road at 60. With one bend of 30 the two legs are -30 and -60: the first
    has to finish on a horizontal carreador, which wants a tall block, and the
    second on a vertical one, which wants a wide block. No rectangle is both.
    Sixty degrees escapes this because its two legs, -60 and -120, are both at
    60 to the horizontal, so both can finish on the main and the boundary.

    So the lower roads take their 30 degrees at both junctions -- off the main
    at 30, onto the side boundary at 30 -- and give up the exact centre: the
    bend lands 72 m from the middle of a 266 m block, in its central third.
    The alternative, a bend dead centre with one junction at 60, trades the
    angle this world is about for a position.
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
    """The 90 degree world, laid out as roads that cross the property.

    The first version of this ringed each quarter of the farm with a loop, and
    the result was a rectangle inside a rectangle inside a rectangle: every
    route went round something, and none of them went anywhere. A farm road
    network is not made of rings. It is made of roads that run from one edge of
    the property to another and cross each other on the way, and the talhoes
    are whatever the crossings leave over, which is what the aerial
    photographs show and what this now does.

    Four through routes and a boundary, ten bends, all of them 90 degrees.
    """
    R, EDGE = 24, 205
    per = polygon_perimeter(90, R, EDGE)
    network = [
        # west edge to east edge, through the origin, stepping north on the way
        ("main", dogleg([(-EDGE, 0), (90, 0), (90, 130), (EDGE, 130)], R)),
        # north edge to south edge, stepping east under the middle of the farm
        ("cross", dogleg([(-90, EDGE), (-90, -50), (150, -50), (150, -EDGE)], R)),
        # west edge to the south edge, along the bottom of the property
        ("south", dogleg([(-EDGE, -135), (40, -135), (40, -EDGE)], R)),
        # north edge to the west edge, over the top
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
        # 3 ms, and the other two farms take the same. Roads that cross a
        # property leave far more of it plantable than roads that ring parts of
        # it -- 5421 trees here, the most of the three -- and the step is what
        # buys that back. Checked rather than assumed, on the robot that has to live
        # with it: at 3 ms the rover settles dead still at the origin and drives
        # 8.5 m up the carreador without its yaw or its lateral position moving
        # off zero.
        step=0.003)


# -------------------------------------------------------------------- main --

# ------------------------------------------------------- collisions of plants --

PLANT_MODELS = re.compile(r'<model name="(?:hedges|scenery)_[^"]*">.*?</model>', re.S)
COLLISION_BLOCK = re.compile(r"[ \t]*<collision[^>]*>.*?</collision>\n?", re.S)


def without_plant_collisions(text: str) -> str:
    """Strip the collision shapes from every hedgerow and mata model.

    A grove simulated for perception does not need the physics engine to know
    where the trees are: the camera sees them because they are visuals, and the
    solver was spending its time on 480 trunk cylinders nothing ever touched.
    Measured in citrus_farm_60 with the tractor spawned, over 20 s of simulated
    time: 32.8 s of wall clock with them, 29.0 s without.

    What is NOT touched, and the distinction is the whole point: the ground
    (the terrain slabs), the structures that decide where a machine can go
    (the shed) and the props included from models/ keep every collision they
    had. Driving through a tree is a deliberate trade; falling through the
    ground would make all three worlds meaningless.

    Pass --collisions full to get the strict worlds back.
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
    # The road meshes are written from scratch every run, stale ones included:
    # this directory holds nothing a human has edited, and a world renamed
    # would otherwise leave its carreadores behind for good.
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
