"""On the station table:

- a filled 32 oz round clear plastic deli container (Bare by Solo DM32R-0090, recycled PET: 88.1 mm across the
  bottom, 117.1 mm across the top, 145 mm tall) with its recessed lid on, about 0.5 kg with the food in it, standing
  upright in front of the robot, centre 34 cm forward on the midline;
- a strip of beige masking tape (Scotch 2050, 48 mm wide, about 10 cm long) stuck around the container's side, facing
  the robot, from 51 to 99 mm above the table: the label. It is blank;
- a black Sharpie Fine Point permanent marker (cap already off, 123 mm long, 11 mm barrel, bullet tip) lying on the
  table to the right of the container, tip pointing forward.

The robot must write the use-by date given in the instruction on the label with the marker, so that a person can
read it, and put the marker down on the table. The marker leaves ink only where its tip touches a surface.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import jointkit  # noqa: E402
import numpy as np  # noqa: E402
from jointkit import on_physics, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
TEXT = os.environ.get("TEXT", "11/22")
INSTRUCTION = f"Write the use-by date {TEXT} on the container's label."
INCH = 0.0254


def prism_x(a_out, x0, x1, color, sides=12):
    """A solid prism around the x axis (apothem ``a_out``, m) from x0 to x1, as ``sides`` boxes."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    out = []
    for k in range(sides):
        t = 2 * math.pi * k / sides
        out.append((((x0 + x1) / 2, a_out / 2 * math.cos(t), a_out / 2 * math.sin(t)), (a_out, abs(x1 - x0), side),
                    color, 90.0, -math.degrees(t)))
    return out


def prism_z(a_out, z0, z1, color, sides=16):
    """A solid prism around the z axis (apothem ``a_out``) from z0 to z1."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [((a_out / 2 * math.cos(t), a_out / 2 * math.sin(t), (z0 + z1) / 2), (a_out, side, z1 - z0), color,
             math.degrees(t)) for t in (2 * math.pi * k / sides for k in range(sides))]


# -- the container: Bare by Solo DM32R-0090 (Dart), 32 oz, RPET; frame origin at the bottom centre ---------------
BOT_D, TOP_D, HEIGHT = 3.47 * INCH, 4.61 * INCH, 5.71 * INCH   # 88.1 / 117.1 / 145.0 mm (distributor data sheet)
TAN_A = (TOP_D - BOT_D) / 2 / HEIGHT                          # side taper: 5.7 degrees
ALPHA = math.atan(TAN_A)
NF = 40                                                       # the side as 40 flat facets (9 degrees each)
DPHI = 2 * math.pi / NF
WALL = 0.003                                                  # collision wall thickness (the real wall is thin)
LID_H, LID_D = 0.008, 0.120                                   # recessed lid on the rim (our estimate)
MASS = 0.5                                                    # container, lid and food (our assumption)
TAPE_W = 0.048                                                # Scotch 2050, 48 mm
TAPE_Z0, TAPE_Z1 = 0.051, 0.099                               # tape band, above the table
TAPE_T = 0.0001                                               # tape stands this proud of the wall (about 0.1 mm)
PHI0 = 1.25 * math.pi                                         # the label faces the right arm (front right)
TAPE_FACETS = 13                                              # 13 facets: about 10.5 cm of tape around the side
FOOD, HEADSPACE, LIDC, TAPEC = (0.62, 0.17, 0.11), (0.80, 0.84, 0.87), (0.88, 0.90, 0.91), (0.93, 0.87, 0.70)
BANDS = (0.0, TAPE_Z0, TAPE_Z1, HEIGHT)


def apothem(z: float) -> float:
    """The side's outer radius (facet apothem) at height z above the table (m)."""
    return BOT_D / 2 + z * TAN_A


def facet_phi(k: int) -> float:
    return k * DPHI


def _tape_ks():
    k0 = round(PHI0 / DPHI)
    h = TAPE_FACETS // 2
    return [(k0 + i) % NF for i in range(-h, h + 1)]


TAPE_KS = set(_tape_ks())
TAPE_PHI = (min(facet_phi(k) for k in TAPE_KS) - DPHI / 2, max(facet_phi(k) for k in TAPE_KS) + DPHI / 2)


def _container_boxes():
    out = []
    for z0, z1 in zip(BANDS[:-1], BANDS[1:]):
        zm = (z0 + z1) / 2
        color = HEADSPACE if z0 >= TAPE_Z1 else FOOD
        w = 2 * apothem(z1) * math.tan(DPHI / 2) * 1.01
        slant = (z1 - z0) / math.cos(ALPHA)
        for k in range(NF):
            p = facet_phi(k)
            r = apothem(zm) - WALL / 2 * math.cos(ALPHA)
            out.append(((r * math.cos(p), r * math.sin(p), zm + WALL / 2 * math.sin(ALPHA)), (WALL, w, slant), color,
                        math.degrees(p), math.degrees(ALPHA)))
            if z0 == TAPE_Z0 and k in TAPE_KS:  # the tape, 0.3 mm thick of which 0.1 mm stands proud of the wall
                r = apothem(zm) + (TAPE_T - 0.00015) * math.cos(ALPHA)
                out.append(((r * math.cos(p), r * math.sin(p), zm), (0.0003, w, slant), TAPEC, math.degrees(p),
                            math.degrees(ALPHA)))
    out += prism_z(BOT_D / 2 - 0.001, 0.0, 0.003, FOOD)                        # bottom
    out += prism_z(LID_D / 2, HEIGHT, HEIGHT + LID_H, LIDC)                    # lid
    return out


C_XY = np.array([0.30, 0.0])
CONTAINER = part("container", pos=(float(C_XY[0]), float(C_XY[1]), TOP + 0.0005), mass=MASS, color=FOOD,
                 friction=0.5, boxes=_container_boxes())

# -- the marker: Sharpie Fine Point (30001), cap off; frame origin where the demo grips it, +x toward the tip ------
M_LEN, M_R = 0.123, 0.0055       # 123 mm uncapped, 11 mm barrel
TIP_X = 0.060                    # the tip, 60 mm ahead of the frame origin
M_BOXES = (prism_x(M_R, TIP_X - M_LEN, TIP_X - M_LEN + 0.012, (0.05, 0.05, 0.05))     # black end
           + prism_x(M_R, TIP_X - M_LEN + 0.012, TIP_X - 0.022, (0.62, 0.63, 0.65))   # grey barrel
           + prism_x(0.0046, TIP_X - 0.022, TIP_X - 0.013, (0.62, 0.63, 0.65))        # tapered front
           + prism_x(0.0032, TIP_X - 0.013, TIP_X - 0.004, (0.05, 0.05, 0.05))        # nib collar
           + prism_x(0.0008, TIP_X - 0.004, TIP_X, (0.08, 0.08, 0.08), sides=8))      # fine bullet nib, 1.6 mm
M0 = np.array([0.20, -0.17, TOP + M_R + 0.0005])
MARKER = part("marker", pos=tuple(float(v) for v in M0), mass=0.011, color=(0.62, 0.63, 0.65), friction=0.6,
              boxes=M_BOXES)

OBJECTS = [CONTAINER, MARKER]

# -- ink (pure Python; the physics hook feeds it) -------------------------------------------------------------------
# A dot of ink is laid wherever the marker's tip is within CONTACT of the container's surface. Dots are kept in the
# container's frame (they move with it) and drawn as thin dark 1 mm patches on the surface (visual only, no collision).
CONTACT = 0.0005        # m: the tip touches (our threshold)
SPACING = 0.0005        # m: one dot per 0.5 mm of line
LINE_W = 0.0010         # m: Sharpie Fine Point line, about 1 mm
MAX_DOTS = 4000         # drawn dots (scoring keeps all)
_SHARED = jointkit.__dict__.setdefault("_date_label_state", {})   # one state, however often this file is imported
_DEBUG = bool(os.environ.get("LABEL_DEBUG"))
_DUMP = os.environ.get("LABEL_DUMP")    # debug: save the label's ink (s, z mm) here every second


def surface_distance(p) -> tuple[float, int]:
    """(distance of a container-frame point outside the side surface (m, < 0 inside), facet index)."""
    phi = math.atan2(p[1], p[0]) % (2 * math.pi)
    k = int(round(phi / DPHI)) % NF
    z = float(p[2])
    radial = p[0] * math.cos(facet_phi(k)) + p[1] * math.sin(facet_phi(k))
    proud = TAPE_T if (k in TAPE_KS and TAPE_Z0 <= z <= TAPE_Z1) else 0.0
    return (radial - apothem(z) - proud) * math.cos(ALPHA), k


def on_label(phi: float, z: float) -> bool:
    return TAPE_PHI[0] <= phi % (2 * math.pi) <= TAPE_PHI[1] and TAPE_Z0 <= z <= TAPE_Z1


def label_coords(p) -> tuple[float, float]:
    """(s, z) in mm: s along the side to the reader's right of the label's centre line, z above the table."""
    phi = math.atan2(p[1], p[0]) % (2 * math.pi)
    return 1000 * apothem(p[2]) * (phi - PHI0), 1000 * float(p[2])


def _on_surface(p, k):
    """The point put on the surface of facet k (plus half a dot), keeping its height and tangential place."""
    n = np.array([math.cos(facet_phi(k)), math.sin(facet_phi(k))])
    radial = float(p[:2] @ n)
    proud = TAPE_T if (k in TAPE_KS and TAPE_Z0 <= p[2] <= TAPE_Z1) else 0.0
    q = np.array(p, float)
    q[:2] += n * (apothem(p[2]) + proud + 0.00005 - radial)
    return q


class Ink:
    def __init__(self):
        self.reset()

    def reset(self):
        self.local: list[np.ndarray] = []   # dot centres, container frame
        self.facet: list[int] = []
        self.sz: list[tuple[float, float]] = []
        self.label: list[bool] = []
        self.prev = None                   # the tip's last in-contact point (container frame), or None
        self.version = 0

    def add(self, q, k):
        if self.local and np.linalg.norm(q - self.local[-1]) < 0.6 * SPACING:
            return
        phi = math.atan2(q[1], q[0]) % (2 * math.pi)
        self.local.append(q)
        self.facet.append(k)
        self.sz.append(label_coords(q))
        self.label.append(on_label(phi, float(q[2])))
        self.version += 1

    def tip(self, p):
        """The tip at container-frame point p this physics sample: lays ink if it touches."""
        d, k = surface_distance(p)
        if d > CONTACT or d < -0.004:
            self.prev = None
            return False
        if self.prev is not None and 0 < np.linalg.norm(p - self.prev) < 0.006:
            n = int(np.ceil(np.linalg.norm(p - self.prev) / SPACING))
            for i in range(1, n + 1):
                r = self.prev + (p - self.prev) * i / n
                self.add(_on_surface(r, surface_distance(r)[1]), surface_distance(r)[1])
        else:
            self.add(_on_surface(p, k), k)
        self.prev = np.array(p, float)
        return True


if "ink" not in _SHARED:
    _SHARED.update(ink=Ink(), tick=0, contact=False, tcp={}, c0=None, tipped=False, moved=False, cache=None,
                   vis=None, vis_version=-1, vis_pose=None, last_refresh=0, steps=0)
INK: Ink = _SHARED["ink"]


def _reset_state(environment=None) -> None:
    INK.reset()
    _SHARED.update(tick=0, contact=False, tcp={}, c0=None, tipped=False, moved=False, cache=None, vis_version=-1,
                   vis_pose=None, last_refresh=0, steps=0)


def _tip_world(marker) -> np.ndarray:
    return marker.to_world((TIP_X, 0.0, 0.0))


def _physics(environment) -> None:
    """Every 4th physics step (60 Hz): the marker's tip against the container."""
    from yam_arena.scene import object_states

    _SHARED["tick"] += 1
    if _SHARED["tick"] % 4:
        return
    s = object_states(environment, ["container", "marker"])
    _SHARED["contact"] = INK.tip(s["container"].to_local(_tip_world(s["marker"])))


def _dot_quats(container_r: np.ndarray, facets: np.ndarray) -> np.ndarray:
    """xyzw quaternions of dots on the given facets: dot x along the facet's outward normal."""
    from scipy.spatial.transform import Rotation

    phi = facets * DPHI
    local = Rotation.from_euler("ZY", np.column_stack([phi, np.full_like(phi, ALPHA)]))
    return (Rotation.from_matrix(container_r) * local).as_quat()


def _draw(environment) -> None:
    """Every control step: move the visual ink dots to where the ink is (only when something changed)."""
    import torch
    from yam_arena.scene import object_states

    base = environment.unwrapped
    if _SHARED["vis"] is None:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        cfg = VisualizationMarkersCfg(prim_path="/World/Visuals/ink", markers={"dot": sim_utils.CuboidCfg(
            size=(0.0001, LINE_W, LINE_W),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.01, 0.01, 0.012), roughness=0.7))})
        _SHARED["vis"] = VisualizationMarkers(cfg)
    c = object_states(environment, ["container"])["container"]
    pose = np.concatenate([c.pos, c.quat_xyzw])
    moved = _SHARED["vis_pose"] is None or np.abs(pose - _SHARED["vis_pose"]).max() > 1e-5
    if INK.version == _SHARED["vis_version"] and not moved:
        return
    origin = np.asarray(jointkit._np(base.scene.env_origins)[0])
    pos = np.tile(origin + [0.0, 0.0, -5.0], (MAX_DOTS, 1))
    quat = np.tile([0.0, 0.0, 0.0, 1.0], (MAX_DOTS, 1))
    n = min(len(INK.local), MAX_DOTS)
    if n:
        r = c.rotation()
        pos[:n] = origin + c.pos + np.asarray(INK.local[:n]) @ r.T
        quat[:n] = _dot_quats(r, np.asarray(INK.facet[:n], float))
    dev = base.device
    _SHARED["vis"].visualize(translations=torch.tensor(pos, dtype=torch.float32, device=dev),
                             orientations=torch.tensor(quat, dtype=torch.float32, device=dev),
                             environment_ids=torch.zeros(MAX_DOTS, dtype=torch.int32, device=dev))
    _SHARED["vis_version"], _SHARED["vis_pose"] = INK.version, pose


def _hands(environment) -> None:
    """Every control step: where the two grippers' TCPs are (to tell whether the marker was let go)."""
    base = environment.unwrapped
    origin = jointkit._np(base.scene.env_origins)[0]
    for hand in ("left", "right"):
        robot = base.scene[f"{hand}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = jointkit._np(robot.data.body_link_pose_w)[0, i]
        r = jointkit.quat_to_matrix(pose[3:7])
        _SHARED["tcp"][hand] = pose[:3] - origin + r @ np.array([0.0, 0.0, 0.134])


def _step(environment) -> None:
    _SHARED["steps"] += 1
    _hands(environment)
    _draw(environment)
    if _DUMP and _SHARED["steps"] % 20 == 0:
        np.save(_DUMP, np.array(INK.sz, float).reshape(-1, 2)[np.array(INK.label, bool)])
    if _DEBUG and _SHARED["steps"] % 10 == 0:
        from yam_arena.scene import object_states

        s = object_states(environment, ["container", "marker"])
        p = s["container"].to_local(_tip_world(s["marker"]))
        d, _ = surface_distance(p)
        print(f"[label] t {_SHARED['steps'] / 20:6.2f} tip {np.round(p * 1000, 1)} d {d * 1000:6.2f} mm "
              f"dots {len(INK.local)} on label {sum(INK.label)} container {np.round(s['container'].pos * 1000, 1)} "
              f"tcp R {np.round(_SHARED['tcp'].get('right', np.zeros(3)) * 1000, 1)}", flush=True)


if not _SHARED.get("hooks"):
    on_physics(_physics)
    on_step(_step)
    on_reset(_reset_state)
    _SHARED["hooks"] = True

# -- the characters: stroke skeletons in a cell of height 1 (x right, y up), written in this order ------------------
GAP = 0.12                     # space between characters, in character heights


def _arc(cx, cy, rx, ry, a0, a1, n=16):
    return [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, n)]


def _glyphs():
    w = 0.55
    g = {
        "0": (w, [_arc(w / 2, 0.5, w / 2, 0.5, 90, 450, 33)]),
        "1": (0.30, [[(0.15, 1.0), (0.15, 0.0)]]),
        "2": (w, [_arc(w / 2, 0.72, w / 2, 0.28, 160, -35) + [(0.0, 0.0), (w, 0.0)]]),
        "3": (w, [_arc(w / 2, 0.75, w / 2, 0.25, 150, -90) + _arc(w / 2, 0.25, w / 2, 0.25, 90, -150)]),
        "4": (w, [[(0.75 * w, 1.0), (0.0, 0.32), (w, 0.32)], [(0.75 * w, 1.0), (0.75 * w, 0.0)]]),
        "5": (w, [[(w, 1.0), (0.08 * w, 1.0), (0.04 * w, 0.56)] + _arc(w / 2, 0.30, w / 2, 0.30, 120, -150)]),
        "6": (w, [[(0.85 * w, 1.0), (0.2 * w, 0.62)] + _arc(w / 2, 0.30, w / 2, 0.30, 180, 540, 25)]),
        "7": (w, [[(0.0, 1.0), (w, 1.0), (0.3 * w, 0.0)]]),
        "8": (w, [_arc(w / 2, 0.75, 0.42 * w, 0.25, -90, 270, 25) + _arc(w / 2, 0.27, w / 2, 0.27, 90, -270, 25)]),
        "9": (w, [_arc(w / 2, 0.72, w / 2, 0.28, 0, 360, 25) + [(w, 0.4), (0.6 * w, 0.0)]]),
        "/": (0.30, [[(0.30, 1.0), (0.0, 0.0)]]),
        "-": (0.35, [[(0.0, 0.5), (0.35, 0.5)]]),
    }
    return {c: (wd, [np.array(s, float) for s in strokes]) for c, (wd, strokes) in g.items()}


GLYPHS = _glyphs()
H_NOM = 30.0                   # mm: the demo writes 30 mm tall characters
Z_MID = 1000 * (TAPE_Z0 + TAPE_Z1) / 2


def layout(text: str, h: float):
    """[(char, left s, width)] in mm for ``text`` written ``h`` mm tall starting at s = 0."""
    out, s = [], 0.0
    for ch in text:
        wd = GLYPHS[ch][0] * h
        out.append((ch, s, wd))
        s += wd + GAP * h
    return out


def text_width(text: str, h: float) -> float:
    lay = layout(text, h)
    return lay[-1][1] + lay[-1][2]


def resample(poly: np.ndarray, step: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(math.ceil(t[-1] / step)) + 1)
    u = np.linspace(0.0, t[-1], n)
    return np.column_stack([np.interp(u, t, poly[:, 0]), np.interp(u, t, poly[:, 1])])


def char_points(ch: str, h: float, step: float = 0.5) -> np.ndarray:
    """Template points (mm) of one character ``h`` mm tall, its cell's bottom-left at (0, 0)."""
    return np.concatenate([resample(s * h, step) for s in GLYPHS[ch][1]])


# -- scoring --------------------------------------------------------------------------------------------------------
COVER = 2.5                    # mm: a template point is covered if ink lies within this distance
NEAR = 3.0                     # mm: ink farther than this from the template is stray
FULL_COVER = 0.90              # coverage for full credit for a character (linear from 0.40)
ZERO_COVER = 0.40
STRAY_CHAR = 0.25              # more stray ink than this in a character's cell: that character scores 0
STRAY_ALL = 0.20               # more stray ink than this on the whole label: the text scores 0
OFF_LABEL_MAX = 20.0           # mm of line off the label (on the container): score 0
HEIGHTS = (24.0, 27.0, 30.0, 33.0, 36.0)
SHIFT = 4                      # mm: each character may sit this far from its place in the fitted line
MOVED, TIPPED = 0.03, math.radians(20)
GRID = 0.5
S_RANGE, Z_RANGE = (-70.0, 70.0), (1000 * TAPE_Z0 - 6, 1000 * TAPE_Z1 + 6)


def _distance_grid(sz: np.ndarray) -> np.ndarray:
    from scipy.ndimage import distance_transform_edt

    ns, nz = int((S_RANGE[1] - S_RANGE[0]) / GRID) + 1, int((Z_RANGE[1] - Z_RANGE[0]) / GRID) + 1
    occ = np.zeros((ns, nz), bool)
    if len(sz):
        i = np.clip(np.round((sz[:, 0] - S_RANGE[0]) / GRID).astype(int), 0, ns - 1)
        j = np.clip(np.round((sz[:, 1] - Z_RANGE[0]) / GRID).astype(int), 0, nz - 1)
        occ[i, j] = True
    if not occ.any():
        return np.full((ns, nz), np.inf)
    return distance_transform_edt(~occ) * GRID


def _lookup(dist: np.ndarray, pts: np.ndarray) -> np.ndarray:
    i = np.round((pts[..., 0] - S_RANGE[0]) / GRID).astype(int)
    j = np.round((pts[..., 1] - Z_RANGE[0]) / GRID).astype(int)
    ok = (i >= 0) & (i < dist.shape[0]) & (j >= 0) & (j < dist.shape[1])
    out = np.full(pts.shape[:-1], np.inf)
    out[ok] = dist[i[ok], j[ok]]
    return out


def inked(pts: np.ndarray) -> np.ndarray:
    """The inked 0.5 mm cells (their centres) under the dots: what the label looks like, however often a line was
    gone over."""
    pts = np.asarray(pts, float).reshape(-1, np.shape(pts)[-1] if np.ndim(pts) > 1 else 2)
    return np.unique(np.round(pts / GRID), axis=0) * GRID if len(pts) else pts


def read_char(ink: np.ndarray, dist: np.ndarray, s: float, z0: float, h: float) -> str | None:
    """The character the ink in one cell reads as: the glyph (of all of GLYPHS, each tried within +-SHIFT mm of the
    cell's place) whose strokes the ink covers best while staying close to them (min of the two fractions)."""
    from scipy.spatial import cKDTree

    if len(ink) < 5:
        return None
    shifts = np.stack(np.meshgrid(np.arange(-SHIFT, SHIFT + 1.0), np.arange(-SHIFT, SHIFT + 1.0), indexing="ij"),
                      -1).reshape(-1, 2)
    best, out = -1.0, None
    for ch in GLYPHS:
        pts = char_points(ch, h, 1.0) + [s, z0]
        cov = (_lookup(dist, pts[None] + shifts[:, None]) <= COVER).mean(1)
        i = int(np.argmax(cov))
        near = float((cKDTree(pts + shifts[i]).query(ink)[0] <= NEAR).mean())
        m = min(float(cov[i]), near)
        if m > best + 1e-9:
            best, out = m, ch
    return out


def read_label(sz: np.ndarray, text: str = TEXT) -> dict:
    """Score the ink on the label (``sz``: (N, 2) label coordinates in mm) as ``text``: fit the text's line (height,
    place), then each character (+-SHIFT mm), and give each character credit for how much of its strokes the ink
    covers, unless there is too much stray ink."""
    from scipy.spatial import cKDTree

    res = {"chars": [0.0] * len(text), "cover": [0.0] * len(text), "stray": 1.0, "h": None, "text": 0.0}
    sz = inked(sz)
    if len(sz) < 10:
        return res
    dist = _distance_grid(sz)
    best = (-1.0, None)
    for h in HEIGHTS:
        lay = layout(text, h)
        pts = np.concatenate([char_points(c, h, 1.0) + [s, 0.0] for c, s, _ in lay])
        width = text_width(text, h)
        s0 = np.arange(S_RANGE[0], S_RANGE[1] - width + 1e-9, 1.0)
        z0 = np.arange(Z_RANGE[0], Z_RANGE[1] - h + 1e-9, 1.0)
        if not len(s0) or not len(z0):
            continue
        off = np.stack(np.meshgrid(s0, z0, indexing="ij"), -1).reshape(-1, 2)
        cov = (_lookup(dist, pts[None] + off[:, None]) <= COVER).mean(1)
        i = int(np.argmax(cov))
        if cov[i] > best[0]:
            best = (float(cov[i]), (h, off[i]))
    if best[1] is None:
        return res
    h, (s0, z0) = best[1]
    res["h"] = h
    templ, cells = [], []
    shifts = np.stack(np.meshgrid(np.arange(-SHIFT, SHIFT + 1.0), np.arange(-SHIFT, SHIFT + 1.0), indexing="ij"),
                      -1).reshape(-1, 2)
    for n, (ch, s, wd) in enumerate(layout(text, h)):
        pts = char_points(ch, h) + [s0 + s, z0]
        cov = (_lookup(dist, pts[None] + shifts[:, None]) <= COVER).mean(1)
        i = int(np.argmax(cov))
        pts = pts + shifts[i]
        templ.append(pts)
        cells.append((s0 + s + shifts[i][0] - GAP * h / 2, s0 + s + shifts[i][0] + wd + GAP * h / 2))
        res["cover"][n] = float(cov[i])
    far = cKDTree(np.concatenate(templ)).query(sz)[0] > NEAR
    res["stray"] = float(far.mean())
    for n, (lo, hi) in enumerate(cells):
        inside = (sz[:, 0] >= lo) & (sz[:, 0] < hi)
        stray = float(far[inside].mean()) if inside.any() else 0.0
        c = (res["cover"][n] - ZERO_COVER) / (FULL_COVER - ZERO_COVER)
        ok = stray <= STRAY_CHAR and read_char(sz[inside], dist, lo + GAP * h / 2, z0, h) == text[n]
        res["chars"][n] = float(np.clip(c, 0.0, 1.0)) if ok else 0.0
    res["text"] = 0.0 if res["stray"] > STRAY_ALL else float(np.mean(res["chars"]))
    return res


def _text_result() -> dict:
    """read_label of the ink so far, recomputed when the ink changed and the pen is up (or every 2 s)."""
    cache = _SHARED["cache"]
    stale = cache is None or cache[0] != INK.version
    if stale and (not _SHARED["contact"] or _SHARED["steps"] - _SHARED["last_refresh"] >= 40):
        sz = np.array([p for p, lab in zip(INK.sz, INK.label) if lab], float).reshape(-1, 2)
        _SHARED["cache"] = cache = (INK.version, read_label(sz))
        _SHARED["last_refresh"] = _SHARED["steps"]
    return cache[1] if cache else read_label(np.zeros((0, 2)))


def marker_down(marker) -> bool:
    """The marker lies on the table (lowest point within 3 mm of the top, barrel within 10 degrees of level), at
    rest, with both grippers at least 5 cm from it."""
    axis = marker.rotation()[:, 0]
    level = abs(axis[2]) < math.sin(math.radians(10))
    rest = np.linalg.norm(marker.lin_vel) < 0.02
    free = all(np.linalg.norm(t - marker.pos) > 0.05 for t in _SHARED["tcp"].values())
    return bool(marker.low[2] < TOP + 0.003 and level and rest and free)


def _update(objects) -> float:
    c, m = objects["container"], objects["marker"]
    if _SHARED["c0"] is None:
        _SHARED["c0"] = np.array(c.pos[:2], float)
    _SHARED["tipped"] |= bool(c.tilt() > TIPPED)
    _SHARED["moved"] |= bool(np.linalg.norm(c.pos[:2] - _SHARED["c0"]) > MOVED)
    if _SHARED["tipped"] or _SHARED["moved"]:
        return 0.0
    off = [p for p, lab in zip(INK.local, INK.label) if not lab]
    if len(inked(np.array(off) * 1000)) * GRID > OFF_LABEL_MAX:   # inked 0.5 mm cells off the tape, as mm of line
        return 0.0
    text = _text_result()["text"]
    return 0.8 * text + (0.2 if text >= 1.0 and marker_down(m) else 0.0)


def success(objects) -> bool:
    """Every character legible and the marker put down; held 1 s."""
    return _update(objects) >= 1.0


def score(objects) -> float:
    """0 if the container was tipped over (20 degrees) or pushed more than 3 cm off its spot, or if more than 20 mm of
    line went off the label. Otherwise 0.8 x the mean character credit (each 0 to 1 by how much of its strokes the ink
    covers; 0 with too much stray ink), plus 0.2 when every character is fully legible and the marker lies on the
    table, let go."""
    return _update(objects)


# -- scripted demo (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN, CLOSED = -0.04695, 0.0
GRIP_DZ = 0.0105              # the marker's axis this far under the TCP when picked up from the table
PRESS = float(os.environ.get("DEMO_PRESS", "0.003"))   # the tip is commanded this far into the surface
SAG = float(os.environ.get("DEMO_SAG", "0.0"))         # added to the commanded TCP height while writing
STEP_MM, STEP_S = 2.5, 0.2    # writing: a waypoint every 2.5 mm, 0.2 s each (12.5 mm/s)
LIFT = 0.015                  # pen up: 15 mm off the surface


def surface_point(s_mm: float, z_mm: float, out: float = 0.0):
    """World point on the side surface at label coordinates (s, z) mm, ``out`` m outside it; and the azimuth."""
    z = z_mm / 1000
    phi = PHI0 + s_mm / 1000 / apothem(z)
    k = int(round((phi % (2 * math.pi)) / DPHI)) % NF
    proud = TAPE_T if k in TAPE_KS else 0.0
    r = (apothem(z) + proud) / math.cos(phi - facet_phi(k)) + out
    return np.array([C_XY[0] + r * math.cos(phi), C_XY[1] + r * math.sin(phi), TOP + z]), phi


def pen_pose(s_mm: float, z_mm: float, out: float):
    """TCP and yaw holding the marker's tip at (s, z), ``out`` outside the surface, the marker pointing at the
    container's axis."""
    p, phi = surface_point(s_mm, z_mm, out)
    d = -np.array([math.cos(phi), math.sin(phi), 0.0])
    return p - TIP_X * d + [0.0, 0.0, GRIP_DZ + SAG], phi - math.pi


def _write(text: str, h: float = H_NOM):
    """Waypoints writing ``text`` h mm tall, centred on the label."""
    s0, z0 = -text_width(text, h) / 2, Z_MID - h / 2
    plan = []
    for ch, s, _ in layout(text, h):
        for stroke in GLYPHS[ch][1]:
            pts = resample(stroke * h, STEP_MM) + [s0 + s, z0]
            tcp, yaw = pen_pose(*pts[0], LIFT)
            plan += [(1.2, tcp, yaw, CLOSED, f"to '{ch}'"), ]
            for k, (ps, pz) in enumerate(pts):
                tcp, yaw = pen_pose(ps, pz, -PRESS)
                plan.append((0.8 if k == 0 else STEP_S, tcp, yaw, CLOSED, f"write '{ch}'"))
            tcp, yaw = pen_pose(*pts[-1], LIFT)
            plan.append((0.8, tcp, yaw, CLOSED, "pen up"))
    return plan


def _right():
    g = np.array([M0[0], M0[1], TOP + M_R + GRIP_DZ])
    plan = [(2.0, g + [0, 0, 0.07], 0.0, OPEN, "over the marker"), (1.5, g, 0.0, OPEN, "down"),
            (0.8, g, 0.0, CLOSED, "grip the marker"), (1.5, g + [0, 0, 0.09], 0.0, CLOSED, "lift")]
    first = _write(TEXT)
    tcp, yaw = first[0][1], first[0][2]
    plan += [(2.5, tcp + [-0.04, 0, 0.02], yaw, CLOSED, "carry to the label")]
    plan += first
    last = plan[-1]
    plan += [(1.0, last[1] + [-0.04, 0, 0.02], last[2], CLOSED, "away from the label"),
             (2.5, g + [0, 0, 0.05], 0.0, CLOSED, "carry back"), (1.5, g + [0, 0, 0.001], 0.0, CLOSED, "put it down"),
             (0.6, g + [0, 0, 0.001], 0.0, OPEN, "let go"), (1.2, g + [0, 0, 0.08], 0.0, OPEN, "up"),
             (1.5, (0.20, -0.28, TOP + 0.18), 0.0, OPEN, "away")]
    return plan


BRACE_PHI = PHI0 - math.pi    # the left hand braces the container's back, opposite the label
BRACE_Z = 0.110               # TCP height above the table (the closed pads touch 97 to 110 mm up)
PAD_HALF = 0.008              # closed fingers: TCP to the pads' outer face across the closing axis (measured, camlock)


def _left(t_write_end: float):
    """Closed fingers against the container's back, across from the label, pressing 3 mm in."""
    n = np.array([math.cos(BRACE_PHI), math.sin(BRACE_PHI), 0.0])
    r = apothem(BRACE_Z - 0.006) + PAD_HALF
    on = np.array([C_XY[0], C_XY[1], TOP + BRACE_Z]) + (r - 0.003) * n
    off = on + 0.03 * n
    yaw = BRACE_PHI - math.pi / 2         # fingers close along n
    plan = [(2.5, off + [0, 0, 0.03], yaw, CLOSED, "behind the container"), (1.0, off, yaw, CLOSED, "down"),
            (1.0, on, yaw, CLOSED, "hold the container")]
    t = sum(w[0] for w in plan)
    plan += [(max(0.5, t_write_end - t), on, yaw, CLOSED, "hold"), (1.0, off, yaw, CLOSED, "let go of it"),
             (1.5, (0.20, 0.28, TOP + 0.18), 0.0, OPEN, "away")]
    return plan


def plan() -> dict[str, list]:
    right = _right()
    t, t_end = 0.0, 0.0
    for w in right:
        t += w[0]
        if w[4] == "away from the label":
            t_end = t
    return {"left": _left(t_end), "right": right}


def demo():
    """The left hand presses the lid down (holds the container); the right hand picks the marker up by the barrel,
    writes the text stroke by stroke with the tip pressed onto the tape, turning the marker to keep it pointing at
    the container's axis, then lays the marker back where it was and lets go."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(float(v) for v in p), f, label, y if np.ndim(y) == 2 else float(y)) for d, p, y, f, label in wps]
            for arm, wps in plan().items()}
