"""On the station table:

- a 32 oz round clear plastic deli container (Bare by Solo DM32R-0090, recycled PET: 88.1 mm across the bottom,
  117.1 mm across the top, 145 mm tall) holding about 470 ml of water, its recessed lid on and taped shut, 0.5 kg in
  all, standing upright in front of the robot, centre 30 cm forward on the midline;
- a strip of beige masking tape (Scotch 2050, 48 mm wide, about 10.5 cm long) stuck around the container's side,
  facing the right arm, from 51 to 99 mm above the table: the label. It is blank;
- a black Sharpie Fine Point permanent marker (cap already off, 123 mm long, 11 mm barrel, bullet tip) lying on a
  sheet of copy paper (US letter, 279 x 216 mm) on the table to the right of the container, tip pointing forward.
  The grippers' finger pads are wrapped in masking tape (not modelled).

The robot must write the use-by date given in the instruction on the label with the marker, so that a person can
read it, and put the marker down. The marker leaves ink only where its nib presses on the container with a normal
force of at least 0.5 N (PhysX contact force).

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
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

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
MASS = 0.5                                                    # container, lid and water: weighed to 500 g
TAPE_W = 0.048                                                # Scotch 2050, 48 mm
TAPE_Z0, TAPE_Z1 = 0.051, 0.099                               # tape band, above the table
TAPE_T = 0.0001                                               # tape stands this proud of the wall (about 0.1 mm)
PHI0 = 1.25 * math.pi                                         # the label faces the right arm (front right)
TAPE_FACETS = 13                                              # 13 facets: about 10.5 cm of tape around the side
FOOD, HEADSPACE, LIDC, TAPEC = (0.70, 0.79, 0.85), (0.80, 0.84, 0.87), (0.88, 0.90, 0.91), (0.93, 0.87, 0.70)
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
              boxes=M_BOXES, contact_report=True)   # PhysX reports its contacts with the container

# a sheet of copy paper under the marker (US letter, 279 x 216 mm, 0.1 mm, long side along x), fixed: keeps ink off
# the table where the marker is picked up and laid down and along the way to the container
PAPER = Box("paper", pos=(float(M0[0]), float(M0[1]), TOP + 0.00005), size=(0.279, 0.216, 0.0001),
            color=(0.95, 0.95, 0.93))

OBJECTS = [CONTAINER, MARKER, PAPER]

# -- ink (pure Python; the physics hook feeds it) -------------------------------------------------------------------
# Ink needs the nib pressed on the container: PhysX's contact points between the marker and the container (a rigid
# contact view) that lie on the nib (within NIB of the tip) must carry, together, a normal force of at least F_MIN.
# The dot goes where that contact is (force-weighted mean of the nib's contact points). Dots are kept in the
# container's frame (they move with it) and drawn as thin dark 1 mm patches on the surface (visual only, no collision).
# F_MIN: Schomaker & Plamondon (1990), Biol. Cybern. 63:277-289, Table 2: mean axial pen force in cursive handwriting
# 109.7 g (1.08 N), variance 806 g^2 (SD 28 g), so mean - 2 SD = 53 g (0.52 N); the means of all six writing and
# drawing conditions are 84 to 115 g (0.82 to 1.13 N). Below 0.5 N no ink is laid. There is no upper limit: a harder
# press still inks (the study gives no force above which a pen stops writing); F_HIGH (mean + 2 SD) is only reported.
F_MIN = 0.5             # N: least normal force at the nib that lays ink
F_HIGH = 1.64           # N: upper end of normal handwriting force (reported only, not a limit)
NIB = 0.003             # m: contact points this close to the tip are on the nib
MAX_CONTACTS = 4096     # contact points PhysX may report for the marker per step (its ~50 boxes make many)
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

    def tip(self, p, force: float):
        """The nib's contact with the container this sample: ``p`` the contact point (container frame), ``force`` the
        normal force on the nib (N, from PhysX). Lays ink if the force is at least F_MIN."""
        if p is None or force < F_MIN:
            self.prev = None
            return False
        d, k = surface_distance(p)
        if d > 0.002 or d < -0.004:     # not on the side wall (the lid, the bottom)
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
                   vis=None, vis_version=-1, vis_pose=None, last_refresh=0, steps=0, cview=None, fsum=0.0, psum=None,
                   force=0.0, inked_forces=[])
INK: Ink = _SHARED["ink"]


def _reset_state(environment=None) -> None:
    INK.reset()
    _SHARED.update(tick=0, contact=False, tcp={}, c0=None, tipped=False, moved=False, cache=None, vis_version=-1,
                   vis_pose=None, last_refresh=0, steps=0, fsum=0.0, psum=None, force=0.0, inked_forces=[])


def _tip_world(marker) -> np.ndarray:
    return marker.to_world((TIP_X, 0.0, 0.0))


def _host(x) -> np.ndarray:
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x.numpy() if hasattr(x, "numpy") else x)


def nib_contact(forces, points, tip) -> tuple[float, np.ndarray | None]:
    """(total normal force on the nib (N), force-weighted mean of its contact points) from PhysX contact points
    (``forces`` (n,), ``points`` (n, 3), same frame as ``tip``)."""
    forces, points = np.abs(np.asarray(forces, float).reshape(-1)), np.asarray(points, float).reshape(-1, 3)
    on = np.linalg.norm(points - tip, axis=1) <= NIB
    f = float(forces[on].sum())
    return f, (forces[on] @ points[on] / f if f > 0 else None)


def _nib_force(environment, marker) -> tuple[float, np.ndarray | None]:
    """The nib's normal contact force against the container in the last physics step, read from a PhysX rigid
    contact view (marker vs container), and where it acts (env frame)."""
    from yam_arena.constants import PHYSICS_DT

    if _SHARED["cview"] is None:
        _SHARED["cview"] = jointkit.contact_view(environment, "marker", ["container"], max_contacts=MAX_CONTACTS)
    f, p, _, _, count, start = _SHARED["cview"].get_contact_data(PHYSICS_DT)
    n, i0 = int(_host(count).reshape(-1)[0]), int(_host(start).reshape(-1)[0])
    if n == 0:
        return 0.0, None
    origin = jointkit._np(environment.unwrapped.scene.env_origins)[0]
    pts = _host(p).reshape(-1, 3)[i0:i0 + n] - origin
    return nib_contact(_host(f).reshape(-1)[i0:i0 + n], pts, _tip_world(marker))


def _physics(environment) -> None:
    """Every physics step: the nib's contact force; every 4th (60 Hz): ink, with the mean force of the last 4 steps
    and the last step's contact point."""
    from yam_arena.scene import object_states

    _SHARED["tick"] += 1
    s = object_states(environment, ["container", "marker"])
    f, p = _nib_force(environment, s["marker"])
    _SHARED["fsum"] += f
    if p is not None:
        _SHARED["psum"] = p
    if _SHARED["tick"] % 4:
        return
    force, p = _SHARED["fsum"] / 4, _SHARED["psum"]
    _SHARED["fsum"], _SHARED["psum"], _SHARED["force"] = 0.0, None, force
    _SHARED["contact"] = INK.tip(None if p is None else s["container"].to_local(p), force)
    if _SHARED["contact"]:
        _SHARED["inked_forces"].append(force)
    elif 0.0 < force < F_MIN:
        _SHARED["light"] = _SHARED.get("light", 0) + 1


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
        fi = np.array(_SHARED["inked_forces"]) if _SHARED["inked_forces"] else np.zeros(1)
        print(f"[label] t {_SHARED['steps'] / 20:6.2f} tip {np.round(p * 1000, 1)} d {d * 1000:6.2f} mm "
              f"F {_SHARED['force']:5.2f} N inked F p5/50/95 {np.percentile(fi, [5, 50, 95]).round(2)} max "
              f"{fi.max():.2f} n {len(fi)} light {_SHARED.get('light', 0)} "
              f"dots {len(INK.local)} on label {sum(INK.label)} container {np.round(s['container'].pos * 1000, 1)} "
              f"tcp R {np.round(_SHARED['tcp'].get('right', np.zeros(3)) * 1000, 1)}", flush=True)


if not _SHARED.get("hooks"):
    on_physics(_physics)
    on_step(_step)
    on_reset(_reset_state)
    _SHARED["hooks"] = True

# -- the characters (cell of height 1, x right, y up) ---------------------------------------------------------------
# Each character has one or more accepted forms. A form is a list of strokes that must all be drawn (the character's
# strokes, counted for partial credit: STROKES) and a list of optional extra strokes that may be there or not (a flag
# or a base on a "1", a bar across a "7"): they are never counted, they only keep that ink from counting as stray.
GAP = 0.12                     # space between characters, in character heights


def _arc(cx, cy, rx, ry, a0, a1, n=16):
    return [(cx + rx * math.cos(math.radians(a)), cy + ry * math.sin(math.radians(a)))
            for a in np.linspace(a0, a1, n)]


def _forms():
    w = 0.55
    one, slash = 0.30, 0.30
    stem1 = [(0.15, 1.0), (0.15, 0.0)]
    flag1, base1 = [(0.15, 1.0), (0.0, 0.80)], [(0.0, 0.0), (0.30, 0.0)]
    four_bar, four_stem = [(0.0, 0.32), (w, 0.32)], [(0.75 * w, 1.0), (0.75 * w, 0.0)]
    seven_top, seven_diag = [(0.0, 1.0), (w, 1.0)], [(w, 1.0), (0.3 * w, 0.0)]
    two_base, two_loop = [(0.0, 0.0), (w, 0.0)], _arc(0.12 * w, 0.09, 0.12 * w, 0.09, 0, 360, 13)
    two_round = _arc(w / 2, 0.72, w / 2, 0.28, 160, -35) + [(0.0, 0.0)]
    two_flat = [(0.05 * w, 1.0), (0.9 * w, 1.0), (0.0, 0.0)]                         # flat top, straight diagonal
    nine_loop, six_loop = _arc(w / 2, 0.72, w / 2, 0.28, 0, 360, 25), _arc(w / 2, 0.30, w / 2, 0.30, 180, 540, 25)
    f = {   # char: (width, [(strokes, optional strokes), ...]); the first form is the one the demo writes
        "0": (w, [([_arc(w / 2, 0.5, w / 2, 0.5, 90, 450, 33)], []),
                  ([_arc(w / 2, 0.5, w / 2, 0.5, 90, 450, 33)], [[(0.9 * w, 0.95), (0.1 * w, 0.05)]])]),  # slashed
        "1": (one, [([stem1], []), ([stem1], [flag1]), ([stem1], [base1]), ([stem1], [flag1, base1])]),
        "2": (w, [([two_round, two_base], []),
                  ([two_round, two_base], [two_loop]),                                  # looped at the bottom
                  ([two_flat, two_base], []),                                           # flat top ("Z")
                  ([two_flat, two_base], [two_loop])]),
        "3": (w, [([_arc(w / 2, 0.75, w / 2, 0.25, 150, -90), _arc(w / 2, 0.25, w / 2, 0.25, 90, -150)], [])]),
        "4": (w, [([[(0.75 * w, 1.0), (0.0, 0.32)], four_bar, four_stem], []),           # closed top
                  ([[(0.1 * w, 1.0), (0.0, 0.32)], four_bar, four_stem], [])]),          # open top
        "5": (w, [([[(w, 1.0), (0.08 * w, 1.0)], [(0.08 * w, 1.0), (0.04 * w, 0.56)],
                    [(0.04 * w, 0.56)] + _arc(w / 2, 0.30, w / 2, 0.30, 120, -150)], [])]),
        "6": (w, [([[(0.85 * w, 1.0), (0.2 * w, 0.62), (0.0, 0.30)], six_loop], []),
                  ([[(0.8 * w, 1.0), (0.0, 0.30)], six_loop], [])]),                    # straight stem
        "7": (w, [([seven_top, seven_diag], []),
                  ([seven_top, seven_diag], [[(0.3 * w, 0.5), (0.95 * w, 0.5)]])]),        # crossed
        "8": (w, [([_arc(w / 2, 0.75, 0.42 * w, 0.25, -90, 270, 25), _arc(w / 2, 0.27, w / 2, 0.27, 90, -270, 25)],
                   [])]),
        "9": (w, [([nine_loop, [(w, 0.72), (w, 0.4), (0.6 * w, 0.0)]], []),
                  ([nine_loop, [(w, 0.72), (w, 0.0)]], [])]),                           # straight stem
        "/": (slash, [([[(0.30, 1.0), (0.0, 0.0)]], [])]),
        "-": (0.35, [([[(0.0, 0.5), (0.35, 0.5)]], [])]),
    }
    return {c: (wd, [([np.array(s, float) for s in core], [np.array(s, float) for s in opt]) for core, opt in forms])
            for c, (wd, forms) in f.items()}


FORMS = _forms()
STROKES = {c: len(forms[0][0]) for c, (_, forms) in FORMS.items()}   # strokes per character (partial credit)


def _pen_paths(strokes):
    """The default form's strokes as pen paths: a stroke that starts where (within 0.05 of the height) the previous
    one ended is drawn without lifting the pen."""
    out = []
    for s in strokes:
        if out and np.linalg.norm(out[-1][-1] - s[0]) < 0.05:
            out[-1] = np.concatenate([out[-1], s])
        else:
            out.append(s)
    return out


GLYPHS = {c: (wd, _pen_paths(forms[0][0])) for c, (wd, forms) in FORMS.items()}   # what the demo writes
H_NOM = 30.0                   # mm: the demo writes 30 mm tall characters
Z_MID = 1000 * (TAPE_Z0 + TAPE_Z1) / 2


def layout(text: str, h: float, width: float = 1.0, gap: float = GAP):
    """[(char, left s, width)] in mm for ``text`` written ``h`` mm tall (characters ``width`` times as wide, ``gap``
    character heights apart) starting at s = 0."""
    out, s = [], 0.0
    for ch in text:
        wd = GLYPHS[ch][0] * h * width
        out.append((ch, s, wd))
        s += wd + gap * h
    return out


def text_width(text: str, h: float, width: float = 1.0, gap: float = GAP) -> float:
    lay = layout(text, h, width, gap)
    return lay[-1][1] + lay[-1][2]


def resample(poly: np.ndarray, step: float) -> np.ndarray:
    seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    t = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(math.ceil(t[-1] / step)) + 1)
    u = np.linspace(0.0, t[-1], n)
    return np.column_stack([np.interp(u, t, poly[:, 0]), np.interp(u, t, poly[:, 1])])


# Handwriting varies: a character may be written narrower or wider than its form (WIDTHS, about its middle), and the
# whole date may lean back or forward (SLANTS, degrees from upright, one slant for the whole date: a writer's slant
# is steady). Between the listed values the COVER tolerance takes up the rest (about +-0.12 in width and +-5 degrees
# of slant at 30 mm), so widths 0.7 to 1.4 and slants -25 to +25 degrees are read. A "1" and a "/" differ only in how
# much they lean, so neither is widened or narrowed (that would tilt them): the slant alone sets their lean.
WIDTHS = (1.0, 0.8, 1.25)            # the first is preferred on a tie
SLANTS = (0.0, -10.0, 10.0, -20.0, 20.0)
CHAR_WIDTHS = {"1": (1.0,), "/": (1.0,)}


def shaped(stroke: np.ndarray, wd: float, width: float = 1.0, slant: float = 0.0) -> np.ndarray:
    """A stroke (cell units, character ``wd`` wide) written ``width`` times as wide and leaning ``slant`` degrees
    forward (top to the right), both about the middle of the cell."""
    x = wd / 2 + (stroke[:, 0] - wd / 2) * width + math.tan(math.radians(slant)) * (stroke[:, 1] - 0.5)
    return np.column_stack([x, stroke[:, 1]])


def variant(ch: str, form: int, width: float = 1.0, slant: float = 0.0):
    """(strokes, optional strokes) of ``form`` of ``ch`` at ``width`` and ``slant`` (cell units)."""
    key = (ch, form, width, slant)
    if key not in _VARIANTS:
        wd, forms = FORMS[ch]
        core, opt = forms[form]
        _VARIANTS[key] = ([shaped(s, wd, width, slant) for s in core], [shaped(s, wd, width, slant) for s in opt])
    return _VARIANTS[key]


_VARIANTS: dict = {}


def char_points(ch: str, h: float, step: float = 0.5, form: int = 0, extras=None, width: float = 1.0,
                slant: float = 0.0) -> np.ndarray:
    """Points (mm) along one character ``h`` mm tall written in ``form`` (its strokes, plus the optional strokes
    listed in ``extras``, default all of the form's), its cell's bottom-left at (0, 0)."""
    core, opt = variant(ch, form, width, slant)
    opt = opt if extras is None else [opt[i] for i in extras]
    return np.concatenate([resample(s * h, step) for s in list(core) + list(opt)])


# -- scoring --------------------------------------------------------------------------------------------------------
COVER = 2.5                    # mm: a point of a stroke is drawn if ink lies within this distance of it
STROKE_DRAWN = 0.80            # a stroke counts as drawn when this share of its length is drawn
NEAR = 3.0                     # mm: ink farther than this from the character's strokes is stray
STRAY_CHAR = 0.25              # more stray ink than this in a character's share: scribbled over, that character 0
STRAY_ALL = 0.20               # more stray ink than this on the whole label: the text scores 0
OTHER_MARGIN = 0.05            # another character reads better when it explains this much more of the ink,
OTHER_COVER = 0.05             # or as much of it while this much more of its strokes' length is drawn
OTHER_MIN_COVER = 0.90         # another character is only read there when this much of its strokes' length is drawn
OTHER_TOL = 0.05               # an unfinished character (or one with clearly less drawn) loses to another that explains
                               # nearly as much (within this share) of the ink
OFF_LABEL_MAX = 20.0           # mm of line off the label (on the container): score 0
HEIGHTS = (24.0, 27.0, 30.0, 33.0, 36.0)
SHIFT = 4                      # mm: each character may sit this far above or below its place in the fitted line
GAPS = (GAP, 0.04, 0.25, 0.40)  # spacing between characters tried for the line, in character heights
LINE_REFINE = 24               # the best coarse line fits refined at 1 mm and ranked
LINE_TRIES = 3                 # the best line fits (each a different height or slant) are each read; the best counts
DRIFT = 15                     # mm: a character may sit this far left or right of its place in the fitted line,
SPACE_CHANGE = 0.35            # each space between neighbours differing from the line's by up to this many heights
SPACE_COST = 0.05              # per height of difference: of two equally good placements, the more even one is kept
_FINE = np.stack(np.meshgrid(np.arange(-2.0, 3.0), np.arange(-2.0, 3.0), indexing="ij"), -1).reshape(-1, 2)
MOVED, TIPPED = 0.03, math.radians(20)
GRID = 0.5
S_RANGE, Z_RANGE = (-70.0, 70.0), (1000 * TAPE_Z0 - 6, 1000 * TAPE_Z1 + 6)
_SHIFTS = np.stack(np.meshgrid(np.arange(-SHIFT, SHIFT + 1.0), np.arange(-SHIFT, SHIFT + 1.0), indexing="ij"),
                   -1).reshape(-1, 2)
_LOCAL = np.stack(np.meshgrid(np.arange(-2.0, 3.0), np.arange(-SHIFT, SHIFT + 1.0), indexing="ij"), -1).reshape(-1, 2)


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


def _best_place(dist, pts, base, shifts=_SHIFTS):
    """The shift of template points ``pts`` (placed at ``base``) that covers the most of them with ink, ties broken
    by the smallest mean distance to the ink; returns (coverage, shift)."""
    d = _lookup(dist, pts[None] + base + shifts[:, None])
    cov = (d <= COVER).mean(1) - 1e-4 * np.minimum(d, 10.0).mean(1)
    i = int(np.argmax(cov))
    return float((d[i] <= COVER).mean()), shifts[i]


def _form_fit(dist, ch, form, h, base, width=1.0, slant=0.0, shifts=_SHIFTS):
    """(strokes drawn, share of the strokes drawn, all the form's points (mm, placed), share of its strokes' length
    drawn) for one form (at ``width`` and ``slant``) at its best shift around ``base``."""
    core, opt = variant(ch, form, width, slant)
    strokes = [resample(s * h, 1.0) for s in core]
    cov, sh = _best_place(dist, np.concatenate(strokes), base, shifts)
    drawn = sum(float((_lookup(dist, s + base + sh) <= COVER).mean()) >= STROKE_DRAWN for s in strokes)
    pts = np.concatenate(strokes + [resample(s * h, 1.0) for s in opt]) + base + sh
    if opt:   # the optional strokes may sit a little differently (a longer flag, a lower bar)
        _, sh2 = _best_place(dist, np.concatenate([resample(s * h, 1.0) for s in opt]), base + sh,
                             _SHIFTS[np.abs(_SHIFTS).max(1) <= 2])
        pts = np.concatenate([np.concatenate(strokes) + base + sh] + [resample(s * h, 1.0) + base + sh + sh2
                                                                      for s in opt])
    return drawn, drawn / len(core), pts, cov


def _explained(ink: np.ndarray, pts: np.ndarray) -> float:
    from scipy.spatial import cKDTree

    return float((cKDTree(pts).query(ink)[0] <= NEAR).mean()) if len(ink) else 0.0


OTHER_SHIFT = 6                # mm: another character is looked for this far around a character's centre,
OTHER_H, OTHER_SLANT = 0.0, 0.0  # and this much taller or shorter (mm) and more or less slanted (degrees)
_OTHER_SHIFTS = np.stack(np.meshgrid(np.arange(-OTHER_SHIFT, OTHER_SHIFT + 1.0), np.arange(-SHIFT, SHIFT + 1.0),
                                     indexing="ij"), -1).reshape(-1, 2)


def read_as_other(sz, dist, h, centre, n, line_pts, target, complete=True, cover=1.0, slant=0.0, own=None):
    """Does the ink around character ``n`` (centre (s, z) mm) read better as another character? For every other
    character and form (any width; the line's height and slant, or OTHER_H mm taller or shorter, or OTHER_SLANT
    degrees more or less: what is written is looked for as it is, not only as the line was fitted) whose strokes are
    all drawn there (within OTHER_SHIFT mm) by character n's share of the ink (``dist``: its distance grid), compare
    how much of the ink in its area (and of character n's ``own`` ink) the intended line explains (``line_pts``: each
    character's points) with how much the line explains with character n swapped for it. Returns (char, explained by the swap, explained by the line) of the best swap that
    explains OTHER_MARGIN more, or nearly as much (within OTHER_TOL) when character n is not ``complete`` (an unfinished
    character that makes another, finished one reads as that one) or when OTHER_COVER more of the other character's
    strokes' length is drawn than of character n's (``cover``), else None."""
    from scipy.spatial import cKDTree

    rest = np.concatenate([p for i, p in enumerate(line_pts) if i != n] or [np.zeros((0, 2))])
    line = cKDTree(np.concatenate(line_pts))
    rest_tree = cKDTree(rest) if len(rest) else None
    best = None
    slants = list(dict.fromkeys(a for a in (slant, slant - OTHER_SLANT, slant + OTHER_SLANT)
                                if abs(a) <= max(map(abs, SLANTS))))
    heights = list(dict.fromkeys((h, h - OTHER_H, h + OTHER_H)))
    for ch, (wd, forms) in FORMS.items():
        if ch == target:
            continue
        for hh, a, (core, opt) in ((hh, a, variant(ch, f, k, a)) for hh in heights
                                   for a in slants for f in range(len(forms)) for k in CHAR_WIDTHS.get(ch, WIDTHS)):
            base = centre - [wd * hh / 2, hh / 2]
            strokes = [resample(s * hh, 1.0) for s in core]
            cov, sh = _best_place(dist, np.concatenate(strokes), base, _OTHER_SHIFTS)
            if cov < OTHER_MIN_COVER or not all(float((_lookup(dist, s + base + sh) <= COVER).mean()) >= STROKE_DRAWN
                                                for s in strokes):
                continue
            pts = np.concatenate(strokes + [resample(s * hh, 1.0) for s in opt]) + base + sh
            lo, hi = pts.min(0) - NEAR, pts.max(0) + NEAR
            ink = sz[np.all((sz >= lo) & (sz <= hi), axis=1)]
            if own is not None and len(own):   # and all of character n's own ink: the swap has to account for it
                ink = np.unique(np.concatenate([ink, own]), axis=0)
            if len(ink) < 5:
                continue
            e_line = float((line.query(ink)[0] <= NEAR).mean())
            d_swap = cKDTree(pts).query(ink)[0]
            if rest_tree is not None:
                d_swap = np.minimum(d_swap, rest_tree.query(ink)[0])
            e_swap = float((d_swap <= NEAR).mean())
            margin = OTHER_MARGIN if complete and cov < cover + OTHER_COVER else -OTHER_TOL
            if e_swap > e_line + margin and (best is None or e_swap - e_line > best[1] - best[2]):
                best = (ch, e_swap, e_line)
    return best


def _line_fits(sz, dist, text):
    """The best LINE_TRIES fits of the text's line to the ink, each a different (height, slant): (quality, (h, slant,
    width, spacing, (s0, z0))), best first. Every height, slant, width and spacing is placed on a 2 mm grid, ranked by coverage + the
    share of the ink it explains (a line written too small fits inside larger writing: it is covered, but leaves ink
    unexplained); the best LINE_REFINE are placed again at 1 mm and ranked the same way."""
    coarse = []
    for sl, k, g, h in ((sl, k, g, h) for sl in SLANTS for k in WIDTHS for g in GAPS for h in HEIGHTS):
        pts = np.concatenate([char_points(c, h, 1.0, width=1.0 if c in CHAR_WIDTHS else k, slant=sl)
                              + [s + (wd - FORMS[c][0] * h) / 2, 0.0] for c, s, wd in layout(text, h, k, g)])
        width = text_width(text, h, k, g)
        s0 = np.arange(S_RANGE[0], S_RANGE[1] - width + 1e-9, 2.0)
        z0 = np.arange(Z_RANGE[0], Z_RANGE[1] - h + 1e-9, 2.0)
        if not len(s0) or not len(z0):
            continue
        off = np.stack(np.meshgrid(s0, z0, indexing="ij"), -1).reshape(-1, 2)
        cov, sh = _best_place(dist, pts, np.zeros(2), off)
        coarse.append((cov + _explained(sz, pts + sh), len(coarse), (sl, k, g, h), pts, sh))
    refined = []
    for _, i, (sl, k, g, h), pts, sh in sorted(coarse, key=lambda c: (-c[0], c[1]))[:LINE_REFINE]:
        cov, fine = _best_place(dist, pts, sh, _FINE)
        refined.append((cov + _explained(sz, pts + sh + fine), i, (h, sl, k, g, sh + fine)))
    out, seen = [], set()
    for q, _, line in sorted(refined, key=lambda r: (-r[0], r[1])):
        if line[:2] not in seen and len(out) < LINE_TRIES:
            seen.add(line[:2])
            out.append((q, line))
    return out


def _place(dist, places, h, sl, z0, variants):
    """Where each character sits along the line (elastic, for uneven spacing): an offset of up to DRIFT mm from its
    place in the fitted line, each space between neighbours changing by at most SPACE_CHANGE heights; the offsets that
    cover the most of the characters' strokes (each at its best form, width and height within +-SHIFT mm), less
    SPACE_COST per height of change in spacing (dynamic programming over 1 mm steps)."""
    dxs = np.arange(-DRIFT, DRIFT + 1.0)
    offs = np.stack(np.meshgrid(dxs, np.arange(-SHIFT, SHIFT + 1.0), indexing="ij"), -1).reshape(-1, 2)
    covs = []
    for ch, s, _ in places:
        best = np.zeros(len(dxs))
        for f, k in variants[ch]:
            pts = np.concatenate([resample(st * h, 1.0) for st in variant(ch, f, k, sl)[0]])
            d = _lookup(dist, pts[None] + [s, z0] + offs[:, None])
            best = np.maximum(best, (d <= COVER).mean(1).reshape(len(dxs), -1).max(1))
        covs.append(best - 1e-4 * np.abs(dxs))
    change = dxs[None, :] - dxs[:, None]                       # [previous, next]: change of the space between them
    step = np.where(np.abs(change) <= SPACE_CHANGE * h, -SPACE_COST * np.abs(change) / h, -np.inf)
    score, back = covs[0], []
    for cov in covs[1:]:
        tot = score[:, None] + step
        arg = tot.argmax(0)
        back.append(arg)
        score = tot[arg, np.arange(len(dxs))] + cov
    path = [int(score.argmax())]
    for arg in reversed(back):
        path.append(int(arg[path[-1]]))
    return [float(dxs[j]) for j in reversed(path)]


def read_label(sz: np.ndarray, text: str = TEXT) -> dict:
    """Score the ink on the label (``sz``: (N, 2) label coordinates in mm) as ``text``, as a person reads it: the
    reading (of the best LINE_TRIES line fits) that gives the most credit counts, except that a character that a
    reading fitting the ink at least as well sees as another character gets nothing.

    1. Fit the text's line (height 24 to 36 mm, slant, character width, spacing, place on the tape) to the ink.
    2. Place the characters along it (``_place``: uneven spacing). Each character: its accepted forms, at each width
       (WIDTHS) and the line's slant, within 2 mm (left/right) and +-SHIFT mm (up/down) of that place; count its
       strokes drawn (STROKE_DRAWN of the length within COVER mm of ink); keep the form with the most strokes drawn
       (then the one explaining the most of the character's ink).
    3. Share out the ink: each inked cell goes to the nearest character; farther than NEAR mm from every character's
       strokes it is stray.
    4. A character's credit = strokes drawn / its strokes (STROKES), but 0 if more than STRAY_CHAR of its share of
       the ink is farther than NEAR from its strokes (scribbled over, extra marks), or if the ink reads as another
       character (``read_as_other``).
    5. The text = mean character credit, 0 if more than STRAY_ALL of all the ink is stray."""
    n_ch = len(text)
    res = {"chars": [0.0] * n_ch, "strokes": [0] * n_ch, "form": [0] * n_ch, "reads": [None] * n_ch,
           "width": [1.0] * n_ch, "stray": 1.0, "h": None, "slant": 0.0, "text": 0.0}
    sz = inked(sz)
    if len(sz) < 10:
        return res
    dist = _distance_grid(sz)
    best, q_best, reads = res, 0.0, []
    for q, line in _line_fits(sz, dist, text):
        r = _read_line(sz, dist, text, *line)
        reads.append((q, {n for n, c in enumerate(r["reads"]) if c is not None and c != text[n]}))
        if (r["text"], -r["stray"]) > (best["text"], -best["stray"]) or best["h"] is None:
            best, q_best = r, q
    # a character that another reading, fitting the ink at least as well, reads as another character: no credit
    changed = False
    for q, other in reads:
        for n in other:
            if q >= q_best - 1e-9 and best["reads"][n] == text[n]:
                best["chars"][n], best["reads"][n], changed = 0.0, "?", True
    if changed and best["stray"] <= STRAY_ALL:
        best["text"] = float(np.mean(best["chars"]))
    return best


def _read_line(sz, dist, text, h, sl, k_line, g_line, s0z0) -> dict:
    """Steps 2 to 5 of ``read_label`` for one fitted line."""
    from scipy.spatial import cKDTree

    n_ch = len(text)
    res = {"chars": [0.0] * n_ch, "strokes": [0] * n_ch, "form": [0] * n_ch, "reads": [None] * n_ch,
           "width": [1.0] * n_ch, "stray": 1.0, "h": h, "slant": sl, "text": 0.0}
    s0, z0 = s0z0
    variants = {ch: [(f, k) for f in range(len(FORMS[ch][1])) for k in CHAR_WIDTHS.get(ch, WIDTHS)]
                for ch in set(text)}
    # each character's cell (its form's width, unscaled) centred on its place in the fitted line, then placed
    places = [(ch, s0 + s + (wd - FORMS[ch][0] * h) / 2, wd) for ch, s, wd in layout(text, h, k_line, g_line)]
    dx = _place(dist, places, h, sl, z0, variants)
    places = [(ch, s + d, wd) for (ch, s, wd), d in zip(places, dx)]
    fits = []
    for ch, s, wd in places:
        base = np.array([s, z0])
        fits.append([(_form_fit(dist, ch, f, h, base, k, sl, _LOCAL), f) for f, k in variants[ch]])
    # share out the ink: nearest character (over the points of its best-fitting forms: the most strokes drawn, and
    # nearly the best share of their length; not a poor fit that slid over to a neighbour's ink); stray if farther
    # than NEAR from all
    owner_pts = []
    for cand in fits:
        top = max((c[0][0], c[0][3]) for c in cand)
        owner_pts.append(np.concatenate([c[0][2] for c in cand if c[0][0] == top[0] and c[0][3] >= top[1] - 0.10]))
    dmin = np.stack([cKDTree(p).query(sz)[0] for p in owner_pts])
    owner = np.argmin(dmin, 0)
    res["stray"] = float((dmin.min(0) > NEAR).mean())
    chosen = []
    for n, (ch, s, wd) in enumerate(places):
        mine = sz[owner == n]                      # this character's share of the ink
        # the strokes again, drawn by this character's own share of the ink only (not by its neighbours')
        local, base = _distance_grid(mine), np.array([s, z0])
        cand = [(_form_fit(local, ch, f, h, base, k, sl, _LOCAL), (f, k)) for f, k in variants[ch]]
        (drawn, share, pts, cov), fk = max(cand, key=lambda c: (c[0][0], _explained(mine, c[0][2])))
        chosen.append((drawn, share, pts, cov, fk, mine))
    line_pts = [c[2] for c in chosen]
    for n, (drawn, share, pts, cov, (f, k), mine) in enumerate(chosen):
        res["strokes"][n], res["form"][n], res["width"][n] = int(drawn), f, k
        if len(mine) == 0 or _explained(mine, pts) < 1.0 - STRAY_CHAR:
            continue
        other = read_as_other(sz, dist, h, np.array([np.median(pts[:, 0]), z0 + h / 2]), n,
                              line_pts, text[n], share >= 1.0, cov, sl, mine)
        res["reads"][n] = other[0] if other else text[n]
        if other is None:
            res["chars"][n] = float(share)
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
    line went off the label. Otherwise 0.8 x the mean character credit (each: strokes drawn / its strokes, a stroke
    drawn when 80 % of its length is; 0 if scribbled over or if it reads as another character), plus 0.2 when every
    character is fully drawn and legible and the marker lies on the table, let go."""
    return _update(objects)


# -- scripted demo (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN, CLOSED = -0.04695, 0.0
GRIP_DZ = 0.0105              # the marker's axis this far under the TCP when picked up from the table
GRIP_BACK = float(os.environ.get("DEMO_GRIP", "0.060"))   # the demo grips the barrel this far behind the tip
PRESS = float(os.environ.get("DEMO_PRESS", "0.003"))   # the tip is commanded this far into the surface
SAG = float(os.environ.get("DEMO_SAG", "0.0"))         # added to the commanded TCP height while writing
STEP_MM = 2.5                 # writing: a waypoint every 2.5 mm,
STEP_S = float(os.environ.get("DEMO_STEP_S", "0.4"))   # this long each (0.4 s: 6.25 mm/s)
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
    return p - GRIP_BACK * d + [0.0, 0.0, GRIP_DZ + SAG], phi - math.pi


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
    g = np.array([M0[0] + TIP_X - GRIP_BACK, M0[1], TOP + M_R + GRIP_DZ])   # on the barrel, GRIP_BACK behind the tip
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
    """The left hand holds the container from behind; the right hand picks the marker up by the barrel (GRIP_BACK
    behind the tip),
    writes the text stroke by stroke with the tip pressed onto the tape, turning the marker to keep it pointing at
    the container's axis, then lays the marker back where it was and lets go."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(float(v) for v in p), f, label, y if np.ndim(y) == 2 else float(y)) for d, p, y, f, label in wps]
            for arm, wps in plan().items()}
