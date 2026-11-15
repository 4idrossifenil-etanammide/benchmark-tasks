"""On the station table:

- a small rectangular steel tank (painted grey, about 40 kg filled), its outlet toward the robot: a 1 in steel pipe
  nipple ending in a 1 in cam & groove Type A adapter x female NPT (Dixon 100-A-AL2, aluminium): 36.5 mm plug,
  overall length 50 mm, 38.1 mm hex. The adapter's axis is horizontal, 12 cm above the table;
- a 1 in cam & groove Type C coupler x hose shank (Dixon 100-C-AL, aluminium, nitrile gasket) on a 20 cm tail of
  1 in black rubber hose: 114.3 mm overall, 39.7 mm coupler body, two cam arms. Its arms are open (pointing
  straight out to the sides). It lies on two pine strips, mouth toward the tank, right of the outlet.

The robot must push the coupler fully onto the adapter (the adapter's face against the gasket) and close both cam
arms flat against the coupler body, which locks the coupling.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import (Attach, PartState, on_physics, on_reset, part, rot_z, set_attach_target,  # noqa: E402
                      set_attached)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Connect the hose to the tank outlet and lock the coupling with both cam arms."

ALU, BLACK, STEEL = (0.76, 0.77, 0.79), (0.08, 0.08, 0.08), (0.45, 0.48, 0.50)
TANK_C, PINE = (0.36, 0.40, 0.44), (0.80, 0.64, 0.42)
SIDES = 16
INCH = 0.0254


def ring_x(a_in, a_out, x0, x1, color, sides=SIDES):
    """A ring (or, with a_in = 0, a solid prism) around the x axis between apothems ``a_in`` and ``a_out`` (m), from x0
    to x1: ``sides`` boxes, each yawed 90 degrees and pitched so that its thickness runs radially."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    rm = (a_in + a_out) / 2
    out = []
    for k in range(sides):
        t = 2 * math.pi * k / sides
        out.append((((x0 + x1) / 2, rm * math.cos(t), rm * math.sin(t)), (a_out - a_in, abs(x1 - x0), side), color,
                    90.0, -math.degrees(t)))
    return out


def fixed(name, lo, hi, color):
    """A fixed box between world corners ``lo`` and ``hi``."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name=name, pos=tuple(float(v) for v in (lo + hi) / 2), size=tuple(float(v) for v in hi - lo),
               color=color)


# -- real dimensions (Dixon cam & groove catalogue; plug OD per the A-A-59326 size chart) --------------------------
PLUG_A = 1.4375 * INCH / 2         # adapter plug OD 1-7/16 in (36.5 mm)
BORE_A = 1.5 * INCH / 2            # coupler bore 1-1/2 in (38.1 mm)
TAKEUP = 1.20 * INCH               # nominal take-up length, 1 in: 30.5 mm (how far the plug goes into the coupler)
ADAPTER_B = 1.375 * INCH           # adapter length (face to hex), 100-A-AL2: 1-3/8 in
ADAPTER_A = (1 + 31 / 32) * INCH   # overall length: 1-31/32 in
HEX_C = 1.5 * INCH                 # distance across flats: 1-1/2 in
COUPLER_B = (1 + 9 / 16) * INCH    # coupler body length, 100-C-AL: 1-9/16 in
COUPLER_A = 4.5 * INCH             # overall length: 4-1/2 in
SHANK_E = (1 + 1 / 16) * INCH      # hose shank OD: 1-1/16 in
ARMS_CLOSED = (2 + 7 / 16) * INCH  # across closed cam arms: 2-7/16 in (61.9 mm)
ARMS_OPEN = (5 + 3 / 16) * INCH    # across open cam arms: 5-3/16 in (131.8 mm)
GASKET_T = 0.25 * INCH             # 100-G gasket: 1-9/16 OD x 1-1/16 ID x 1/4 in
GASKET_ID, GASKET_OD = (1 + 1 / 16) * INCH, (1 + 9 / 16) * INCH

# -- the tank and its outlet: one rigid body (origin on the adapter's axis at its face; axis +x into the tank) -------
XA, AZ = 0.26, TOP + 0.12          # adapter face (world x) and axis height
NIPPLE_A = 0.0334 / 2              # 1 in schedule 40 pipe, 33.4 mm OD
TANK_X0, TANK_X1, TANK_W, TANK_H = 0.12, 0.42, 0.36, 0.26
NOSE = 0.0015                      # rounded nose of the plug, as a 1.5 mm step
TANK_BOXES = (
    ring_x(0.0125, PLUG_A - 0.0012, 0.0, NOSE, ALU)                              # the plug's nose
    + ring_x(0.0125, PLUG_A, NOSE, 0.010, ALU)
    + ring_x(0.0125, PLUG_A - 0.0018, 0.010, 0.017, ALU)                         # the groove (see README)
    + ring_x(0.0125, PLUG_A, 0.017, ADAPTER_B, ALU)
    + ring_x(0.0125, HEX_C / 2, ADAPTER_B, ADAPTER_A, ALU, sides=6)              # the hex
    + ring_x(0.0, NIPPLE_A, ADAPTER_A, TANK_X0, STEEL)                           # the pipe nipple
    + [(((TANK_X0 + TANK_X1) / 2, 0.0, (TOP - AZ + TOP + TANK_H - AZ) / 2),
        (TANK_X1 - TANK_X0, TANK_W, TANK_H), TANK_C)])
TANK = part("tank", pos=(XA, 0.0, AZ + 0.0005), mass=40.0, color=TANK_C, friction=0.8, boxes=TANK_BOXES)

# -- the coupler (origin on its axis at the mouth; mouth toward +x, hose toward -x) ----------------------------------
WALL_A = 0.023                     # body outer apothem (46 mm body; see README)
CHAMFER = 0.0015
HOSE_A, HOSE_L = 0.017, 0.20       # 1 in hose, 34 mm OD, 20 cm tail
HOSE_X0 = -0.050                   # the hose starts over the shank
COUPLER_BOXES = (
    ring_x(BORE_A + 0.0012, WALL_A, -CHAMFER, 0.0, ALU)                          # chamfered mouth
    + ring_x(BORE_A, WALL_A, -TAKEUP, -CHAMFER, ALU)                             # the bore
    + ring_x(GASKET_ID / 2, BORE_A + 0.0005, -TAKEUP - GASKET_T, -TAKEUP, BLACK)  # the gasket at the bore's bottom
    + ring_x(0.0105, WALL_A, -COUPLER_B, -TAKEUP - GASKET_T, ALU)                # back wall
    + ring_x(0.0105, SHANK_E / 2, -COUPLER_A, -COUPLER_B, ALU, sides=12)         # hose shank
    + ring_x(0.0, HOSE_A, HOSE_X0 - HOSE_L, HOSE_X0, BLACK)  # hose (rigid stub)
    + ring_x(HOSE_A - 0.0005, HOSE_A + 0.0015, -0.068, -0.058, STEEL)            # band clamp
    + [((-0.010, s * 0.0255, z), (0.012, 0.007, 0.004), ALU) for s in (1, -1) for z in (0.0075, -0.0075)])  # ears
SEAT = TAKEUP                      # the coupler's origin in the tank frame, seated: x = SEAT
LOCK = Attach("tank", pos0=(SEAT, 0.0, 0.0), stiffness=20.0, damping=0.2, enabled=False)

C0 = np.array([0.22, -0.12, TOP + 0.035])     # the coupler's origin at the start
COUPLER = part("coupler", pos=tuple(float(v) for v in C0 + [0, 0, 0.0005]), mass=0.25, color=ALU, friction=0.7,
               boxes=COUPLER_BOXES, attach=LOCK)
# pine strips under the body and under the hose (their tops touch the body's and the hose's undersides)
STRIPS = [fixed("strip_body", (C0[0] - 0.034, C0[1] - 0.02, TOP), (C0[0] - 0.008, C0[1] + 0.02, C0[2] - WALL_A), PINE),
          fixed("strip_hose", (C0[0] - 0.20, C0[1] - 0.025, TOP), (C0[0] - 0.17, C0[1] + 0.025, C0[2] - HOSE_A), PINE)]

# -- the cam arms (origin at the pivot; closed: pointing -x along the body) -----------------------------------------
PX, PY = -0.010, 0.0255 + 0.0015   # pivot in the coupler frame (27 mm out from the axis)
ARM_L, ARM_W, ARM_H = 0.036, 0.007, 0.010   # pivot to tip; closed across 2 * (27 + 3.5) = 61 mm, open 2 * 66 = 132 mm
ARM_BOXES = [((-ARM_L / 2 + 0.002, 0.0, 0.0), (ARM_L + 0.004, ARM_W, ARM_H), ALU),
             ((-ARM_L + 0.004, 0.0, 0.0), (0.008, ARM_W + 0.002, ARM_H + 0.002), ALU)]   # thicker tip
K_ARM, D_ARM = 8.0, 0.02
OPEN0 = math.radians(90)


def _arm(side: int):
    name = "arm_l" if side > 0 else "arm_r"
    pivot = C0 + [PX, side * PY, 0.0]
    return part(name, pos=tuple(float(v) for v in pivot + [0, 0, 0.0005]), mass=0.025, color=ALU, friction=0.7, yaw=float(-side * OPEN0),
                boxes=ARM_BOXES, attach=Attach("coupler", pos0=(PX, side * PY, 0.0), stiffness=K_ARM, damping=D_ARM))


ARMS = {1: _arm(1), -1: _arm(-1)}
OBJECTS = STRIPS + [TANK, COUPLER, ARMS[1], ARMS[-1]]

# -- the cam arms and the lock (pure Python; the physics hook below feeds it) ----------------------------------------
# theta: an arm's open angle (rad), 0 = flat against the body, 90 degrees = pointing straight out.
RUN = 0.03                 # N m: a free cam arm's pivot friction
CAM = 0.20                 # N m: closing torque over the last 30 degrees, cams drawing the gasket (assumed; README)
CAM_FROM = math.radians(30)
LOBE = math.radians(40)    # the cam lobes meet the plug's outside here if the plug is in but not far enough
OPEN_MAX = math.radians(100)
CLOSED = math.radians(8)   # flat against the body (within 8 degrees): locked
REACH = 0.002              # m: the cams draw the coupler on its last 2 mm onto the gasket (assumed; README)
ON = 0.005                 # m: the plug is in the coupler's mouth


class Detent:
    """A cam arm's pivot: breakaway friction (a stiff spring to an anchor that slips), and the angles it can reach."""

    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.anchor = OPEN0

    def step(self, theta: float, lo: float, friction: float) -> float:
        pull = K_ARM * (self.anchor - theta)
        if abs(pull) > friction:
            self.anchor = theta + math.copysign(friction, pull) / K_ARM
        self.anchor = min(max(self.anchor, lo), OPEN_MAX)
        return self.anchor


def _yaw_rel(a: PartState, b: PartState) -> float:
    r = a.rotation().T @ b.rotation()
    return math.atan2(r[1, 0], r[0, 0])


def arm_angle(coupler: PartState, arm: PartState, side: int) -> float:
    return -side * _yaw_rel(coupler, arm)


def on_adapter(tank: PartState, coupler: PartState):
    """(coaxial, depth): the coupler's axis on the plug's (within 2 mm, 5 degrees) and how far the plug is in (m)."""
    p = tank.to_local(coupler.pos)
    ax = tank.rotation().T @ coupler.rotation()[:, 0]
    coax = bool(math.hypot(p[1], p[2]) < 0.002 and ax[0] > math.cos(math.radians(5)))
    return coax, float(p[0])


DETENTS = {1: Detent(), -1: Detent()}
_STATE: dict = {}
_DEBUG = [0] if os.environ.get("CAMLOCK_DEBUG") else []


def _reset_state(environment=None) -> None:
    for d in DETENTS.values():
        d.reset()
    _STATE.update(locked={1: False, -1: False}, armed={1: False, -1: False}, joined=False, dropped=False,
                  theta={1: OPEN0, -1: OPEN0}, depth=-1.0, coax=False)
    if environment is not None:
        set_attached(environment, "coupler", False)


_reset_state()
on_reset(_reset_state)


def cam_step(side: int, theta: float, coax: bool, depth: float) -> float:
    """One physics step of one arm (pure Python): updates armed / locked, returns the arm's anchor angle."""
    inside = coax and depth > ON
    near = inside and depth > SEAT - REACH
    if theta > LOBE:
        _STATE["armed"][side] = inside       # open while on the plug: closing it now works the cams
        _STATE["locked"][side] = False
    elif not inside:
        _STATE["armed"][side] = False        # closed off the plug: the lobes are in the bore (README)
    lo = LOBE if (inside and not near and _STATE["armed"][side]) else 0.0
    friction = CAM if (near and theta < CAM_FROM) else RUN
    anchor = DETENTS[side].step(theta, lo, friction)
    if near and theta < CLOSED and _STATE["armed"][side]:
        _STATE["locked"][side] = True
    return anchor


if _DEBUG:
    from jointkit import _np, on_step, quat_to_matrix

    _TICK = [0]

    @on_step
    def _watch_hands(environment) -> None:
        """CAMLOCK_DEBUG: each hand's TCP relative to its cam arm's pivot, in the cam arm's closed frame (mm)."""
        from yam_arena.scene import object_states

        _TICK[0] += 1
        if _TICK[0] % 4:
            return
        base = environment.unwrapped
        s = object_states(environment, ["arm_l", "arm_r"])
        out = []
        for hand, obj in (("left", "arm_l"), ("right", "arm_r")):
            robot = base.scene[f"{hand}_robot"]
            i = robot.find_bodies("gripper")[0][0]
            pose = _np(robot.data.body_link_pose_w)[0, i]
            r = quat_to_matrix(pose[3:7])
            tcp = pose[:3] - _np(base.scene.env_origins)[0] + r @ np.array([0.0, 0.0, 0.134])
            out.append(f"{hand} tcp {np.round(tcp * 1000, 1)} in {obj} {np.round(s[obj].to_local(tcp) * 1000, 1)}")
        print(f"[hands] t {_TICK[0] / 20:6.2f}  " + "  ".join(out), flush=True)


@on_physics
def _cams(environment) -> None:
    """Every physics step: the arms' friction and stops, the cam lock."""
    from yam_arena.scene import object_states

    s = object_states(environment, ["tank", "coupler", "arm_l", "arm_r"])
    tank, coupler = s["tank"], s["coupler"]
    coax, depth = on_adapter(tank, coupler)
    _STATE.update(coax=coax, depth=depth)
    for side, name in ((1, "arm_l"), (-1, "arm_r")):
        theta = arm_angle(coupler, s[name], side)
        _STATE["theta"][side] = theta
        anchor = cam_step(side, theta, coax, depth)
        set_attach_target(environment, name, -side * anchor)
    join = _STATE["locked"][1] or _STATE["locked"][-1]
    if join != _STATE["joined"]:
        set_attached(environment, "coupler", join)  # cams in the groove: the coupler is held on (and drawn on)
        _STATE["joined"] = join
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 48 == 0:
            p = tank.to_local(coupler.pos)
            ax = tank.rotation().T @ coupler.rotation()[:, 0]
            print(f"[camlock] axis {np.round(ax, 3)}", flush=True)
            print(f"[camlock] t {_DEBUG[0] / 240:6.2f}  coupler {np.round(coupler.pos * 1000, 1)}  in tank "
                  f"{np.round(p * 1000, 1)} coax {coax}  theta L {math.degrees(_STATE['theta'][1]):6.1f} "
                  f"R {math.degrees(_STATE['theta'][-1]):6.1f}  armed {_STATE['armed']} locked {_STATE['locked']}",
                  flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------
OFF_TABLE = TOP - 0.03
SEATED = 0.001             # the plug's face within 1 mm of the gasket


def _update(objects) -> float:
    tank, coupler = objects["tank"], objects["coupler"]
    _STATE["dropped"] |= bool(coupler.low[2] < OFF_TABLE)
    if _STATE["dropped"]:
        return 0.0
    coax, depth = on_adapter(tank, coupler)
    if not coax or depth < TAKEUP / 2:
        return 0.0
    if depth < SEAT - SEATED:
        return 0.25
    out = 0.5
    for side, name in ((1, "arm_l"), (-1, "arm_r")):
        if _STATE["locked"][side] and arm_angle(coupler, objects[name], side) < CLOSED:
            out += 0.25
    return out


def success(objects) -> bool:
    """Coupler seated on the adapter and both cam arms locked; held 1 s."""
    return _update(objects) == 1.0


def score(objects) -> float:
    """0 if the coupler fell off the table or is not on the adapter at least half way. Otherwise 0.25 on the adapter,
    not seated; 0.5 seated (the plug's face within 1 mm of the gasket); +0.25 for each cam arm that was closed flat on
    the seated coupler and is still closed."""
    return _update(objects)


# -- scripted demo (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN = -0.04695
HOSE_PINCH = -(HOSE_A - 0.005)       # 5 mm into the hose on each side
GRIP_X = -0.095                      # where the right hand holds the hose (coupler frame)
# The right arm's tracking error with the coupler in hand at the outlet (measured in simulation: once settled it hangs
# 4.5 mm low lined up 3 cm in front of the adapter, 6 mm low pushed home); added to its commands there.
SAG_PRE, SAG = np.array([0.0, 0.0, 0.0045]), np.array([0.0, 0.0, 0.0065])
PUSH = 0.006                         # pushed this far past seated (the arm's compliance presses it on)
HT = 0.0035 + 0.008                  # TCP from the arm's centre line when pushing it (half the arm + a closed pad, measured)
R_PUSH = 0.030                       # pushes the arm this far out from its pivot


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down(yaw: float) -> np.ndarray:
    return rot_z(yaw) @ frame((0, 0, -1), (0, -1, 0))


def _right_connect():
    pick = C0 + [GRIP_X, 0, 0]
    pre = np.array([XA - 0.03 + GRIP_X, 0.0, AZ]) + SAG_PRE
    home = np.array([XA + SEAT + PUSH + GRIP_X, 0.0, AZ]) + SAG
    g = down(0.0)
    return [(2.5, pick + [0, 0, 0.05], g, OPEN, "over the hose"), (1.5, pick, g, OPEN, "down"),
            (0.8, pick, g, HOSE_PINCH, "grip the hose"), (1.5, pick + [0, 0, 0.06], g, HOSE_PINCH, "lift"),
            (2.5, pre + [0, 0, 0.01], g, HOSE_PINCH, "carry to the outlet"),
            (2.0, pre, g, HOSE_PINCH, "line up with the adapter"),
            (3.0, home, g, HOSE_PINCH, "push the coupler onto the adapter"),
            (0.5, home, g, HOSE_PINCH, "hold"), (0.6, home, g, OPEN, "let go"),
            (1.0, home + [0, 0, 0.06], g, OPEN, "up")]


def _cam_point(side: int, theta: float, out: float = 0.0):
    """TCP pushing an arm at open angle ``theta``, ``out`` further off the arm on its closing side; grip rotation
    (straight down, fingers closed, pads across x)."""
    pivot = np.array([XA + SEAT + PX, side * PY, AZ])
    u = np.array([-math.cos(theta), side * math.sin(theta), 0.0])
    n = np.array([math.sin(theta), side * math.cos(theta), 0.0])
    return pivot + R_PUSH * u + (HT + out) * n, down(-side * math.pi / 2)


def _close_arm(side: int):
    """Down beside the open arm, sweep it flat and press it, then clear."""
    p, g = _cam_point(side, math.radians(98), 0.015)
    plan = [(1.5, p + [0, 0, 0.05], g, 0.0, "over the cam arm"), (1.0, p, g, 0.0, "down beside it")]
    for deg in (90, 65, 40, 15, 0, -20):
        p, g = _cam_point(side, math.radians(deg), -0.006 if deg < 0 else 0.0)
        plan.append((1.0 if deg else 0.8, p, g, 0.0, "close the cam arm" if deg > 0 else "press it flat"))
    plan.append((1.0, p, g, 0.0, "hold"))
    p, g = _cam_point(side, 0.0, 0.02)
    plan += [(0.8, p, g, 0.0, "off the arm"), (1.0, p + [0, 0, 0.06], g, 0.0, "up")]
    return plan


def plan() -> dict[str, list]:
    """Right hand connects the coupler; then the left hand closes the left cam arm and the right hand the right one,
    one after the other (two hands side by side over the coupling would collide)."""
    right = _right_connect()
    t_free = sum(w[0] for w in right)            # the right hand has let go and is up
    close_l = _close_arm(1)
    # waits out to the left until the right hand has let go and gone away
    left = [(t_free + 1.5, (0.18, 0.30, TOP + 0.20), down(0.0), 0.0, "wait")] + close_l
    left += [(1.5, (0.18, 0.30, TOP + 0.20), down(0.0), OPEN, "away")]
    t_left = sum(w[0] for w in left)             # the left hand is away
    close_r = _close_arm(-1)
    right += [(1.5, (0.18, -0.30, TOP + 0.20), down(0.0), OPEN, "away")]
    right += [(max(0.5, t_left - t_free - 1.5), right[-1][1], right[-1][2], OPEN, "wait")] + close_r
    return {"left": left, "right": right}


def demo():
    """Right arm picks the hose up behind the coupler, lines the coupler up with the adapter and pushes it on until the
    plug's face is against the gasket, then lets go. Both arms then sweep a cam arm each from open to flat against the
    body at the same time, pressing them flat."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan().items()}
