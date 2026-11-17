"""On the station table:

- a low-profile drill-press vise (Wilton 11744, 4 in jaws) with magnetic soft jaw caps (Yost MR-340), its jaws along
  y in front of the robot, holding a 14 in window squeegee channel (Ettore Master clipped channel, stainless steel,
  346 mm long) by its middle, back down, blade up. The worn rubber is in it; two brass end clips hold the worn
  rubber, one at each end of the channel;
- a new 14 in squeegee rubber (Ettore Master rubber, 349 mm), standing on its edge on the table behind the channel,
  along y, right of centre;
- two new brass end clips on the table between the channel and the robot, one left, one right;
- a shallow grey plastic waste tray (inside 40 x 8 cm) on the robot's right.

The robot must take the worn rubber (with its clips) out of the channel and put it in the tray, slide the new rubber
through the channel from one end until both of its ends are at the channel's ends, and put a clip on each end.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import Attach, PartState, on_physics, on_reset, on_step, part, set_attach_target, set_attached  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Replace the worn rubber in the squeegee channel with the new rubber and clip it in."

STEEL, BLACK, WORN, BRASS, TRAY_C = (0.72, 0.73, 0.75), (0.06, 0.06, 0.06), (0.16, 0.15, 0.14), (0.78, 0.62, 0.25), (0.42, 0.44, 0.46)
VISE_C, ALU, PAD_C = (0.16, 0.30, 0.52), (0.80, 0.81, 0.83), (0.10, 0.10, 0.10)
INCH = 0.0254


def fixed(name, lo, hi, color):
    """A fixed box between world corners ``lo`` and ``hi``."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name=name, pos=tuple(float(v) for v in (lo + hi) / 2), size=tuple(float(v) for v in hi - lo),
               color=color)


# -- real lengths (Ettore rubber-change instructions: rubber 1/8 in longer than the channel, assembled length = model
#    size - 1/4 in, so 13-3/4 in for the 14 in model) ----------------------------------------------------------------
L_R = 13.75 * INCH                 # rubber, 349.3 mm
L_C = L_R - INCH / 8               # channel, 346.1 mm
# -- cross-sections: OUR assumptions (Ettore publishes neither; README) ----------------------------------------------
RW, RH = 0.006, 0.012              # rubber 6 mm thick, 12 mm tall
DEPTH = 0.007                      # held 7 mm deep in the channel; 5 mm of blade stands out
T = 0.0008                         # channel sheet
GAP = 0.0072                       # channel inside width (0.6 mm play each side of the rubber)
W_OUT = GAP + 2 * T                # 8.8 mm
H_CH = T + DEPTH                   # 7.8 mm
RUBBER_DENSITY = 1200.0            # kg/m3
E_RUBBER = 4.0e6                   # Pa, about Shore 60A
N_SEG = 10
SEG = L_R / N_SEG                  # 34.9 mm segments
SEG_MASS = RUBBER_DENSITY * RW * RH * SEG
K_BEND = E_RUBBER * (RH * RW ** 3 / 12) / SEG     # N m/rad per joint, bending through the 6 mm thickness
D_BEND = 3e-4

# -- the vise: Wilton 11744 4 in low-profile drill-press vise (data sheet: jaw width 4 in, jaw depth 1-1/2 in, height
#    2-7/8 in, 6 x 11-1/2 in), with Yost MR-340 4 in magnetic aluminium jaw caps with a rubber face (cap thickness
#    ours). The fixed jaw faces the robot; the screw runs away from it along +x. --------------------------------------
CX, CY = 0.30, 0.0                 # the channel's centre line (and the jaws' centre)
VISE_H = 2.875 * INCH              # jaw top above the table, 73.0 mm
JAW_D = 1.5 * INCH                 # jaw depth, 38.1 mm
JAW_W = 4.0 * INCH                 # jaw width (along the channel), 101.6 mm
VISE_W, VISE_L = 6.0 * INCH, 11.5 * INCH
JAW_T, CAP_AL, CAP_RUB = 0.020, 0.0045, 0.0015   # jaw plate, cap aluminium and rubber face thickness (ours)
GRIP_D = 0.005                     # the channel's lower 5 mm is in the jaws: its back 5 mm below the jaw tops
CZ = TOP + VISE_H - GRIP_D         # the channel's back (its origin) above the table: 68.0 mm
_B = TOP + VISE_H - JAW_D          # the vise bed
_xin = W_OUT / 2                   # the caps' rubber faces touch the channel's sides
_x0 = CX - _xin - CAP_RUB - CAP_AL - JAW_T       # the fixed jaw's back, toward the robot
VISE = [fixed("vise_bed", (_x0, CY - VISE_W / 2, TOP), (_x0 + VISE_L, CY + VISE_W / 2, _B), VISE_C)]
for _s, _n in ((-1, "fixed"), (1, "moving")):
    _f = CX + _s * _xin
    _r = _f + _s * CAP_RUB
    _a = _r + _s * CAP_AL
    _j = _a + _s * JAW_T
    for _nm, _p, _q, _c in (("pad", _f, _r, PAD_C), ("cap", _r, _a, ALU), ("jaw", _a, _j, VISE_C)):
        VISE.append(fixed(f"vise_{_n}_{_nm}", (min(_p, _q), CY - JAW_W / 2, _B), (max(_p, _q), CY + JAW_W / 2,
                                                                                    TOP + VISE_H), _c))
VISE.append(fixed("vise_screw", (CX + _xin + CAP_RUB + CAP_AL + JAW_T, CY - 0.009, _B + 0.004),
                  (_x0 + VISE_L + 0.03, CY + 0.009, _B + 0.022), STEEL))

# -- the channel (origin: centre of its back's underside), held fixed in the vise ---------------------------------
CHANNEL_BOXES = [((0.0, 0.0, T / 2), (W_OUT, L_C, T), STEEL),
                 ((GAP / 2 + T / 2, 0.0, H_CH / 2), (T, L_C, H_CH), STEEL),
                 ((-GAP / 2 - T / 2, 0.0, H_CH / 2), (T, L_C, H_CH), STEEL)]
CHANNEL_MASS = 7900.0 * T * (W_OUT + 2 * DEPTH) * L_C   # about 55 g
CHANNEL = part("channel", pos=(CX, CY, CZ), mass=CHANNEL_MASS, color=STEEL, friction=0.5, boxes=CHANNEL_BOXES,
               kinematic=True)
Z_SEG = T + RH / 2                 # a seated segment's centre above the channel's back
RUB_TOP = T + RH                   # the seated rubber's top above the channel's back

# -- end clips (origin: centre of the bridge's underside; it sits on the rubber's top edge) -------------------------
CL = 0.010                         # clip length along the rubber
CB_T = 0.001                       # bridge thickness
LEG_IN, LEG_T, LEG_H = 0.00325, 0.0006, 0.0015
CH_A = math.radians(30)            # lead-in: the legs flare out below LEG_H (to 4.2 mm off the centre line)
CH_L = 0.0008
CLIP_W = 2 * (LEG_IN + LEG_T)      # 7.7 mm across the legs
CLIP_MASS = 0.002


def _clip_boxes(y0=0.0, z0=0.0):
    out = [((0.0, y0, z0 + CB_T / 2), (CLIP_W, CL, CB_T), BRASS)]
    for s in (1, -1):
        out.append(((s * (LEG_IN + LEG_T / 2), y0, z0 - LEG_H / 2), (LEG_T, CL, LEG_H), BRASS))
        cx = LEG_IN + math.sin(CH_A) * CH_L / 2 + math.cos(CH_A) * LEG_T / 2
        cz = -LEG_H - math.cos(CH_A) * CH_L / 2
        out.append(((s * cx, y0, z0 + cz), (LEG_T, CL, CH_L), BRASS, 0.0, -s * math.degrees(CH_A)))
    return out


CLIP_DROP = LEG_H + math.cos(CH_A) * CH_L + math.sin(CH_A) * LEG_T / 2   # bridge underside above the legs' lowest point
CLIP_XY = {"clip_l": (0.20, 0.10), "clip_r": (0.20, -0.10)}
CLIPS = [part(n, pos=(x, y, TOP + CLIP_DROP + 0.0005), mass=CLIP_MASS, color=BRASS, friction=0.6,
              boxes=_clip_boxes()) for n, (x, y) in CLIP_XY.items()]

# -- the rubbers: N_SEG rigid segments on spring hinges (vertical axis: they bend through their 6 mm thickness) -----
# Segment k = 0 is the rubber's -y end. Each joins the one before it.
RX = ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0))   # joint frame whose slide axis is the part's +y
S0, LOCK_TRAVEL = 0.008, 0.014     # the clip lock: slide coordinate at the start and its range (m)
F_CLIPS = 1.0                      # N: the two seated clips' grip on the channel (assumed; README)
K_LOCK = 3000.0
RELEASE_OUT, RELEASE_IN = 0.004, 0.003


def _rubber(prefix, x, y_minus_end, z_centre, color, clipped):
    out = []
    for k in range(N_SEG):
        yk = y_minus_end + (k + 0.5) * SEG
        boxes = [((0.0, 0.0, 0.0), (RW, SEG, RH), color)]
        if clipped and k in (0, N_SEG - 1):  # its clip, sitting on the top edge at the end (moves with it)
            yc = (-SEG / 2 + CL / 2) if k == 0 else (SEG / 2 - CL / 2)
            boxes += _clip_boxes(yc, RH / 2)
        if k:
            att = Attach(f"{prefix}_{k - 1}", pos0=(0.0, SEG / 2, 0.0), pos1=(0.0, -SEG / 2, 0.0),
                         stiffness=K_BEND, damping=D_BEND)
        elif clipped:   # the clips' grip on the channel (one slide joint, released once pulled out of the seat)
            att = Attach("channel", pos0=(0.0, yk - CY - S0, Z_SEG), pos1=(0.0, 0.0, 0.0), rot0=RX, rot1=RX,
                         travel=LOCK_TRAVEL, lin_stiffness=K_LOCK, lin_damping=2.0, stiffness=0.0, damping=1e-4)
        else:
            att = None
        out.append(part(f"{prefix}_{k}", pos=(x, yk, z_centre + 0.0005), mass=SEG_MASS, color=color, friction=0.9,
                        boxes=boxes, attach=att))
    return out


WORN = _rubber("worn", CX, CY - L_R / 2, CZ + Z_SEG - 0.0005, WORN, clipped=True)
NX, NY_LEAD = 0.42, -0.17          # the new rubber stands on its edge at x 42 cm, its +y end at y -17 cm
NEW = _rubber("new", NX, NY_LEAD - L_R, TOP + RH / 2, BLACK, clipped=False)

# -- the waste tray (fixed): inside x 15..23 cm, y -57..-17 cm, 15 mm walls, 2 mm floor ------------------------------
TX0, TX1, TY0, TY1, TW, TH, TF = 0.15, 0.23, -0.57, -0.17, 0.004, 0.015, 0.002
TRAY = [fixed("tray_floor", (TX0 - TW, TY0 - TW, TOP), (TX1 + TW, TY1 + TW, TOP + TF), TRAY_C),
        fixed("tray_n", (TX1, TY0 - TW, TOP), (TX1 + TW, TY1 + TW, TOP + TH), TRAY_C),
        fixed("tray_s", (TX0 - TW, TY0 - TW, TOP), (TX0, TY1 + TW, TOP + TH), TRAY_C),
        fixed("tray_e", (TX0, TY0 - TW, TOP), (TX1, TY0, TOP + TH), TRAY_C),
        fixed("tray_w", (TX0, TY1, TOP), (TX1, TY1 + TW, TOP + TH), TRAY_C)]

OBJECTS = TRAY + VISE + [CHANNEL] + WORN + NEW + CLIPS
WORN_N = [p.name for p in WORN]
NEW_N = [p.name for p in NEW]

# -- the clips' grip on the worn rubber (physics hook) ------------------------------------------------------------
_STATE: dict = {}
_DEBUG = [0] if os.environ.get("SQUEEGEE_DEBUG") else []
_TICK = [0]


def _reset_state(environment=None) -> None:
    _STATE.update(anchor=S0, locked=True, slide=S0, dropped=False, pried={"worn": False, "new": False},
                  inside={"worn": [True] * N_SEG, "new": [False] * N_SEG}, last={}, clear=False)
    if environment is not None:
        set_attached(environment, "worn_0", True)


_reset_state()
on_reset(_reset_state)


def lock_step(slide: float) -> tuple[float, bool]:
    """One physics step of the clips' grip (pure Python): the slide joint's anchor follows the rubber once the pull
    passes F_CLIPS (breakaway friction); out of the seat by RELEASE_OUT (pulled) or RELEASE_IN (pushed in), the clips
    are free of the channel's ends. Returns (anchor, still locked)."""
    if not _STATE["locked"]:
        return _STATE["anchor"], False
    pull = K_LOCK * (_STATE["anchor"] - slide)
    if abs(pull) > F_CLIPS:
        _STATE["anchor"] = slide + math.copysign(F_CLIPS, pull) / K_LOCK
    _STATE["slide"] = slide
    if slide < S0 - RELEASE_OUT or slide > S0 + RELEASE_IN:
        _STATE["locked"] = False
    return _STATE["anchor"], _STATE["locked"]


@on_physics
def _clips(environment) -> None:
    from yam_arena.scene import object_states

    if not _STATE["locked"]:
        return

    s = object_states(environment, ["channel", "worn_0"])
    slide = float(s["channel"].to_local(s["worn_0"].pos)[1]) - (WORN[0].pos[1] - CY - S0)
    anchor, locked = lock_step(slide)
    if locked:
        set_attach_target(environment, "worn_0", 0.0, height=anchor)
    else:
        set_attached(environment, "worn_0", False)
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 24 == 0 or not locked:
            print(f"[clips] t {_DEBUG[0] / 240:6.2f} slide {slide * 1000:6.2f} mm anchor {anchor * 1000:6.2f} "
                  f"locked {locked}", flush=True)


CLEAR = 0.05


@on_step
def _hands(environment) -> None:
    """Both hands clear of the channel: each tool centre point 5 cm or more above the channel's rim, or 5 cm or more
    away from the channel in plan."""
    from jointkit import _np, quat_to_matrix
    from yam_arena.scene import object_states

    _TICK[0] += 1
    base = environment.unwrapped
    names = ["channel"] + (NEW_N + ["clip_l", "clip_r"] if _DEBUG else [])
    st = object_states(environment, names)
    ch = st["channel"]
    clear = True
    for hand in ("left", "right"):
        robot = base.scene[f"{hand}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = _np(robot.data.body_link_pose_w)[0, i]
        tcp = pose[:3] - _np(base.scene.env_origins)[0] + quat_to_matrix(pose[3:7]) @ np.array([0.0, 0.0, 0.134])
        p = ch.to_local(tcp)
        near = abs(p[0]) < CLEAR and abs(p[1]) < L_C / 2 + CLEAR
        clear &= bool(p[2] > H_CH + CLEAR or not near)
        if _DEBUG and _TICK[0] % 10 == 0:
            print(f"[tcp] t {_TICK[0] / 20:6.2f} {hand} {np.round(tcp * 1000, 1)}", flush=True)
    if _DEBUG and _TICK[0] % 10 == 0:
        loc = [ch.to_local(st[n].pos) for n in NEW_N]
        print(f"[new] t {_TICK[0] / 20:6.2f} " + " ".join(f"{q[0]*1000:.1f},{q[1]*1000:.0f},{q[2]*1000:.1f}"
                                                         for q in loc), flush=True)
        for c in ("clip_l", "clip_r"):
            q = ch.to_local(st[c].pos)
            print(f"[{c}] t {_TICK[0] / 20:6.2f} local {np.round(q * 1000, 2)} x-axis "
                  f"{np.round(ch.rotation().T @ st[c].rotation()[:, 0], 3)}", flush=True)
    _STATE["clear"] = clear


# -- scoring --------------------------------------------------------------------------------------------------------
OFF_TABLE = TOP - 0.03
IN_X = GAP / 2 - RW / 2 + 0.0005   # a segment's centre this close to the channel's centre line (1.1 mm)
LIFT_MAX = DEPTH - 0.001           # its bottom below the channel's rim by 1 mm or more
SEAT_Z = 0.0012                    # seated: its bottom within 1.2 mm of the channel's back
ENDS = (-0.002, 0.005)             # each rubber end from 2 mm inside to 5 mm beyond the channel's end
CLIP_Y, CLIP_X, CLIP_Z = 0.004, 0.0012, (-0.0005, 0.0012)
PRY_MARGIN = 0.03                  # in or out more than 3 cm from both ends: over the channel's edge, not along it


def seg_local(ch: PartState, s: PartState) -> np.ndarray:
    return ch.to_local(s.pos)


def is_inside(p) -> bool:
    """A segment (its centre in the channel frame) in the channel: on its centre line and down between its sides."""
    return bool(abs(p[0]) < IN_X and abs(p[1]) < L_C / 2 and p[2] - RH / 2 - T < LIFT_MAX)


def is_seated(p) -> bool:
    return is_inside(p) and abs(p[2] - Z_SEG) < SEAT_Z


def track(which: str, locals_) -> list[bool]:
    """Update which segments are in the channel; a segment that goes in or out more than PRY_MARGIN from both ends
    latches ``pried`` (the real channel's folded edges let the rubber in and out only at the ends)."""
    now = [is_inside(p) for p in locals_]
    before = _STATE["last"].get(which) or locals_
    for was, isn, p, q in zip(_STATE["inside"][which], now, locals_, before):
        if was != isn and min(abs(p[1]), abs(q[1])) < L_C / 2 - PRY_MARGIN:
            _STATE["pried"][which] = True
    _STATE["inside"][which] = now
    _STATE["last"][which] = locals_
    return now


def in_tray(s: PartState) -> bool:
    x, y, z = s.pos
    return bool(TX0 < x < TX1 and TY0 < y < TY1 and z < TOP + TH)


def rubber_ends(locals_) -> tuple[float, float]:
    """(overhang at -y, overhang at +y) of a rubber in the channel (m; > 0: beyond the channel's end)."""
    ys = [p[1] for p in locals_]
    return -L_C / 2 - (min(ys) - SEG / 2), max(ys) + SEG / 2 - L_C / 2


def clip_seated(ch: PartState, clip: PartState, end_y: float, side: int) -> bool:
    """A clip on the rubber's top edge at that end: centred on it, resting on it, its outer face at the rubber's end."""
    p = ch.to_local(clip.pos)
    ax = ch.rotation().T @ clip.rotation()[:, 0]
    return bool(abs(p[0]) < CLIP_X and CLIP_Z[0] < p[2] - RUB_TOP < CLIP_Z[1]
                and abs(p[1] - (end_y - side * CL / 2)) < CLIP_Y and abs(ax[0]) > math.cos(math.radians(10)))


def _update(objects) -> dict:
    ch = objects["channel"]
    names = ["channel"] + WORN_N + NEW_N + ["clip_l", "clip_r"]
    _STATE["dropped"] |= any(objects[n].low[2] < OFF_TABLE for n in names)
    worn = [seg_local(ch, objects[n]) for n in WORN_N]
    new = [seg_local(ch, objects[n]) for n in NEW_N]
    worn_in, new_in = track("worn", worn), track("new", new)
    out = dict(worn=0.0, new=0.0, clips=0.0, clear=0.0)
    if _STATE["dropped"]:
        return out
    if not any(worn_in) and not _STATE["pried"]["worn"]:
        out["worn"] = 0.10 + (0.15 if all(in_tray(objects[n]) for n in WORN_N) else 0.0)
    full = False
    if not _STATE["pried"]["new"]:
        seated = [is_seated(p) for p in new]
        out["new"] = 0.15 * sum(seated) / N_SEG
        lo, hi = rubber_ends(new)
        full = all(seated) and ENDS[0] < lo < ENDS[1] and ENDS[0] < hi < ENDS[1]
        if full:
            out["new"] += 0.20
            ys = [p[1] for p in new]
            end_m, end_p = min(ys) - SEG / 2, max(ys) + SEG / 2
            for end_y, side in ((end_m, -1), (end_p, 1)):
                if any(clip_seated(ch, objects[c], end_y, side) for c in ("clip_l", "clip_r")):
                    out["clips"] += 0.15
    if out["worn"] == 0.25 and full and out["clips"] == 0.30 and _STATE["clear"]:
        out["clear"] = 0.10
    return out


def success(objects) -> bool:
    """Everything done and the hands clear; held 1 s."""
    return abs(sum(_update(objects).values()) - 1.0) < 1e-9


def score(objects) -> float:
    """0 if anything fell off the table. Worn rubber out of the channel 0.10, in the tray +0.15 (none if levered out
    over the channel's edge). New rubber: 0.15 x the fraction of it seated in the channel; +0.20 once all of it is
    seated with both ends at the channel's ends (2 mm inside to 5 mm beyond); then +0.15 for a clip seated on each
    end. +0.10 for both hands clear of the channel once all that is done. None for the new rubber if it was pressed
    in over the channel's edge."""
    return round(sum(_update(objects).values()), 4)


# -- scripted demo (fixed waypoints; no state read) ---------------------------------------------------------------
OPEN = -0.04695
TIP = 0.013                         # pad tips below the tool centre point
YX = math.pi / 2                    # fingers closing along x
PINCH_R = -(RW / 2 - 0.0025)        # 2.5 mm into the rubber on each side
PINCH_CLIP = -(CLIP_W / 2 - 0.0012)
PAD_FACE = 0.008                    # closed fingers' outer face from the tool centre point (closing direction)
Z_RIM = CZ + H_CH
Z_SEATED_TOP = CZ + RUB_TOP
Z_PULL = Z_RIM + 0.0018 + TIP                   # pads on the top 3.2 mm of a seated rubber (tips 1.8 mm over the rim)
PICK_D = 0.007                                  # pads on the top 7 mm of the new rubber (holds it level)
Z_PICK = TOP + RH - PICK_D + TIP
LIFT_INS = 0.0045                               # carried with its bottom 4.5 mm over the channel's back: the pad
Z_INS = CZ + LIFT_INS + RH - PICK_D + TIP       # tips pass 1.7 mm over the rim
PINCH_BACK = 0.08                               # the new rubber pinched this far behind its leading end (near
#                                                 enough to its middle that it stays level)
X_TAUGHT = 0.00105                              # taught offset: the arm sits about 1 mm short in x there
X_CLIP_R = 0.0010                               # taught offset for the right hand's clip
Z_CLIP_PICK = TOP + CLIP_DROP - 0.0013 + TIP    # pad tips 1.3 mm below the bridge
Z_CLIP_SET = Z_SEATED_TOP + 0.0008 - 0.0013 + TIP
END_P = L_C / 2                                 # the new rubber pushed flush with the channel's +y end ...
END_M = L_C / 2 - L_R                           # ... so its -y end stands 3.2 mm out of the other
UP = 0.05
GRIP_WAIT = 0.6                     # the fingers take about 1.2 s to close from wide open: wait before moving on


def wp(d, x, y, z, f, label, yaw=YX):
    return (d, (x, y, z), f, label, yaw)


BREAK = 0.02                        # the first pull: the clips come out of the channel's ends


def _right_remove():
    y0, y1a, y1 = CY - 0.14, CY - 0.27, CY - 0.36   # first stroke 13 cm: most of the rubber stays in the channel
    plan = [wp(2.5, CX, y0, Z_PULL + UP, OPEN, "over the worn rubber"),
            wp(1.2, CX, y0, Z_PULL, OPEN, "down"),
            wp(0.8, CX, y0, Z_PULL, PINCH_R, "pinch the blade"),
            wp(GRIP_WAIT, CX, y0, Z_PULL, PINCH_R, "pinch the blade"),
            wp(1.0, CX, y0 - BREAK, Z_PULL, PINCH_R, "pull the clips out of the channel's ends"),
            wp(3.0, CX, y1a, Z_PULL, PINCH_R, "slide it out along the channel"),
            wp(0.5, CX, y1a, Z_PULL, OPEN, "let go"), wp(0.8, CX, y1a, Z_PULL + 0.03, OPEN, "up"),
            wp(1.5, CX, y0, Z_PULL + UP, OPEN, "back along it"),
            wp(1.2, CX, y0, Z_PULL, OPEN, "down"),
            wp(0.8, CX, y0, Z_PULL, PINCH_R, "pinch the blade"),
            wp(GRIP_WAIT, CX, y0, Z_PULL, PINCH_R, "pinch the blade"),
            wp(4.0, CX, y1, Z_PULL, PINCH_R, "slide it out of the channel")]
    hx, hy = (TX0 + TX1) / 2, -0.38
    z_set = TOP + TF + 0.002 + RH - 0.0025 + TIP
    plan += [wp(1.5, CX, y1, Z_PULL + UP, PINCH_R, "lift it"),
             wp(2.5, hx, hy, Z_PULL + UP, PINCH_R, "over the tray"),
             wp(1.5, hx, hy, z_set, PINCH_R, "lower it in"),
             wp(0.6, hx, hy, z_set, OPEN, "let go"),
             wp(1.0, hx, hy, Z_PULL + UP, OPEN, "up")]
    return plan


Y_OUT = 0.205                                   # the new rubber slid on until its leading end is here (3.2 cm out)


def _right_thread():
    yp = NY_LEAD - PINCH_BACK
    ys = CY - L_C / 2 + 0.015 - PINCH_BACK    # leading end 1.5 cm in over the channel's end
    xs = CX + X_TAUGHT
    y1 = CY + 0.0                             # end of the first stroke: leading end 8 cm past the middle
    y2 = CY - 0.16                            # second pinch, just inside the channel's end
    return [wp(2.0, NX, yp, Z_PICK + UP, OPEN, "over the new rubber"),
            wp(1.5, NX, yp, Z_PICK, OPEN, "down"),
            wp(0.8, NX, yp, Z_PICK, PINCH_R, "pinch it"),
            wp(GRIP_WAIT, NX, yp, Z_PICK, PINCH_R, "pinch it"),
            wp(1.5, NX, yp, Z_INS + 0.015, PINCH_R, "lift"),
            wp(3.0, xs, ys, Z_INS + 0.015, PINCH_R, "line it up with the channel"),
            wp(1.2, xs, ys, Z_INS + 0.0055, PINCH_R, "down to just over the channel's end"),
            wp(1.5, xs, ys, Z_INS + 0.0055, PINCH_R, "let it settle"),
            wp(1.5, xs, ys, Z_INS, PINCH_R, "lower its end into the channel"),
            wp(6.0, xs, y1, Z_INS, PINCH_R, "slide it into the channel"),
            wp(0.5, xs, y1, Z_INS, OPEN, "let go"),
            wp(0.8, CX, y1, Z_PULL + 0.025, OPEN, "up"),
            wp(2.0, CX, y2, Z_PULL + 0.025, OPEN, "back along it"),
            wp(1.2, CX, y2, Z_PULL, OPEN, "down"),
            wp(0.8, CX, y2, Z_PULL, PINCH_R, "pinch"),
            wp(GRIP_WAIT, CX, y2, Z_PULL, PINCH_R, "pinch"),
            wp(3.5, CX, y2 + Y_OUT - (y1 + PINCH_BACK), Z_PULL, PINCH_R, "slide it on through"),
            wp(0.5, CX, y2 + Y_OUT - (y1 + PINCH_BACK), Z_PULL, OPEN, "let go"),
            wp(1.0, CX, y2 + Y_OUT - (y1 + PINCH_BACK), Z_PULL + UP, OPEN, "up")]


def _left_push():
    """Closed fingers pushed against the rubber's +y end, down past the channel's back: the channel's end stops them,
    so the rubber ends flush with it."""
    y_stop = CY + END_P + PAD_FACE - 0.0015       # pressed 1.5 mm past the stop
    z = CZ - 0.002 + TIP                          # pad tips 2 mm below the channel's back
    return [wp(2.0, CX, y_stop + 0.045, z + 0.06, 0.0, "beside the rubber's end", 0.0),
            wp(1.2, CX, y_stop + 0.045, z, 0.0, "down", 0.0),
            wp(3.0, CX, y_stop, z, 0.0, "push it back to the channel's end", 0.0),
            wp(0.5, CX, y_stop, z, 0.0, "hold", 0.0),
            wp(0.8, CX, y_stop + 0.02, z, 0.0, "back off", 0.0),
            wp(1.0, CX, y_stop + 0.02, z + 0.06, 0.0, "up", 0.0)]


def _clip_pick(name):
    px, py = CLIP_XY[name]
    return [wp(2.0, px, py, Z_CLIP_PICK + UP, OPEN, "over a clip"),
            wp(1.2, px, py, Z_CLIP_PICK, OPEN, "down"),
            wp(0.8, px, py, Z_CLIP_PICK, PINCH_CLIP, "pick the clip"),
            wp(GRIP_WAIT, px, py, Z_CLIP_PICK, PINCH_CLIP, "pick the clip"),
            wp(1.0, px, py, Z_CLIP_SET + UP, PINCH_CLIP, "lift")]


def _clip_set(y, away, x=CX):
    return [wp(2.5, x, y, Z_CLIP_SET + UP, PINCH_CLIP, "over the rubber's end"),
            wp(1.5, x, y, Z_CLIP_SET + 0.004, PINCH_CLIP, "lower it to just above the blade"),
            wp(0.8, x, y, Z_CLIP_SET + 0.004, PINCH_CLIP, "let the arm settle"),
            wp(1.0, x, y, Z_CLIP_SET, PINCH_CLIP, "lower it onto the blade"),
            wp(0.6, x, y, Z_CLIP_SET, -0.010, "let go"),
            wp(0.8, x, y, Z_CLIP_SET + 0.012, 0.0, "up, close"),
            wp(1.0, x, y, Z_SEATED_TOP + CB_T - 0.001 + TIP, 0.0, "press it on"),
            wp(1.0, x, y, Z_CLIP_SET + UP, 0.0, "up"),
            wp(1.5, away[0], away[1], Z_CLIP_SET + UP, 0.0, "away")]


def _t(plan):
    return sum(w[0] for w in plan)


def _wait(arm, until, label="wait"):
    """Stay at the last waypoint until time ``until``."""
    if _t(arm) < until:
        last = arm[-1]
        arm.append((until - _t(arm), last[1], last[2], label, last[4]))


def _left(plan):
    """The left arm closes its fingers across x the other way round (its wrist's range)."""
    return [(d, p, f, lab, -y if y == YX else y) for d, p, f, lab, y in plan]


def plan() -> dict[str, list]:
    """The channel is held in the vise. The right hand pulls the worn rubber's clips out of the channel's ends, slides
    the worn rubber out in two strokes and lays it in the tray. It picks up the new rubber 8 cm from one end, holds it
    in line with the channel with that end 1.5 cm in over the channel's -y end, lowers the end in and slides the rubber
    in, then pinches it again at that end and slides it on until it stands 3 cm out of the +y end. The left hand pushes that end back flush with the channel's end (the channel's end stops its
    fingers) while the right picks up a clip; each hand then sets a clip on the rubber's top edge at its end and
    presses it on."""
    right = _right_remove() + _right_thread()
    t_threaded = _t(right)
    left = [wp(3.0, 0.22, 0.24, TOP + 0.14, 0.0, "out of the way", 0.0)]
    push = _left_push()
    _wait(left, t_threaded - push[0][0] + 1.0)
    left += push
    right += _clip_pick("clip_r")
    set_r = _clip_set(END_M + CL / 2, (0.20, -0.22), CX + X_CLIP_R)
    right += set_r[:1]
    _wait(right, _t(left))
    right += set_r[1:]
    left += _left(_clip_pick("clip_l") + _clip_set(END_P - CL / 2, (0.20, 0.22)))
    return {"left": left, "right": right}


def demo():
    """See plan(): a fixed sequence of tool-centre-point waypoints, nothing read from the simulation."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(float(v) for v in p), f, label, y) for d, p, f, label, y in wps]
            for arm, wps in plan().items()}
