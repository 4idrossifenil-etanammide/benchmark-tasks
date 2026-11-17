"""Checks that need no GPU: the task file loads, the lengths are the maker's, the clip grip and the scoring do what
they should, and the demo is reachable within the run limit."""

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from jointkit import PartState  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

RUN_LIMIT = 2400


def test_loads():
    t = load(HERE / "task.py")
    assert t.success and t.score and t.demo
    assert {"channel", "clip_l", "clip_r", "worn_0", f"new_{task.N_SEG - 1}"} <= set(t.free_objects)


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_maker_lengths():
    # Ettore: rubber 1/8 in longer than the channel; assembled 1/4 in shorter than the 14 in model size
    assert abs(task.L_R / 0.0254 - 13.75) < 1e-9
    assert abs((task.L_R - task.L_C) / 0.0254 - 0.125) < 1e-9
    assert abs(task.N_SEG * task.SEG - task.L_R) < 1e-9


def test_clip_clears_a_hand_on_the_channel():
    # the worn rubber's far clip passes along the channel: nothing of it reaches the channel's outside faces
    widest = max(abs(c[0]) + 0.5 * (abs(s[0] * math.cos(math.radians(r[1]))) + abs(s[2] * math.sin(math.radians(r[1]))))
                 if (r := b[3:]) else abs(c[0]) + s[0] / 2 for b in task._clip_boxes() for c, s in [(b[0], b[1])])
    assert widest < task.W_OUT / 2


def test_channel_held_in_the_vise():
    # the channel is held fixed; its sides stand out of the jaws, which touch them, and the rubber clears the jaws
    assert task.CHANNEL.kinematic
    jaws = [o for o in task.VISE if o.name.endswith("_pad")]
    assert len(jaws) == 2
    for j in jaws:
        top = j.pos[2] + j.size[2] / 2
        assert abs(top - (task.TOP + task.VISE_H)) < 1e-9
        assert task.CZ < top < task.CZ + task.H_CH
        assert abs(abs(j.pos[0] - task.CX) - j.size[0] / 2 - task.W_OUT / 2) < 1e-9
        assert abs(j.size[1] - 4 * 0.0254) < 1e-9


def test_clip_grip_breaks_away_and_releases():
    task._reset_state()
    a, locked = task.lock_step(task.S0 - 0.0002)       # light pull: holds
    assert locked and a == task.S0
    for k in range(60):                                 # pulled out 6 mm
        a, locked = task.lock_step(task.S0 - k * 0.0001)
    assert not locked
    task._reset_state()
    a, locked = task.lock_step(task.S0 - 0.002)         # a hard tug moves the anchor, capped at the grip force
    assert abs(task.K_LOCK * (a - (task.S0 - 0.002)) - task.F_CLIPS) < 1e-9


def _state(pos, R=np.eye(3)):
    pos = np.asarray(pos, float)
    return PartState(pos=pos, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                     low=pos - 0.005, high=pos + 0.005, R=R)


def _scene(worn="channel", new="table", clips=False, shift=0.0):
    cz = task.CZ
    objs = {"channel": _state((task.CX, task.CY, cz))}
    for k in range(task.N_SEG):
        y = -task.L_R / 2 + (k + 0.5) * task.SEG
        if worn == "channel":
            objs[f"worn_{k}"] = _state((task.CX, task.CY + y, cz + task.Z_SEG))
        else:
            objs[f"worn_{k}"] = _state((0.19, -0.37 + y, task.TOP + 0.01))
        if new == "channel":
            objs[f"new_{k}"] = _state((task.CX, task.CY + y + shift, cz + task.Z_SEG))
        else:
            objs[f"new_{k}"] = _state((task.NX, task.NY_LEAD - task.L_R / 2 + y, task.TOP + task.RH / 2))
    for name, side in (("clip_l", 1), ("clip_r", -1)):
        if clips:
            y = side * (task.L_R / 2 - task.CL / 2) + shift
            objs[name] = _state((task.CX, task.CY + y, cz + task.RUB_TOP + 0.0003))
        else:
            objs[name] = _state((*task.CLIP_XY[name], task.TOP + 0.003))
    return objs


def _score(scene, clear=True, threaded=True):
    """Score one end state. ``threaded``: the rubbers went out and in along the channel (through its ends), as the
    per-step tracking would have seen; False: they jumped there (out or in over the channel's edge)."""
    task._reset_state()
    if threaded:
        task._STATE["inside"]["worn"] = [task.is_inside(task.seg_local(scene["channel"], scene[n])) for n in task.WORN_N]
        task._STATE["inside"]["new"] = [task.is_inside(task.seg_local(scene["channel"], scene[n])) for n in task.NEW_N]
    task._STATE["clear"] = clear
    return task.score(scene)


def test_nothing_done_scores_zero():
    assert _score(_scene()) == 0.0


def test_full_job_scores_one():
    assert _score(_scene(worn="tray", new="channel", clips=True)) == 1.0
    assert _score(_scene(worn="tray", new="channel", clips=True), clear=False) == 0.9


def test_partial_credit():
    assert _score(_scene(worn="tray")) == 0.25                                          # worn rubber out, in the tray
    assert _score(_scene(worn="tray", new="channel")) == 0.6                            # new one in, no clips
    assert _score(_scene(worn="tray", new="channel", clips=True, shift=0.01)) < 0.6     # 1 cm off: not all seated


def test_over_the_edge_gets_nothing():
    # both rubbers jump between the channel and elsewhere without passing an end: levered over the channel's edge
    task._reset_state()
    task._STATE["clear"] = True
    task.score(_scene())
    assert task.score(_scene(worn="tray", new="channel", clips=True)) == 0.0


def test_dropped_scores_zero():
    s = _scene(worn="tray", new="channel", clips=True)
    s["clip_l"] = _state((0.3, 0.3, task.TOP - 0.2))
    assert _score(s) == 0.0


def test_demo_reachable():
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm


def test_demo_fits_run_limit():
    dur = max(sum(w.duration for w in ws) for ws in task.demo().values())
    assert dur * 20 + 20 < RUN_LIMIT
