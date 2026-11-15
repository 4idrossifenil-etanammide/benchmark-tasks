"""Checks that need no GPU: the task file loads, the dimensions are the data-sheet ones, the cam logic scores what it
should, and the demo is reachable."""

import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402


def test_loads():
    t = load(HERE / "task.py")
    assert t.success and t.score and t.demo
    assert {"tank", "coupler", "arm_l", "arm_r"} <= set(t.free_objects)


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_data_sheet_dimensions():
    mm = 1000
    assert abs(task.PLUG_A * 2 * mm - 36.5) < 0.1 and abs(task.BORE_A * 2 * mm - 38.1) < 0.1
    assert abs(task.TAKEUP * mm - 30.5) < 0.1 and abs(task.COUPLER_A * mm - 114.3) < 0.1
    # closed / open across the arms: 61.9 / 131.8 mm on the data sheet
    closed = 2 * (task.PY + task.ARM_W / 2)
    opened = 2 * (task.PY + task.ARM_L + task.ARM_W / 2)
    assert abs(closed - task.ARMS_CLOSED) < 0.002 and abs(opened - task.ARMS_OPEN) < 0.003


def _run(depth_seq, theta_seq, coax=True):
    task._reset_state()
    for depth, theta in zip(depth_seq, theta_seq):
        task.cam_step(1, math.radians(theta), coax, depth)
    return task._STATE["locked"][1]


def test_arm_locks_only_when_closed_on_seated_coupler():
    seated = task.SEAT
    assert _run([seated] * 3, [90, 30, 2])                          # open on the seated coupler, then closed: locked
    assert not _run([seated - 0.01] * 3, [90, 30, 2])               # plug only part way in: not locked
    assert not _run([-0.05, -0.05, seated, seated], [90, 2, 2, 2])  # closed off the plug, then pushed on: not locked
    assert not _run([seated] * 3, [90, 30, 20])                     # not flat: not locked


def test_lobe_stops_arm_when_not_far_enough():
    task._reset_state()
    a = task.cam_step(1, math.radians(60), True, task.SEAT - 0.01)
    a = task.cam_step(1, math.radians(10), True, task.SEAT - 0.01)
    assert a >= task.LOBE - 1e-9


def test_demo_reachable():
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm


def test_demo_fits_run_limit():
    dur = max(sum(w.duration for w in ws) for ws in task.demo().values())
    assert dur * 20 + 20 < 1200
