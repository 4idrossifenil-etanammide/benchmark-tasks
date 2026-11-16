"""Checks that need no GPU: the task file loads, the dimensions are the data-sheet ones, the label reader scores what
it should (and not scribbles, other dates or nothing), and the demo is reachable within the run limit."""

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

RUN_LIMIT = 1600


def written(text, h=30.0, ds=0.0, dz=0.0):
    s0, z0 = -task.text_width(text, h) / 2 + ds, task.Z_MID - h / 2 + dz
    return np.concatenate([task.char_points(c, h) + [s0 + s, z0] for c, s, _ in task.layout(text, h)])


def test_loads():
    t = load(HERE / "task.py")
    assert t.success and t.score and t.demo
    assert {"container", "marker"} <= set(t.free_objects)


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_data_sheet_dimensions():
    mm = 1000
    assert abs(task.BOT_D * mm - 88.1) < 0.1 and abs(task.TOP_D * mm - 117.1) < 0.1
    assert abs(task.HEIGHT * mm - 145.0) < 0.1
    assert abs((task.TAPE_Z1 - task.TAPE_Z0) - task.TAPE_W) < 1e-9
    assert abs(task.M_LEN * mm - 123) < 0.5 and abs(task.M_R * 2 * mm - 11) < 0.5


def test_demo_text_fits_label():
    assert task.H_NOM == 30.0
    w = task.text_width(task.TEXT, task.H_NOM)
    tape_len = task.apothem(task.Z_MID / 1000) * (task.TAPE_PHI[1] - task.TAPE_PHI[0]) * 1000
    assert w < tape_len - 10
    assert task.H_NOM < task.TAPE_W * 1000 - 10


def test_reader_accepts_the_text():
    rng = np.random.default_rng(0)
    ink = written("11/22")
    for ink in (ink + rng.normal(0, 0.7, ink.shape), written("11/22", 26, 8, 3), written("11/22", 34, -5, -2)):
        assert task.read_label(ink)["text"] == 1.0


def test_reader_rejects_nothing_scribbles_and_other_text():
    rng = np.random.default_rng(1)
    assert task.read_label(np.zeros((0, 2)))["text"] == 0.0
    scribble = np.column_stack([rng.uniform(-50, 50, 4000), rng.uniform(52, 98, 4000)])
    assert task.read_label(scribble)["text"] == 0.0
    fill = np.array([(s, z) for s in np.arange(-50, 50, 0.5) for z in np.arange(52, 98, 0.5)])
    assert task.read_label(fill)["text"] == 0.0
    assert task.read_label(written("88/88"))["text"] == 0.0
    for other in ("11/23", "17/22", "11/28"):
        r = task.read_label(written(other))
        assert r["text"] <= 0.8 + 1e-9 and min(r["chars"]) == 0.0


def test_partial_credit():
    ink = written("11/22")
    r = task.read_label(ink[: len(ink) // 2])
    assert 0.0 < r["text"] < 1.0


def test_ink_only_on_contact():
    ink = task.Ink()
    p, _ = task.surface_point(0.0, task.Z_MID)
    local = p - [task.C_XY[0], task.C_XY[1], task.TOP]
    n = np.array([np.cos(task.PHI0), np.sin(task.PHI0), 0.0])     # outward at the label's centre
    t = np.array([-np.sin(task.PHI0), np.cos(task.PHI0), 0.0])     # along the label
    out = local + n * 0.002                                         # 2 mm off the surface: no ink
    assert not ink.tip(out) and not ink.local
    assert ink.tip(local) and len(ink.local) == 1 and ink.label[0]
    assert ink.tip(local + t * 0.003) and len(ink.local) >= 6   # a 3 mm stroke: dots every 0.5 mm


def test_demo_reachable():
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm


def test_demo_fits_run_limit():
    dur = max(sum(w.duration for w in ws) for ws in task.demo().values())
    assert dur * 20 + 20 < RUN_LIMIT
    assert not math.isnan(dur)


def test_going_over_a_line_again_changes_nothing():
    ink = written("11/22")
    stray = np.column_stack([np.linspace(-30, 30, 120), np.full(120, 97.0)])   # a stray line along the tape's top
    once = task.read_label(np.concatenate([ink, stray]))
    thrice = task.read_label(np.concatenate([ink, stray, stray, stray]))
    assert once["text"] == thrice["text"] and once["stray"] == thrice["stray"]
