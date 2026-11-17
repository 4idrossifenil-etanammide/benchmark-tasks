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


def written(text, h=30.0, ds=0.0, dz=0.0, forms=None, drop=None):
    """Ink of ``text`` written along the accepted strokes, centred on the label; ``forms`` {index: form} picks other
    accepted forms, ``drop`` {index: [stroke numbers]} leaves strokes out."""
    s0, z0 = -task.text_width(text, h) / 2 + ds, task.Z_MID - h / 2 + dz
    forms, drop, out = forms or {}, drop or {}, []
    for i, (c, s, _) in enumerate(task.layout(text, h)):
        core, opt = task.FORMS[c][1][forms.get(i, 0)]
        out += [task.resample(st * h, 0.5) + [s0 + s, z0] for k, st in enumerate(list(core) + list(opt))
                if k not in drop.get(i, ())]
    return np.concatenate(out)


def test_loads():
    t = load(HERE / "task.py")
    assert t.success and t.score and t.demo
    assert {"container", "marker"} <= set(t.free_objects) and "paper" not in t.free_objects


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


def test_stroke_counts():
    assert task.STROKES == {"0": 1, "1": 1, "2": 2, "3": 2, "4": 3, "5": 3, "6": 2, "7": 2, "8": 2, "9": 2, "/": 1,
                            "-": 1}


def test_partial_credit_is_strokes_drawn_over_strokes():
    r = task.read_label(written("11/22", drop={4: [1]}))            # last "2" without its base line: 1 of 2
    assert r["chars"] == [1.0, 1.0, 1.0, 1.0, 0.5] and abs(r["text"] - 0.9) < 1e-9
    r = task.read_label(written("04/14", drop={1: [2]}), "04/14")   # first "4" without its upright: 2 of 3
    assert abs(r["chars"][1] - 2 / 3) < 1e-9 and abs(r["text"] - (4 + 2 / 3) / 5) < 1e-9
    r = task.read_label(written("08/26", drop={1: [1]}), "08/26")   # "8" without its lower loop: 1 of 2
    assert r["chars"][1] == 0.5
    r = task.read_label(written("05/16", drop={1: [0]}), "05/16")   # "5" without its top bar: 2 of 3
    assert abs(r["chars"][1] - 2 / 3) < 1e-9


def test_other_dates_read_when_written_as_shown():
    rng = np.random.default_rng(2)
    for d in ("03/09", "12/05", "10/31", "04/17", "08/26", "06/28", "09/30", "01/14", "07/19", "05/16", "02/27",
              "12/31"):
        assert task.read_label(written(d), d)["text"] == 1.0, d
        ink = written(d)
        assert task.read_label(ink + rng.normal(0, 0.7, ink.shape), d)["text"] == 1.0, d
        assert task.read_label(written(d, 26, 6, 3), d)["text"] == 1.0, d


def test_common_letterforms_read():
    for d, forms in (("11/22", {0: 1, 1: 1}),    # "1" with a flag
                     ("11/22", {0: 2, 1: 2}),    # "1" with a base
                     ("11/22", {0: 3, 1: 3}),    # "1" with a flag and a base
                     ("04/14", {1: 1, 4: 1}),    # open-top "4"
                     ("07/17", {1: 1, 4: 1}),    # crossed "7"
                     ("11/17", {0: 3, 3: 1, 4: 1})):
        r = task.read_label(written(d, forms=forms), d)
        assert r["text"] == 1.0, (d, forms, r)


def test_other_digits_and_dates_fail():
    for target, ink in (("11/22", "11/23"), ("11/22", "17/22"), ("11/22", "11/28"), ("03/09", "08/09"),
                        ("03/09", "03/08"), ("10/31", "10/37"), ("12/05", "12/06"), ("04/17", "09/17"),
                        ("06/28", "05/28"), ("11/22", "11-22"), ("11/22", "71/22"), ("18/22", "15/22")):
        r = task.read_label(written(ink), target)
        assert r["text"] <= 0.8 + 1e-9 and min(r["chars"]) == 0.0, (target, ink, r)
    for a in "0123456789":       # any other digit in place of one: that character gets nothing
        for b in "0123456789":
            if a != b:
                assert task.read_label(written(f"1{b}/22"), f"1{a}/22")["text"] <= 0.8 + 1e-9, (a, b)
    rng = np.random.default_rng(3)
    scribble = np.column_stack([rng.uniform(-50, 50, 4000), rng.uniform(52, 98, 4000)])
    assert task.read_label(scribble, "03/09")["text"] == 0.0


def test_ink_needs_nib_contact_force():
    ink = task.Ink()
    p, _ = task.surface_point(0.0, task.Z_MID)
    local = p - [task.C_XY[0], task.C_XY[1], task.TOP]
    t = np.array([-np.sin(task.PHI0), np.cos(task.PHI0), 0.0])     # along the label
    assert task.F_MIN == 0.5
    assert not ink.tip(None, 2.0) and not ink.local                 # no contact reported: no ink
    assert not ink.tip(local, 0.0) and not ink.local                # touching without force: no ink
    assert not ink.tip(local, 0.45) and not ink.local               # below 0.5 N: no ink
    assert ink.tip(local, 0.6) and len(ink.local) == 1 and ink.label[0]
    assert ink.tip(local + t * 0.003, 0.9) and len(ink.local) >= 6  # a 3 mm stroke: dots every 0.5 mm
    assert ink.tip(local + t * 0.006, 5.0)                          # harder than handwriting: still inks
    assert not ink.tip(local + [0, 0, 0.2], 1.0)                    # a contact off the side wall (lid): no ink


def test_only_nib_contacts_count():
    tip = np.array([0.1, 0.2, 0.3])
    f, p = task.nib_contact([0.3, 0.4, 2.0], [tip + [0.001, 0, 0], tip - [0.001, 0, 0], tip + [0.02, 0, 0]], tip)
    assert abs(f - 0.7) < 1e-9 and np.linalg.norm(p - tip) < 0.001  # the barrel's 2 N contact 20 mm away is left out
    f, p = task.nib_contact([1.0], [tip + [0.0, 0.01, 0.0]], tip)
    assert f == 0.0 and p is None


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
