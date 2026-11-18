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


# -- handwriting not drawn from the reader's forms ------------------------------------------------------------------
# Hand-made letterforms (cell of height 1), drawn independently of task.FORMS: other proportions and curves, the
# common variants people write, each as its strokes in the order of the human rule (README, "Character credit").


def _arc(cx, cy, rx, ry, a0, a1, n=24):
    a = np.radians(np.linspace(a0, a1, n))
    return np.column_stack([cx + rx * np.cos(a), cy + ry * np.sin(a)])


def _line(*pts):
    return np.array(pts, float)


def _cat(*parts):
    return np.concatenate(parts)


HAND = {   # char: (width, {form name: [strokes]})
    "0": (0.55, {"oval": [_arc(0.28, 0.5, 0.25, 0.49, 100, 460, 40)],
                 "slashed": [_arc(0.28, 0.5, 0.25, 0.49, 100, 460, 40), _line((0.47, 0.9), (0.08, 0.1))]}),
    "1": (0.3, {"plain": [_line((0.17, 1.0), (0.13, 0.0))],
                "flag": [_cat(_line((0.0, 0.78)), _line((0.17, 1.0), (0.13, 0.0)))],
                "base": [_line((0.17, 1.0), (0.13, 0.0)), _line((0.0, 0.01), (0.29, 0.0))]}),
    "2": (0.58, {"round": [_cat(_arc(0.29, 0.71, 0.27, 0.27, 165, -30), _line((0.04, 0.03))),
                           _line((0.04, 0.02), (0.58, 0.03))],
                 "looped": [_cat(_arc(0.29, 0.71, 0.27, 0.27, 165, -30), _line((0.1, 0.12)),
                                 _arc(0.13, 0.09, 0.06, 0.07, 140, 500, 16)), _line((0.15, 0.03), (0.6, 0.02))],
                 "flat": [_line((0.03, 0.98), (0.5, 1.0), (0.02, 0.01)), _line((0.02, 0.01), (0.57, 0.02))]}),
    "3": (0.55, {"round": [_arc(0.27, 0.74, 0.25, 0.25, 155, -95), _arc(0.27, 0.26, 0.27, 0.26, 92, -155)]}),
    "4": (0.58, {"closed": [_line((0.42, 1.0), (0.02, 0.33)), _line((0.0, 0.33), (0.58, 0.32)),
                            _line((0.43, 0.98), (0.43, 0.0))],
                 "open": [_line((0.06, 1.0), (0.02, 0.33)), _line((0.0, 0.33), (0.58, 0.32)),
                          _line((0.42, 0.85), (0.42, 0.0))]}),
    "5": (0.55, {"plain": [_line((0.53, 1.0), (0.06, 0.99)), _line((0.06, 1.0), (0.03, 0.55)),
                           _cat(_line((0.03, 0.57)), _arc(0.27, 0.31, 0.27, 0.29, 125, -155))]}),
    "6": (0.55, {"curved": [_cat(_line((0.45, 1.0)), _arc(0.55, 0.3, 0.55, 0.7, 120, 180, 12)),
                            _arc(0.28, 0.29, 0.27, 0.29, 180, 540, 30)],
                 "straight": [_line((0.43, 1.0), (0.02, 0.32)), _arc(0.28, 0.29, 0.27, 0.29, 180, 540, 30)]}),
    "7": (0.55, {"plain": [_line((0.0, 0.99), (0.55, 1.0)), _line((0.55, 1.0), (0.2, 0.0))],
                 "crossed": [_line((0.0, 0.99), (0.55, 1.0)), _line((0.55, 1.0), (0.2, 0.0)),
                             _line((0.18, 0.5), (0.52, 0.52))]}),
    "8": (0.55, {"plain": [_arc(0.28, 0.76, 0.21, 0.23, -90, 270, 30), _arc(0.28, 0.27, 0.27, 0.27, 90, -270, 30)]}),
    "9": (0.55, {"curved": [_arc(0.28, 0.73, 0.26, 0.26, 0, 360, 30), _line((0.54, 0.73), (0.52, 0.35), (0.3, 0.0))],
                 "straight": [_arc(0.28, 0.73, 0.26, 0.26, 0, 360, 30), _line((0.54, 0.73), (0.54, 0.0))]}),
    "/": (0.3, {"plain": [_line((0.3, 1.0), (0.01, 0.0))]}),
    "-": (0.35, {"plain": [_line((0.0, 0.5), (0.35, 0.51))]}),
}


def hand(text, forms=None, h=30.0, slant=0.0, width=1.0, gaps=None, size=None, drift=None, wobble=0.5, seed=0,
         drop=None, ds=0.0, dz=0.0):
    """Ink of ``text`` handwritten with HAND forms (``forms`` {index: name}, default the first), ``h`` mm tall,
    leaning ``slant`` degrees, ``width`` times as wide, ``gaps`` (mm before each character after the first), ``size``
    (each character's height factor), ``drift`` (mm each character sits above the line), a slow ``wobble`` (mm) along
    the strokes plus 0.3 mm of jitter, strokes ``drop`` {index: [stroke numbers]} left out. Centred on the label."""
    rng = np.random.default_rng(seed)
    n = len(text)
    gaps = gaps if gaps is not None else [0.12 * h] * (n - 1)
    size, drift, forms, drop = size or [1.0] * n, drift or [0.0] * n, forms or {}, drop or {}
    t = math.tan(math.radians(slant))
    chars, s = [], 0.0
    for i, c in enumerate(text):
        wd, fs = HAND[c]
        strokes = fs[forms.get(i, next(iter(fs)))]
        hh = h * size[i]
        for k, st in enumerate(strokes):
            if k in drop.get(i, ()):
                continue
            p = task.resample(st, 0.02)
            x = wd / 2 + (p[:, 0] - wd / 2) * width
            pts = np.column_stack([s + x * hh + t * (p[:, 1] - 0.5) * hh, p[:, 1] * hh + drift[i]])
            u = np.linspace(0, 1, len(pts))
            pts += wobble * np.column_stack([np.sin(2 * math.pi * (u * rng.uniform(0.7, 1.5) + rng.uniform())),
                                             np.cos(2 * math.pi * (u * rng.uniform(0.7, 1.5) + rng.uniform()))])
            chars.append(task.resample(pts, 0.5) + rng.normal(0, 0.3, (len(task.resample(pts, 0.5)), 2)))
        s += wd * width * hh + (gaps[i] if i < n - 1 else 0.0)
    ink = np.concatenate(chars)
    return ink + [-s / 2 + ds, task.Z_MID - h / 2 + dz]


def test_handwritten_forms_read():
    for c, (_, fs) in HAND.items():
        for name in fs:
            if c in "/-":
                text, idx = ("11/22", 2) if c == "/" else ("11-22", 2)
            else:
                text, idx = f"1{c}/2{c}", 1
            forms = {idx: name, idx + 3: name} if c not in "/-" else {idx: name}
            r = task.read_label(hand(text, forms, seed=len(name)), text)
            assert r["text"] == 1.0, (c, name, text, r)


def test_slant_width_spacing_and_size_vary():
    rng = np.random.default_rng(7)
    cases = [dict(slant=-20), dict(slant=-10), dict(slant=10), dict(slant=20), dict(width=0.75), dict(width=1.35),
             dict(gaps=[1.0, 1.0, 1.0, 1.0]), dict(gaps=[10.0, 10.0, 10.0, 10.0]), dict(gaps=[2.0, 9.0, 3.0, 8.0]),
             dict(h=25.0), dict(h=35.0), dict(size=[1.0, 0.9, 1.1, 0.92, 1.08]), dict(drift=[0, 2, -2, 1.5, -1.5]),
             dict(slant=15, width=1.2, gaps=[2.0, 6.0, 4.0, 7.0], size=[1.05, 0.95, 1.0, 1.08, 0.94],
                  drift=[1, -1, 0, 2, -2], forms={3: "looped", 4: "looped"}),
             dict(slant=-12, width=0.8, gaps=[5.0, 3.0, 8.0, 2.0], forms={0: "flag", 1: "base", 3: "flat", 4: "flat"})]
    for k, kw in enumerate(cases):
        r = task.read_label(hand("11/22", seed=k, **kw))
        assert r["text"] == 1.0, (kw, r)
    for k, d in enumerate(("03/09", "12/05", "10/31", "04/17", "08/26", "06/28", "09/30", "05/16", "02/27")):
        kw = dict(slant=float(rng.uniform(-18, 18)), width=float(rng.uniform(0.8, 1.25)),
                  gaps=list(rng.uniform(1.5, 8.0, 4)), size=list(rng.uniform(0.92, 1.08, 5)),
                  drift=list(rng.uniform(-1.5, 1.5, 5)))
        r = task.read_label(hand(d, seed=k, **kw), d)
        assert r["text"] == 1.0, (d, kw, r)


def test_handwriting_scored_by_the_stroke_rule():
    """Leave strokes out of varied handwriting: the reader's credit is what the human rule gives (strokes drawn /
    strokes of the character)."""
    kw = dict(slant=12, width=1.15, gaps=[3.0, 6.0, 4.0, 7.0])
    for forms, drop, credit in (({4: "looped"}, {4: [1]}, [1, 1, 1, 1, 0.5]),     # looped "2" without its base
                                ({3: "round"}, {3: [1]}, [1, 1, 1, 0.5, 1]),       # round "2" without its base
                                ({}, {0: [0]}, [0, 1, 1, 1, 1])):                  # one "1" left out
        r = task.read_label(hand("11/22", forms, drop=drop, **kw))
        assert abs(r["text"] - np.mean(credit)) < 1e-9, (forms, drop, r)
        if 0 not in drop:
            assert r["chars"] == credit, (forms, drop, r)
        else:   # "1/22" for "11/22": either "1" is the missing one, the score is the same
            assert sorted(r["chars"][:2]) == [0.0, 1.0] and r["chars"][2:] == [1.0, 1.0, 1.0], r
    r = task.read_label(hand("11/22", {3: "flat"}, drop={3: [1]}, **kw))            # flat-top "2" without its base
    assert r["chars"][3] == 0.0 and r["reads"][3] == "7", r                          # is a "7", as a person reads it
    r = task.read_label(hand("04/14", {1: "open", 4: "open"}, drop={4: [2]}, slant=-10, width=0.9), "04/14")
    assert abs(r["chars"][4] - 2 / 3) < 1e-9 and min(r["chars"][:4]) == 1.0, r


def test_varied_handwriting_of_other_dates_fails():
    kw = dict(slant=14, width=1.15, gaps=[3.0, 7.0, 4.0, 6.0])
    for target, text, forms in (("11/22", "11/23", {3: "looped"}), ("11/22", "17/22", {1: "crossed"}),
                                ("11/22", "11/27", {3: "flat"}), ("11/22", "11-22", {}), ("03/09", "08/09", {}),
                                ("10/31", "10/37", {4: "crossed"}), ("06/28", "05/28", {}),
                                ("09/30", "04/30", {1: "open"}), ("12/05", "12/06", {4: "straight"})):
        r = task.read_label(hand(text, forms, **kw), target)
        assert r["text"] <= 0.8 + 1e-9 and min(r["chars"]) == 0.0, (target, text, r)
