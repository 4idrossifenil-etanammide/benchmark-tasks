# Replace the worn rubber in a window squeegee channel

A 14 in window squeegee channel is held by its middle in a small vise with soft jaw caps, back down, blade up, with
a worn rubber in it held by a brass end clip at each end. A new rubber stands on its edge on the table beyond it, two
new brass end clips lie between the channel and the robot, and a shallow waste tray stands on the right. The robot must take the worn rubber out of the channel (it
slides out along the channel, its clips with it) and lay it in the tray, slide the new rubber through the channel from
one end until both of its ends are at the channel's ends, and put a clip on each end.

**Instruction:** `Replace the worn rubber in the squeegee channel with the new rubber and clip it in.`

## Why this task

Cleaning windows and glass with a squeegee is a listed task of janitors and cleaners (O*NET 37-2011.00: "Clean
windows, glass partitions, or mirrors, using soapy water or other cleaners, sponges, or squeegees"); the BLS
Occupational Outlook Handbook counts 2,447,700 jobs for janitors and building cleaners in 2024. A squeegee only
leaves glass streak-free while its rubber edge is sharp, so professionals change the rubber often (a high-rise window cleaner
quoted on Ettore's blog: daily when it hits concrete or debris, every two or three days on residential work).

The rubber is a long, flexible strip: it bends as soon as it is held anywhere but at the channel, it has to be lined
up with a 7.2 mm wide channel at one end and pushed or drawn through 35 cm of it, and it only goes in and out along
the channel. The old one is held by its clips until pulled out of the channel's ends. The clips go on at the two
ends, 35 cm apart, one hand at each.

**Why a vise.** A window cleaner holds the channel (or the squeegee handle it is locked in) in one hand and works the
rubber with the other. The robot's arms are compliant: a gripper holding a loose channel yields several millimetres
under the 1 to 3 N it takes to pull the rubber through, so the channel is dragged along and the rubber jams against
its 0.6 mm side play. The channel is therefore held the way a workshop holds a part it works on: in a vise with soft
jaw caps (so the stainless channel is not marked). This replaces the cleaner's holding hand; everything done to the
rubber and the clips is the same.

## Hardware

- **Channel:** Ettore Master clipped channel, 14 in, stainless steel (Ettore 1173, "Master 14 in stainless steel
  clipped channel with rubber"; the brass Master channel is the same size).
- **Rubber:** Ettore Master replacement rubber, 14 in (Ettore 1418 in the 144-pack).
- **Clips:** Ettore brass end clips for window squeegee channels (sold separately; one at each end of the channel).
- **Vise:** Wilton 11744 4 in low-profile drill-press vise (Wilton data sheet: jaw width 4 in, jaw depth 1-1/2 in,
  maximum opening 4 in, height 2-7/8 in, 6 x 11-1/2 in, 12 lb), standing on the table.
- **Soft jaws:** Yost MR-340 4 in magnetic aluminium vise jaw caps with a rubber face, on both jaws.

Lengths from Ettore's rubber-change instructions ("Rubbers should be 1/8 in longer than the channel (not stretched).
The assembled channel and rubber should measure 1/4 in less in overall length than the designated model size"):

| | Value | Simulation |
|---|---|---|
| Rubber length, 14 in model | 13-3/4 in (349.3 mm) | 349.3 mm |
| Channel length | 1/8 in shorter: 346.1 mm | 346.1 mm |
| Rubber beyond each channel end | 1/16 in (1.6 mm) | 1.6 mm |

Ettore does not publish the cross-sections; these are **our assumptions**: rubber 6 mm thick x 12 mm tall, held 7 mm
deep in the channel (5 mm of blade stands out); channel inside 7.2 mm wide (0.6 mm play each side), 0.8 mm stainless
sheet, so 8.8 mm wide and 7.8 mm tall outside; clip 10 mm long, a brass bridge on the rubber's top edge with two
1.5 mm legs down its sides and a small lead-in flare. Masses: rubber 1.2 g/cm3 (30 g), channel 50 g (stainless,
from the section), clip 2 g.

## Scene (positions in cm from the midpoint between the arm bases, on the table top; x forward, y left)

- Vise (fixed) standing on the table, its jaws along y, centred at x 30, y 0, its fixed jaw toward the robot and its
  screw running away from it (bed from x 27 to 56, y -7.6 to 7.6). Jaw tops 7.3 cm above the table.
- Channel along y, centred at x 30, y 0, back down, blade up, gripped by its middle between the soft jaws: its lower
  5 mm is between the jaws, so its back is 6.8 cm above the table and 2.8 mm of its sides stand above the jaws (**our
  assumption** of how it is clamped). Worn rubber in it, blade up, its ends 1.6 mm beyond the channel's ends; a brass
  clip on the rubber's top edge at each end.
- New rubber standing on its 6 mm edge along y at x 42, from y -52 to y -17.
- New clips standing on their legs at (20, 10) and (20, -10).
- Waste tray, grey plastic, fixed: inside 8 x 40 cm (x 15 to 23, y -57 to -17), 15 mm walls, 2 mm floor.

## Scoring (0 to 1, at the end of the run; success = 1.0 held 1 s)

- 0 if anything (channel, either rubber, a clip) fell off the table.
- Worn rubber: 0.10 once no part of it is in the channel; +0.15 if all of it lies in the tray.
- New rubber: 0.15 x the fraction of its length seated in the channel (down on the channel's back, between its
  sides); +0.20 once all of it is seated with each end between 2 mm inside and 5 mm beyond the channel's end
  (Ettore: 1.6 mm beyond each end).
- Clips (only once the new rubber is fully in): +0.15 for each end with a clip seated on it: on the rubber's top edge
  (within 1.2 mm of centred, resting on it), its outer face within 4 mm of the rubber's end.
- +0.10 when all of the above is done and both hands are clear of the channel (5 cm above the channel or 5 cm away from it in plan).
- A rubber that goes into or out of the channel over its edge (more than 3 cm from both ends) gets nothing: the real
  channel's folded edges hold the rubber, which only slides in and out at the ends. The worn rubber left in, or put
  back, scores nothing for it, and leaves no room for the new one.

Doing nothing scores 0.

## How it is modelled (fidelity)

Approximate. Rigid-body PhysX (Isaac Sim 6.1, 240 Hz physics, 20 Hz control).

- **Rubber:** 10 rigid 34.9 mm segments joined by spring hinges about the vertical (in the channel) axis, so it bends
  through its 6 mm thickness: hinge stiffness EI/L = 0.025 N m/rad from E = 4 MPa (about Shore 60A, **our
  assumption**). It does not bend the other way (through its 12 mm height), does not twist and does not stretch, so
  held by one end it stays straight in that direction instead of drooping. It slides in the channel on contact and
  friction only (rubber 0.9, steel 0.5).
- **Clips:** the worn rubber's clips are part of its end segments. Their grip on the channel's ends is one slide joint
  between the rubber and the channel with breakaway friction: 1 N for the two clips (**our assumption**, not
  measured); 4 mm out of the seat (or 3 mm further in) it lets go. The new clips are loose parts: a clip counts when
  it rests seated on the new rubber's top edge at an end; nothing holds it to the rubber, and its grip on the channel
  is not modelled (nothing pulls on the new rubber after it is clipped).
- **Not stretched:** Ettore's procedure stretches the rubber slightly: "Pull rubber from the other end far enough to
  put on the other clip" and, when taking it out, "Pull rubber enough to take the clip off". The rubber here does not
  stretch, so the clips go on with the rubber already in place (each end 1.6 mm out), and the worn rubber comes out
  with its clips on (it goes in the tray with them).
- The real clip is pushed in between the rubber and the channel; here it sits on the 5 mm of blade that stands out of
  the channel, at the end.
- **Vise:** fixed boxes (bed, jaws, aluminium caps with a 1.5 mm rubber face; cap and jaw-plate thickness are **our
  assumption**, Yost does not publish them). The channel it grips is held fixed (a kinematic body): its 5 kg vise
  does not move under the robot's few newtons, and the clamped channel is not modelled slipping in the jaws.

## Scripted demo

A fixed plan of tool-centre-point waypoints (nothing read from the simulation). The right hand pinches the worn
rubber's blade near its right end, pulls the clips out of the channel's ends, slides the worn rubber out in two strokes
and lays it in the tray. It picks up the new rubber 8 cm from one end (close enough to its middle that the strip stays
level), holds it in line with the channel with that end 1.5 cm in over the channel's right end, lowers the end into
the channel, slides the rubber in, then pinches it again at the channel's right end and slides it on until it stands
3 cm out of the left end. The
left hand pushes that end back with its closed fingers, held low enough that the channel's end stops them, so the
rubber ends up within a few millimetres of flush with the channel's left end, and its right end 1.6 to 3.2 mm out
(each end must be from 2 mm inside to 5 mm beyond its channel end). Meanwhile the right picks up a clip; each hand then sets a clip on the rubber's top edge at its end,
presses it on and moves away. Some waypoints were taught in simulation, as one would teach a robot: the line-up over
the channel's end is 1.05 mm off the channel's centre line and the right hand's clip 1.0 mm off it (the loaded arm sits
about 1 mm short there; the rubber has 0.6 mm of play each side), and each clip is held 4 mm over the blade for 0.8 s
before it is set down. The worn rubber's first stroke is 13 cm, so that most of it is still in the channel when the
hand lets go and takes a new grip.

## Verified in simulation

| Run | Host / GPU | Steps | Score |
|---|---|---|---|
| Scripted demo (task.py sha256 e31719a66c9f2eab) | shiva, RTX A6000 (GPU 2) | 1,580 of 2,400 (success) | 1.0 |
| Dream-1 (same task.py) | shiva, RTX A6000 (GPU 2) | 2,400 of 2,400 (step limit) | 0.0: both arms rise and hover near their bases for the whole run, touching neither rubber nor the clips; the worn rubber stays in the channel |

The worn rubber comes out with its clips and lies in the tray; the new rubber is threaded in from the channel's right
end, pushed back against the left end (its ends end up about 1.5 and 2.5 mm out), both new clips are seated on its ends and both
hands are clear when the run ends.

## Files

- `task.py`: scene, clip grip, scoring and the scripted demo.
- `jointkit.py`: multi-box parts and spring joints (with two additions: a joint frame may be a full rotation, and a
  part may be kinematic, held where it is spawned).
- `test_squeegee_rubber.py`: checks that need no GPU (`python -m pytest -q`).
