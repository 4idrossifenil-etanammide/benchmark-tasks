# Write the use-by date on a food container's label with a marker

A 32 oz round deli container of tomato sauce with its lid on stands on the table. A strip of masking tape is stuck
around its side, facing the right arm, as a blank label. A black fine-point permanent marker, cap already off, lies on
the table to its right. The robot must write the use-by date given in the instruction on the tape, legibly, and put the marker
down. The container is light and stands on a small, tapered base, so it has to be held steady while it is written on.

**Instruction:** `Write the use-by date 11/22 on the container's label.`

The date is a parameter: `TEXT` (environment variable, default `11/22`; digits, `/` and `-`) changes both the
instruction and what the label is scored against, so it can be varied per trial.

## Why this task

Date marking prepared food is a core, many-times-a-day duty of prep cooks, line cooks and food preparation workers
(O*NET 35-2014 Cooks, Restaurant; 35-2021 Food Preparation Workers). The FDA Food Code, section 3-501.17
("Ready-to-Eat, Time/Temperature Control for Safety Food, Date Marking"), requires refrigerated ready-to-eat TCS food
prepared and held in a food establishment for more than 24 hours to be clearly marked with the date or day by which
it must be consumed, sold or discarded (at most 7 days at 5 C / 41 F, the day of preparation counting as day 1).
Most US states adopt it (for example Minnesota Rules 4626.0400). In most kitchens the mark is written by hand with a
permanent marker on a strip of masking tape on the container's side.

For a robot this is handwriting on a curved, tapered, loose object: it has to pick up a thin tool and hold it so
that its tip, 6 cm ahead of the fingers, stays in light contact with a surface that curves away from it (the text
runs over about 90 degrees of a 10 cm container), lift and place the tip between strokes, and keep the shapes
readable; the other hand has to keep the container from sliding or turning under the pen. Pressing too hard pushes the
container away, too lightly leaves gaps in the strokes.

## Hardware

- **Container:** Bare by Solo DM32R-0090 (Dart Container), 32 oz round deli container, clear recycled PET, with its
  recessed lid. Data sheet dimensions (distributor listing): top diameter 4.61 in (117.1 mm), bottom diameter 3.47 in
  (88.1 mm), wall height 5.71 in (145.0 mm). Retail listings give 4-5/8 in, 3-1/2 in and 5-3/4 in.
- **Tape:** 3M Scotch 2050 general purpose masking tape, 48 mm (1.88 in) wide, beige crepe paper.
- **Marker:** Sharpie Fine Point permanent marker, black (30001), cap off. Retail listings: 5.5 in (137 to 140 mm)
  long capped, 12.3 cm uncapped, barrel about 10 to 11.5 mm, line about 1 mm (fine bullet tip).

| Data | Value | Simulation |
|---|---|---|
| Container top / bottom diameter | 117.1 / 88.1 mm | same (40-sided, the facets tangent to the real cone) |
| Container height | 145.0 mm | 145.0 mm, plus an 8 mm lid |
| Tape width | 48 mm | 48 mm band, 51 to 99 mm above the table |
| Marker length uncapped | 123 mm | 123 mm |
| Marker barrel | 10 to 11.5 mm | 11 mm (12-sided) |
| Line width | about 1 mm | 1 mm dots |

## Design

| Object | Where | Mass | Notes |
|---|---|---|---|
| Container | centre x 30, y 0 cm, on a taped outline | 0.5 kg with sauce and lid | Free; friction 0.5; drawn as red sauce behind clear plastic below 10 cm, clear above |
| Label | tape strip about 10.5 cm long around the side, centred on the side facing the right arm (front right, 45 degrees from -x toward -y) | in the container | 0.1 mm proud of the wall |
| Marker | lying along x, tip forward (+x), its middle (6 cm behind the tip) at x 20, y -17 cm | 11 g | Free; friction 0.6 |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

**Ink.** 60 times a second the marker's tip is checked against the container's side. When it is within 0.5 mm of the
surface (touching), a dot of ink is laid there, and when it slides along the surface the dots are filled in every
0.5 mm. Dots stay on the container (they move with it) and are drawn as 1 mm black patches on the surface (visual
only, no collision). Ink on the tape counts for the reading; ink on the container off the tape counts against.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). It stands in for a
person reading the label:

| Condition | Score |
|---|---|
| The container was tipped over (20 degrees) or pushed more than 3 cm off its outline, at any time | 0 |
| More than 20 mm of line on the container off the tape | 0 |
| Otherwise | 0.8 x the mean character credit, + 0.2 if every character is fully legible and the marker lies on the table, let go |

**How the label is read (the proxy).** The date's characters have stroke skeletons (simple handwritten shapes: `1` a
vertical line, `2` a hook, a diagonal and a base line, and so on; `task.GLYPHS`). The reader:

1. fits the whole date to the ink: character height 24 to 36 mm, any place on
   the tape, choosing the fit that covers the most of the skeletons;
2. lets each character move up to 4 mm from its place in the line (uneven spacing);
3. reads each character: of all the skeletons (digits, `/`, `-`), the one the ink in its cell matches best (min of
   "share of the skeleton covered by ink within 2.5 mm" and "share of the ink within 3 mm of the skeleton") must be
   the right one;
4. gives each character credit 0 to 1 by how much of its skeleton is covered (full credit at 90 %, none at 40 %),
   and 0 if more than 25 % of the ink in its cell is stray (farther than 3 mm from the skeletons);
5. gives the text 0 if more than 20 % of all the ink on the tape is stray.

The reader works on the inked area (0.5 mm cells), not on how the ink got there: going over a line again changes
nothing, as for a person looking at the label afterwards.

Doing nothing scores 0. Scribbling over the label, filling it, or writing another date scores 0 for every wrong
character (tested: `88/88` scores 0; `11/23` gets 0 for the last character). Writing the right date but leaving the
marker in the hand scores 0.8. The tests (`test_date_label_marker.py`) check these cases.

## Verified in simulation

| Run | Host / GPU | Steps | Score |
|---|---|---|---|
| Scripted demo (task.py sha256 4c456fec674f2bef) | shiva, RTX A6000 (GPU 1) | 955 of 1,600 (success) | 1.0 |
| Dream-1 (same task.py) | shiva, RTX A6000 (GPU 1) | 1,600 of 1,600 (step limit) | 0.0: both arms rise and hover near their bases for the whole run; the right arm never picks up the marker and no ink reaches the label |

Scripted demo: the left hand puts its closed fingers against the back of the container, across from the label, about
10 cm up, pressing 3 mm in, and holds it there for the whole writing (the pen pushes the container toward it). The right hand picks the marker up by the barrel
6 cm behind the tip, brings it to the tape and writes the date 30 mm tall, centred on the tape, stroke by stroke: the
tip is put on the tape at the start of each stroke, commanded 3 mm into the surface (the arm is compliant, so it
presses lightly), moved along the stroke at 12.5 mm/s and lifted 15 mm off between strokes. The marker is turned with the
stroke so that it always points at the container's axis. The right hand then lays the marker back where it was and
lets go. It plays a fixed plan; it reads no simulation state.

## Still approximate (our assumptions)

- **Mass** of the container with its sauce (0.5 kg, about half full) and the **lid** (8 mm thick, 120 mm across) are
  our estimates.
- **Ink:** no pressure, angle, speed or ink-flow model; ink is laid wherever the tip is within 0.5 mm of the
  surface (our threshold). The tape is crepe paper and does not wrap a cone perfectly flat; here it is a flat band.
- **Reader:** the stroke-skeleton match is a proxy for a person reading the label. Skeletons are one simple handwritten
  form per character; very different but readable letterforms (for example a `1` with a long base) may score lower.
- **Rendering:** the container wall is drawn opaque (the sauce's colour behind clear plastic).
- Round parts are flat-faceted prisms.
