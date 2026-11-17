# Write the use-by date on a food container's label with a marker

A 32 oz round deli container filled with water to 0.5 kg, its lid on and taped shut, stands on the table. A strip of
masking tape is stuck around its side, facing the right arm, as a blank label. A black fine-point permanent marker, cap
already off, lies on a sheet of paper on the table to its right. The robot must write the use-by date given in the
instruction on the tape, legibly, and put the marker down. The container is light and stands on a small, tapered base, so it has to be held steady while it is written on.

**Instruction:** `Write the use-by date 11/22 on the container's label.`

The date is a parameter: `TEXT` (environment variable, default `11/22`; digits, `/` and `-`) changes both the
instruction and what the label is scored against, so it can be varied per trial.

## Why this task

Date marking prepared food is routine in every commercial kitchen. The FDA Food Code, section 3-501.17
("Ready-to-Eat, Time/Temperature Control for Safety Food, Date Marking"), requires refrigerated ready-to-eat TCS food
prepared and held in a food establishment for more than 24 hours to be clearly marked with the date or day by which
it must be consumed, sold or discarded (at most 7 days at 5 C / 41 F, the day of preparation counting as day 1).
Most US states adopt it (for example Minnesota Rules 4626.0400). In most kitchens the mark is written by hand with a
permanent marker on a strip of masking tape on the container's side.

O*NET (release 30.0, Task Statements) lists this work among the occupations' tasks:

| Occupation (O*NET-SOC) | O*NET task | US workers |
|---|---|---|
| Cooks, restaurant (35-2014) | "Ensure freshness of food and ingredients by checking for quality, keeping track of old and new items, and rotating stock." | 1,409,890 |
| Cooks, short order (35-2015) | "Restock kitchen supplies, rotate food, and stamp the time and date on food in coolers." | 138,650 |
| Baristas (35-3023.01) | "Wrap, label, or date food items for sale." | part of fast food and counter workers (35-3023), 3,854,050 |
| Cooks, institution and cafeteria (35-2012) | "Rotate and store food supplies." | 441,050 |

Date marks are how old and new items are told apart and stock is rotated in a kitchen under the Food Code.
The three cook occupations alone total 1,989,590 US workers. Sources: O*NET 30.0 Task Statements
(https://www.onetcenter.org/dl_files/database/db_30_0_text/Task%20Statements.txt); employment from BLS OEWS, May 2025
(https://www.bls.gov/oes/).

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
- **Contents:** about 470 ml of room-temperature tap water (not food: nothing perishable, the same every trial). Fill
  until container, water and lid weigh 500 g on a kitchen scale, press the lid on and seal it with a strip of masking
  tape around the rim. The contents only set the mass that the other hand must hold steady.
- **Paper:** one sheet of 20 lb (75 g/m2) copy paper, US letter folded in half (216 x 140 mm), long side along x,
  centred under the marker and taped to the table at its corners. It keeps the ink off the shared table where the
  marker is picked up and laid down. The grippers hold the marker only by its barrel, away from the tip (the demo
  grips it 6 cm behind the tip), so the pads never touch the ink.

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
| Container | centre x 30, y 0 cm, on a taped outline | 0.5 kg with water and lid | Free; friction 0.5; drawn light blue-grey (water behind clear plastic) below 10 cm, clear above |
| Label | tape strip about 10.5 cm long around the side, centred on the side facing the right arm (front right, 45 degrees from -x toward -y) | in the container | 0.1 mm proud of the wall |
| Marker | lying along x, tip forward (+x), its middle (6 cm behind the tip) at x 20, y -17 cm, on the paper | 11 g | Free; friction 0.6 |
| Paper | 21.6 x 14 cm, 0.1 mm, centred under the marker, long side along x | - | Fixed (taped down) |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

**Ink: physical contact and writing force.** The marker is a rigid body whose contacts PhysX reports (contact report
API). Every physics step (240 Hz) a PhysX rigid contact view between the marker and the container
(`create_rigid_contact_view` / `get_contact_data`, the same engine data Isaac Lab's contact sensor uses) gives each
contact point between them with its normal force. The contact points on the nib (within 3 mm of the tip) are summed:
that is the nib's normal force on the container; contacts of the barrel or any other part of the marker do not count.
60 times a second, a dot of ink is laid where the nib touches (force-weighted mean of its contact points) when the
nib's mean normal force over the last 4 physics steps is at least **0.5 N**; while it stays pressed and slides, the
dots are filled in every 0.5 mm. Touching more lightly, hovering close to the surface or a contact anywhere else leaves
no ink. Dots stay on the container (they move with it) and are drawn as 1 mm black patches on the surface (visual
only, no collision). Ink on the tape counts for the reading; ink on the container off the tape counts against.

*Where 0.5 N comes from.* Schomaker, L.R.B. and Plamondon, R. (1990), "The relation between pen force and pen-point
kinematics in handwriting", Biological Cybernetics 63, 277-289, measured the axial pen force of 16 adults writing
on paper with a force-sensing ball-point stylus. Their Table 2 gives a mean of 109.7 g (1.08 N) with a variance of
806 g^2 (SD 28 g) for cursive handwriting, and means of 84 to 115 g (0.82 to 1.13 N) for all six writing and drawing
conditions. The ink threshold is the mean minus two SD, 53 g = 0.52 N, rounded to 0.5 N. The paper gives no force
above which a pen stops writing, so there is no upper limit: a harder press also inks (mean + 2 SD = 1.64 N is the
upper end of normal handwriting force). The pen here points at the container's axis, so the nib's normal force is
along the marker, like the axial force they measured. A felt-tip marker inks at lower force than a ball-point; we
keep the handwriting value.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`). It stands in for a
person reading the label:

| Condition | Score |
|---|---|
| The container was tipped over (20 degrees) or pushed more than 3 cm off its outline, at any time | 0 |
| More than 20 mm of line on the container off the tape | 0 |
| More than a fifth of the ink on the tape belongs to no character (farther than 3 mm from every character's strokes) | 0 |
| Otherwise | 0.8 x the mean character credit, + 0.2 if every character has all its strokes and reads as itself and the marker lies on the table (or paper), let go |

**Character credit (the same rule for a person and for the simulation).** Each character is made of strokes; its
credit is the number of its strokes drawn divided by its number of strokes:

| Character | Strokes | Which |
|---|---|---|
| `0` | 1 | the oval |
| `1` | 1 | the upright (a flag at the top and/or a base line may be added; they are not counted) |
| `2` | 2 | hook with diagonal; base line |
| `3` | 2 | upper bowl; lower bowl |
| `4` | 3 | diagonal (closed top) or left upright (open top); crossbar; upright |
| `5` | 3 | top bar; short upright; bowl |
| `6` | 2 | stem; loop |
| `7` | 2 | top bar; diagonal (a cross bar may be added; not counted) |
| `8` | 2 | upper loop; lower loop |
| `9` | 2 | loop; stem |
| `/` | 1 | the slash |
| `-` | 1 | the bar |

A stroke is drawn when at least 80 % of its length is inked: the gaps along it, measured with a ruler, add up to no
more than 20 % of its length. A character scores 0 if it reads as another character (for example a `7` where a `1`
should be, or a `3` where an `8` should be, even if some strokes of the right one are there), or if more than a quarter
of its ink is off its strokes (scribbled over or crossed out). Characters 24 to 36 mm tall, anywhere on the tape,
count. Examples: `11/22` with the last `2`'s base line missing scores 0.8 x (1 + 1 + 1 + 1 + 0.5) / 5 = 0.72; with an
open-top `4` missing its upright, that `4` gets 2/3.

**How the simulation reads the label.** Every accepted form of each character has stroke skeletons (`task.FORMS`; the
first form of each is what the scripted demo writes; `1` also with a flag, a base or both, `4` also open-top, `7` also
crossed). The reader:

1. fits the whole date to the ink: character height 24 to 36 mm, any place on the tape (ties broken by the closest
   fit);
2. places each character's forms within 4 mm of its place in the line (uneven spacing);
3. divides the ink: each inked 0.5 mm cell goes to the nearest character; farther than 3 mm from every character's
   strokes it is stray;
4. counts, for each character, its strokes drawn by its own share of the ink: a stroke is drawn when 80 % of the points
   along it have ink within 2.5 mm; it keeps the form with the most strokes drawn;
5. gives 0 to a character if more than 25 % of its share of the ink is farther than 3 mm from its strokes, or if its
   ink reads as another character: some other character (any form, placed within 6 mm) has all its strokes drawn by
   that ink and explains (ink within 3 mm) more of the ink around it than the intended character does (5 points more;
   or as much, if the intended character is not complete or if a clearly larger share (10 points) of the other
   character's stroke length is drawn);
6. gives the text 0 if more than 20 % of all the ink on the tape is stray.

The reader works on the inked area (0.5 mm cells), not on how the ink got there: going over a line again changes
nothing, as for a person looking at the label afterwards.

Doing nothing scores 0. Scribbling over the label, filling it, or writing another date scores 0 for every wrong
character (tested: `88/88` scores 0; `11/23`, `17/22`, `11-22` and every other single wrong digit get 0 for the wrong
character; dates such as `03/09`, `12/05`, `10/31` read in full when written as shown). Writing the right date but
leaving the marker in the hand scores 0.8. The tests (`test_date_label_marker.py`) check these cases.

## Verified in simulation

| Run | Host / GPU | Steps | Score |
|---|---|---|---|
| Scripted demo (task.py sha256 4e5bf641cfd9d4e6) | shiva, RTX A6000 (GPU 1) | 1,319 of 1,600 (success) | 1.0 |
| Dream-1 (same task.py) | shiva, RTX A6000 (GPU 1) | 1,600 of 1,600 (step limit) | 0.0: the right arm reaches the marker on its paper sheet and turns it but never picks it up; no ink reaches the label |

In this run every character has all its strokes drawn and reads as itself (`1`, `1`, `/`, `2`, `2`); 0.4 % of the ink
is stray. Nib force while ink was laid (logged in an identical debug run of the same plan, `DEMO_STEP_S=0.4` set by
environment, same video): median 1.0 N, 5th to 95th percentile 0.66 to 1.43 N, peak 2.4 N, inside the handwriting range
above except the peak.

Scripted demo: the left hand puts its closed fingers against the back of the container, across from the label, about
10 cm up, pressing 3 mm in, and holds it there for the whole writing (the pen pushes the container toward it). The
right hand picks the marker up by the barrel 6 cm behind the tip, brings it to the tape and writes the date 30 mm tall,
centred on the tape, stroke by stroke: the tip is put on the tape at the start of each stroke, commanded 3 mm into the
surface (the arm is compliant, so it presses with about 1 N), moved along the stroke at 6.25 mm/s and lifted 15 mm off
between strokes. The marker is turned with the stroke so that it always points at the container's axis. The right hand
then lays the marker back on the paper where it was and lets go. It plays a fixed plan; it reads no simulation state.

## Still approximate (our assumptions)

- **Mass:** the whole container is one rigid 0.5 kg body (weighed to 500 g in the real trial); the water does not
  slosh. The water volume (about 470 ml) assumes the empty container and lid weigh about 30 g (our estimate); the
  scale reading is what counts. The **lid** (8 mm thick, 120 mm across) is our estimate.
- **Ink:** the threshold (0.5 N) is from ball-point handwriting (Schomaker and Plamondon 1990); a felt-tip marker
  inks at lower force. Forces above normal handwriting also ink (no upper limit). The force is PhysX's rigid contact
  force between a rigid nib and a rigid wall: no felt-tip or tape compliance, no line-width, angle, speed or ink-flow
  model. The tape is crepe paper and does not wrap a cone perfectly flat; here it is a flat band.
- **Reader:** the stroke-skeleton match is a proxy for a person reading the label. It accepts the listed forms (`1`
  plain, with a flag, a base or both; `4` closed or open-top; `7` plain or crossed; one form for the other
  characters); other readable handwriting (strongly slanted, very rounded or looped digits) may score lower.
- **Paper:** fixed to the table (taped down); ink on it is not simulated.
- **Rendering:** the container wall is drawn opaque (the water's colour behind clear plastic).
- Round parts are flat-faceted prisms.
