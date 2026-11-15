# Connect a hose to a tank outlet with a camlock coupling and lock both cam arms

A filled steel tank stands on the table, its outlet pointing at the robot: a 1 in pipe nipple ending in a cam and
groove (camlock) Type A adapter. A 1 in Type C coupler on a short tail of rubber hose lies on the table to the right,
its two cam arms open. The robot must push the coupler fully onto the adapter, so that the adapter's face is against
the coupler's gasket, and press both cam arms flat against the coupler body, which locks the coupling.

**Instruction:** `Connect the hose to the tank outlet and lock the coupling with both cam arms.`

## Why this task

Cam and groove couplings are the standard quick connection for liquid transfer hoses: tank trucks and tank cars,
totes and drums, pumps, frac and storage tanks, septic and dewatering hoses, farm sprayers. Connecting one is routine
work for tank truck drivers, tank car, truck and ship loaders (O*NET 53-7121), pump system operators (53-7072),
chemical and petroleum plant and system operators, septic tank servicers (47-4071) and construction laborers running
dewatering pumps. The connection is only safe if the coupler is fully on and both arms are down: a coupler that is
not seated leaks, and an arm left up lets the coupler cock and blow off under pressure.

The coupler is a loose part with 0.8 mm of radial play on a 36.5 mm plug, held by its hose 10 cm behind the mouth; it
has to be lined up and pushed 30 mm onto the adapter. Its two arms are hinged parts that must each be swung through
90 degrees and pressed flat, on both sides of the coupling. Closing the arms first, or with the coupler only
part way on, does not lock it.

## Hardware

Both halves are standard parts made to the US interchange standard (Commercial Item Description A-A-59326):

- **Coupler:** Dixon **100-C-AL**, 1 in Type C coupler x hose shank, 356-T6 aluminium, nitrile gasket (Dixon 100-G).
- **Adapter:** Dixon **100-A-AL2**, 1 in Type A adapter x female NPT, 6061-T6 aluminium, screwed onto a 1 in pipe
  nipple from the tank.

Dimensions from the Dixon Cam & Groove catalogue (dimension tables for Type A adapters, Type C couplers, gaskets and
nominal take-up lengths), as built in the simulation:

| Data-sheet dimension (1 in) | Value | Simulation |
|---|---|---|
| Coupler overall length A | 4-1/2 in (114.3 mm) | 114.3 mm |
| Coupler length B | 1-9/16 in (39.7 mm) | 39.7 mm |
| Distance across closed cam arms C | 2-7/16 in (61.9 mm) | 61 mm |
| Distance across open cam arms D | 5-3/16 in (131.8 mm) | 133 mm |
| Hose shank OD E | 1-1/16 in (27.0 mm) | 27.0 mm (12-sided) |
| Adapter overall length A | 1-31/32 in (50.0 mm) | 50.0 mm |
| Adapter length B | 1-3/8 in (34.9 mm) | 34.9 mm |
| Adapter hex across flats C | 1-1/2 in (38.1 mm) | 38.1 mm |
| Nominal take-up length | 1.20 in (30.5 mm) | the plug goes 30.5 mm into the coupler |
| Gasket 100-G | 1-9/16 OD x 1-1/16 ID x 1/4 in | same, at the bottom of the bore |
| Plug OD / coupler bore (size chart) | 1-7/16 in (36.5 mm) / 1-1/2 in (38.1 mm) | same |
| Coupler weight | 0.4 lb (0.18 kg) | coupler with hose tail 0.25 kg, arms 25 g each |

**Why 1 in and not 2 in:** Dixon's instruction is to "press the cam levers down with normal hand pressure". The
YAM arms push the arms with the side of the closed fingers; the arms are compliant (position-controlled joints), so
the push force they can give is limited. The 1 in coupler has short arms without pull rings and the smallest closing
torque; its hose (34 mm) also fits the 94 mm gripper with room, and the 0.25 kg assembly can be carried at arm's
length. A 2 in coupler (0.5 kg and more, 4 in across, closing on a 2 in gasket) would be closer to tanker work but
asks more force than we can show the arms give.

## Design

| Object | Size | Mass | Notes |
|---|---|---|---|
| Tank | 30 x 36 x 26 cm, x 38 to 68, y -18 to 18 cm | 40 kg (filled) | Painted steel; does not move under the robot's push |
| Outlet | 1 in schedule 40 steel pipe nipple (33.4 mm OD) + Dixon 100-A-AL2 adapter, axis along x at y 0, 12 cm up, adapter face at x 26 cm | part of the tank | |
| Coupler | Dixon 100-C-AL (table above), mouth toward the tank, mouth at x 22, y -12 cm, axis 3.5 cm up | 0.25 kg with hose | Aluminium; black nitrile gasket |
| Cam arms | 36 mm from pivot to tip, 7 x 10 mm section; pivots 27 mm out from the axis, 10 mm behind the mouth | 25 g each | Start open, straight out to the sides |
| Hose tail | 1 in rubber hose, 34 mm OD, 20 cm, band clamp over the shank | in the coupler's mass | Rigid in the simulation |
| Pine strips | under the body (12 mm tall) and under the hose (18 mm tall) | fixed | Keep the coupler level so the fingers can reach under the hose |

Positions are from the midpoint between the two arm bases (x forward, y left, z up).

**The cam arms.** Each arm turns on a vertical pivot. A free arm turns with 0.03 N m of pivot friction. The cams'
real behaviour is emulated, not modelled by contact:

- the arms close only when the coupler is far enough on: with the plug in the coupler but more than 2 mm short of the
  gasket, an arm that is opened stops at 40 degrees (its cam lobe on the plug's outside);
- within the last 2 mm, closing the arm works the cams: 0.20 N m over its last 30 degrees (the cams drawing the coupler
  onto the gasket);
- an arm flat against the body (within 8 degrees) on a seated coupler is **locked**: the coupler is then joined to the
  adapter (drawn the last 2 mm onto the gasket and held) until both arms are opened again past 40 degrees;
- an arm closed while the coupler is off the adapter does not count, even if the coupler is pushed on afterwards (on
  the real coupler its cams would be in the bore and stop the plug); it has to be opened and closed again.

## Scoring

Evaluated every control step; the score at the end of the episode counts (`SCORE_OVER = "end"`).

| Condition | Score |
|---|---|
| The coupler at any point fell off the table, or it is not on the adapter (coaxial within 2 mm and 5 degrees) at least half way | 0 |
| On the adapter, not seated | 0.25 |
| Seated: the adapter's face within 1 mm of the gasket | 0.5 |
| Seated, plus 0.25 for each cam arm that was closed flat (within 8 degrees) on the seated coupler and is still flat | 0.75 / 1.0 |

1.0 (both arms locked on the seated coupler) is success, held 1 s. Doing nothing scores 0; closing the arms before
connecting, or pushing the coupler on without closing them, scores at most 0.5.

## Verified in simulation

| Run | Host / GPU | Steps | Score |
|---|---|---|---|
| Scripted demo (task.py sha256 201cec688dd85004) | shiva, RTX A6000 (GPU 0) | 781 of 1,200 (success at 39.1 s) | 1.0 |
| Dream-1 (same task.py) | shiva, RTX A6000 (GPU 2) | 1,200 of 1,200 (step limit) | 0.0: both arms stay near their bases for most of the run; late on the right arm reaches the coupler and nudges and turns it on the table but never puts it on the adapter |

Scripted demo: the right arm picks the hose up behind the coupler, lines the coupler up with the adapter and pushes
it on until the adapter's face is against the gasket, then lets go and moves away. The left arm then sweeps the left
cam arm from open to flat with the side of its closed fingers and presses it; then the right arm does the same on the
right (one after the other: two hands side by side over the 6 cm coupling collide). It plays a fixed plan with two
measured offsets (the right arm sags 4.5 to 6.5 mm with the coupler at arm's length); it reads no simulation state.

## Still approximate (our assumptions)

- **Tank:** our own size and mass (a filled 28 L steel tank); only its outlet matters.
- **Coupler body OD (46 mm) and arm section** are our estimates; the catalogue gives lengths, across-arm distances and
  the shank, not the body OD. The pivots are placed so that the closed and open across-arm distances match the
  catalogue.
- **Groove:** drawn 10 to 17 mm from the plug's face, 1.8 mm deep (our estimate; it is visual only, the cams are
  emulated).
- **Take-up length as insertion depth:** we read Dixon's nominal take-up length (1.20 in) as how far the plug goes
  into the coupler.
- **Closing torque** (0.20 N m over the last 30 degrees, about 6 N at the arm's tip), **pivot friction** and the
  **2 mm cam draw** are our assumptions; Dixon gives only "normal hand pressure".
- **Cocking:** on a real coupler, closing one arm alone tilts the coupler on the plug; here one closed arm already holds
  the coupler on. The score needs both arms anyway.
- **Gasket rigid; hose tail rigid** (20 cm stub).
- Round parts are 16-sided prisms (the hex is 6-sided).
