# G1 capture-aware landing experiment · r37

## Question

Revision r36 measured the failure precisely: during the first CMU 37/01
transfer, DCM left the eroded support polygon by more than 85 cm while the
incoming landing remained fixed. Revision r37 asks whether a bounded,
allocation-free landing planner can improve that transfer without weakening
touchdown admission or hard feasibility.

All transfer cases use the pinned Unitree G1 URDF, 600 × 5 ms Rust-owned ticks,
backward DCM preview, a 1 cm support erosion, a 200-tick precontact horizon,
centroidal angular-momentum damping, and unchanged 2.5 cm / 0.20 m/s measured
touchdown limits.

## CPU policy

`bonesaw-core` now exposes a pure capture-landing primitive. It moves a latched
sole-center anchor toward measured DCM, but only along the authored-to-capture
ray. An analytic ray/sphere intersection simultaneously enforces:

- a configurable horizontal disk around the authored endpoint;
- the future root's spherical leg-reach envelope;
- fixed authored landing height; and
- a maximum anchor speed per control tick.

The PyO3 session owns the previous applied anchor as explicit state. It uses
the prior tick's DCM observation, updates only in measured Precontact, and can
freeze the anchor before the authored contact edge. Once frozen, the existing
measured touchdown gate naturally retimes contact: the outgoing support stays
established until the new sole is both close and slow. No task rows, vectors,
or Python callbacks are created per tick.

Fixed-shape telemetry records applied anchor, capture scale, authored offset,
future-root reach, projection/slew/freeze flags, sole-center position error,
whole-patch tangential speed, and normal speed.

## Spatial sweep

The first matrix used feedback-envelope weight 0.10 and allowed the anchor to
continue following DCM after the authored edge. `first contingency` is the
length of the clean nominal prefix.

| offset / speed | first contingency | root RMS | stance RMS | swing RMS | DCM RMS | minimum DCM margin | rejected |
|---|---:|---:|---:|---:|---:|---:|---:|
| r36 fixed landing | 472 | 56.72 cm | 38.09 cm | 53.61 cm | 33.52 cm | -85.86 cm | 0 |
| 4 cm / 0.25 m/s | 467 | 55.62 cm | 32.94 cm | 59.54 cm | 34.50 cm | -105.42 cm | 0 |
| 8 cm / 0.25 m/s | 470 | 61.61 cm | 39.87 cm | 37.25 cm | 20.24 cm | **-64.88 cm** | 0 |
| **8 cm / 0.50 m/s** | **482** | **40.34 cm** | 31.04 cm | 45.60 cm | **18.29 cm** | -69.71 cm | 0 |
| 12 cm / 0.25 m/s | 427 | 85.71 cm | 52.81 cm | 81.02 cm | 77.34 cm | -222.89 cm | 0 |
| 12 cm / 0.50 m/s | 461 | 61.29 cm | 33.44 cm | 52.18 cm | 22.58 cm | -129.08 cm | 0 |
| 16 cm / 0.50 m/s | 460 | 63.75 cm | 36.67 cm | 52.96 cm | 31.33 cm | -94.51 cm | 0 |

Eight centimetres is the useful spatial budget in this trace. Larger motion is
not monotonically better because the landing task competes with saturated
joint velocity, root attitude, and single-support balance.

## Landing commitment sweep

Continuous post-edge chasing cannot converge to contact, so the final policy
freezes at or before the authored edge.

| freeze lead | first contingency | root RMS | stance RMS | swing RMS | DCM RMS | rejected |
|---|---:|---:|---:|---:|---:|---:|
| edge (0 ticks) | **482** | 46.73 cm | **22.69 cm** | 43.36 cm | **18.13 cm** | 0 |
| 20 ticks | **482** | **36.38 cm** | 29.71 cm | **36.66 cm** | 27.39 cm | 0 |
| 40 ticks | **482** | 65.04 cm | 33.07 cm | 67.27 cm | 24.86 cm | 6 |
| 80 ticks | **482** | 58.26 cm | 29.95 cm | 51.03 cm | 26.72 cm | 0 |

Freezing at the edge is the stable diagnostic configuration: 200 update ticks,
172 frozen delayed-admission ticks, an exact 8 cm maximum offset, 0.852 m
maximum future-root reach, and no unreachable authored geometry.

## What the physical gate says

The incoming sole still never becomes an admissible contact. Across the 172
post-edge observations in the edge-freeze case:

- minimum sole-center position error is 19.86 cm, versus a 2.5 cm limit;
- minimum whole-patch tangential speed is 0.678 m/s, versus 0.20 m/s;
- normal speed is individually viable on 32 ticks; and
- zero ticks satisfy all three conditions.

The gate is therefore rejecting a physically distant and fast sole, not
blocking a plausible touchdown. Loosening it would manufacture a contact.

## Authority sensitivity

Retargeting does not remove the controller's nonlinear sensitivity to the
joint-velocity envelope. With the 8 cm edge-freeze policy, weight 0.15 reaches
tick 491. Weight 0.20 reaches tick 533 and reduces minimum position error to
10.18 cm, but tangential speed rises to 1.674 m/s. The neighboring 0.22 case
falls back to tick 433 and produces 85 rejected solves. We retain 0.10 as the
stable experimental setting and leave the entire planner disabled by default.

## Architectural conclusion

R37 closes the missing spatial-policy seam: landing intent is now measured,
bounded, reachable when the authored geometry permits it, rate-limited,
committed, and fully observable in Rust. It improves the clean prefix by ten
ticks over r36 and roughly halves DCM RMS in the stable case. It does not make
the CMU transfer walk.

The remaining failure is whole-trajectory feasibility before touchdown. The
next planner slice should retime the complete root/CoM/swing jet from measured
liftoff through landing, with a viability preview that rejects references whose
sole position and tangential-speed envelopes cannot intersect the touchdown
gate. The contact gate and hard-row `1e-8` contract remain frozen.

## Regression gates

| profile | ticks | root RMS | stance RMS | swing RMS | max attitude | p99 |
|---|---:|---:|---:|---:|---:|---:|
| deterministic 1 cm toe step | 160 | 3.567 cm | 0.002 cm | 0.639 cm | 0.139° | 4.948 ms |
| moving liftoff | 260 | 0.748 cm | 0.000 cm | 0.173 cm | 2.256° | 3.937 ms |

Both remain combined green. The planner is opt-in, so stable default traces are
unchanged apart from the new fixed-shape telemetry.

Workspace verification passes 90 `bonesaw-core` and two `bonesaw-tools` unit
tests, strict stable Clippy with warnings denied, checked Rust formatting, and
Python bytecode compilation. A 1,000-tick native release sentinel remains
bitwise repeatable and reports zero allocation calls/bytes per tick for all
three controller scenarios, the compiled rig, fixed/floating dynamic WBC, and
collision-enabled control.
