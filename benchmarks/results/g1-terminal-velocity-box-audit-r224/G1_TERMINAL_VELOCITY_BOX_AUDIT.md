# Bonesaw G1 terminal velocity-box audit · r224

> Velocity-box mechanism **PASS** · frozen prediction profile **ALREADY REJECTED** · authority **NOT ADMITTED** · physics/policy/controller/integration **0 / 0 / 0 / 0**.

## Contract

- Rust propagates the entire frozen R220 generalized-velocity box to the declared 0.45 m root-impact plane. Ballistic time is bounded from vertical-velocity endpoints; tilt, angular rate, joint position, and joint velocity use allocation-free interval arithmetic across that time interval. Pressure and aggregate fields are upper bounds; joint headroom is a lower bound.
- The box is componentwise and distribution-free. It assigns no probability, does not assert every Cartesian corner is reachable, omits yaw/horizontal velocity because the declared terminal proxy does not consume them, and remains only a collision-consequence proxy—not injury, recovery, or hardware safety.
- The paired predicted/completed contact states are exactly the R222 zero-plant ablation. Smooth-force evolution is excluded identically. Every contained completed state must be bounded componentwise; all queries repeat bitwise and allocate nothing in the timed Rust path.

## Result

| law | source full coverage | terminal projection coverage | center → box false-safe | contained false-safe | conservative rejects | contained harm slack p50/p95 | box p99 µs |
|---|---|---|---|---|---|---|---|
| mid_elliptic_implicitfast | 91.667% | 91.667% | 7 → 0 | 0 | 10 / 48 | 21.379 / 25.370 | 1.344 |
| hard_pyramidal_rk4 | 64.583% | 39.583% | 1 → 0 | 0 | 2 / 48 | 21.841 / 35.029 | 1.194 |

## Decision

Retain the generic velocity-box terminal bound: it removes false-safe decisions for every completed state actually contained by the declared terminal projection, with deterministic allocation-free execution. Do not admit the frozen R220/R221 profile: it missed fresh-law rows before terminal scoring, and the conservative box can reject safe completed states. The next gate is therefore a tighter transferable transition set—not more permissive terminal thresholding.
