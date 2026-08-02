# Bonesaw R40 G1 balance-phase causal report

This is an explicit rejected-policy report for the sustained CMU-to-G1 transfer. It asks whether the R39 Rust-owned source cursor can also use measured DCM support margin to recover balance. Python owns experiment selection and reporting; every cursor update, target sample, solve, state transition, and diagnostic tick remains in Rust.

## Result

**The balance-phase mechanism is not promoted.** A smooth capture-margin rate improves prefix and tracking relative to abrupt engagement, but no case admits touchdown or keeps the sustained transfer upright. The 800-tick extension diverges rather than converging.

Inactive-path parity is **PASS** across `71` shared non-timing arrays (NaNs compared in place). Defaults remain behaviorally unchanged.

## Consumer audit

The Rust batch path uses `reference_tick`, `reference_next_tick`, and one fraction for root, CoM, four endpoint jets, target activation, contact intent, precontact lookahead, prospective landing anchor, and future root reach. The DCM controller consumes the already time-warped CoM position/velocity jet. No remaining floating-WBC reference consumer was found indexing physical output tick after R39.

The earliest R39 symptoms precede touchdown retiming: root error crosses 5 cm at physical tick 197, the first liftoff is tick 199, measured DCM margin turns negative at tick 244, joint speed reaches 7.9 rad/s at tick 282, while touchdown geometry does not limit phase until tick 348. This motivated a separate measured-margin phase signal.

## Controller/reference matrix

| case | ticks | clean prefix | contingency/rejected | final source tick | min rate | root RMS | stance RMS | swing RMS | max rotation | DCM RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| R39 touchdown cursor | 600 | 482 | 118 | 420.872 | 0.070 | 68.26 cm | 30.20 cm | 81.21 cm | 179.8° | 47.62 cm |
| whole posture at Intent | 600 | 452 | 148 | 420.545 | 0.036 | 98.75 cm | 76.71 cm | 130.65 cm | 179.7° | 102.03 cm |
| pelvis horizontal at Preference | 600 | 390 | 210 | 417.608 | 0.060 | 103.75 cm | 62.26 cm | 126.30 cm | 180.0° | 95.00 cm |
| static support-preview + cursor | 600 | 542 | 58 | 423.928 | 0.031 | 39.69 cm | 9.87 cm | 45.36 cm | 179.6° | — |
| 0.10× spatial retarget | 600 | 465 | 135 | 423.758 | 0.039 | 59.97 cm | 23.11 cm | 45.29 cm | 179.9° | 28.23 cm |
| 0.10× + upper posture Intent | 600 | 471 | 129 | 425.098 | 0.028 | 53.84 cm | 31.21 cm | 43.65 cm | 179.9° | 27.55 cm |
| 0.10× + upper posture Viability | 600 | 481 | 119 | 424.879 | 0.027 | 45.84 cm | 36.25 cm | 36.94 cm | 162.5° | 27.52 cm |
| margin hold 0 / full +2 cm | 600 | 345 | 255 | 243.161 | 0.000 | 103.15 cm | 85.41 cm | 126.15 cm | 17.2° | 145.76 cm |
| outside taper, immediate | 600 | 268 | 154 | 296.926 | 0.000 | 67.88 cm | 54.07 cm | 81.03 cm | 137.6° | 61.18 cm |
| outside taper, 50-tick engage | 600 | 479 | 121 | 321.337 | 0.000 | 52.39 cm | 28.13 cm | 58.65 cm | 179.3° | 33.24 cm |
| outside taper, 0.5× floor | 600 | 490 | 110 | 447.087 | 0.500 | 57.09 cm | 31.37 cm | 46.33 cm | 179.8° | 55.25 cm |
| 0.5× floor, 800 ticks | 800 | 490 | 310 | 547.087 | 0.500 | 223.66 cm | 197.76 cm | 255.57 cm | 179.9° | 246.15 cm |

## Causal findings

- Moving whole-body posture from Preference to Intent makes root attitude fail earlier. Moving pelvis-horizontal intent down to Preference is worse again. The strict-priority conflict is real, but those reversals are not the solution.
- Reducing the spatial retarget from 0.35× to 0.10× cuts DCM RMS and swing error, but does not change the failure class. A Viability upper-body task creates 64 rejected solves and is rejected outright.
- Requiring +2 cm DCM margin for full rate is geometrically impossible after liftoff: the G1 sole half-width is 2.75 cm and the configured erosion is 1 cm, leaving at most 1.75 cm. The policy defaults therefore taper only outside support (full at 0, hold at -2 cm).
- Immediate rate engagement creates a 56.1 s⁻¹ phase-acceleration impulse through the exact chain rule and cuts the clean prefix to 268 ticks. A 50-tick engagement bounds that term to 4.0 s⁻¹ and restores a 479-tick prefix.
- Adding a 0.5× floor reaches authored touchdown tick 428 and extends the clean prefix to 490 ticks, versus 482 for R39. Swing RMS falls from 81.2 cm to 46.3 cm and non-nominal ticks from 118 to 110.
- That apparent progress is not physical touchdown progress: at the contact edge the minimum sole error is 0.639 m and minimum whole-patch tangential speed is 6.044 m/s. Both remain far outside the immutable 0.025 m / 0.20 m/s admission envelope.

## Failure over execution time

| trace | first 7.9 rad/s | first 5° root | first contingency | delayed touchdown ticks | minimum landing error | minimum tangential speed |
|---|---:|---:|---:|---:|---:|---:|
| R39 | 282 | 385 | 482 | 0 | — | — |
| smooth outside taper | 298 | 377 | 479 | 0 | — | — |
| 0.5× floor / 600 | 298 | 397 | 490 | 39 | 0.639 m | 6.044 m/s |
| 0.5× floor / 800 | 298 | 397 | 490 | 239 | 0.642 m | 3.366 m/s |

The 800-tick extension retains 239 delayed-touchdown observations. Its minimum error never improves below 0.642 m and the trajectory grows to metre-scale tracking error, proving non-convergence rather than insufficient observation time.

## CPU, jitter, memory, and solver work

| case | p50 | p99 | jitter p99 | thread CPU | RSS Δ | GC | pseudoinverses/tick mean |
|---|---:|---:|---:|---:|---:|---:|---:|
| R39 | 2505.4 µs | 186555.8 µs | 57860.0 µs | 6474.2 ms | 508.0 KiB | 0 | 7.79 |
| smooth outside taper | 2534.7 µs | 125919.6 µs | 98018.4 µs | 4835.7 ms | 456.0 KiB | 0 | 7.83 |
| 0.5× floor / 600 | 2542.4 µs | 208714.7 µs | 159226.8 µs | 10678.0 ms | 536.0 KiB | 0 | 7.87 |
| 0.5× floor / 800 | 2594.0 µs | 189063.5 µs | 145988.9 µs | 12293.3 ms | 916.0 KiB | 0 | 8.12 |

The optional margin policy is scalar and allocation-free, but these full tails are deadline-red because contingency solves dominate. Timing does not justify promotion while the behavior gate fails.

## Decision

Keep the pure support-margin phase-rate primitive and explicit root-horizontal priority as opt-in research controls; keep both disabled in production defaults. Do not relax contact thresholds, do not call a pre-edge hold a success, and do not promote the CMU/G1 controller. The next controller work must make the root/CoM/contact reference dynamically feasible—likely through a robot-native footstep/DCM planner or a reference implementation that emits consistent pelvis, CoM, and contact-wrench intent—before phase policy is revisited.

## Next evaluation boundary

Reference generators are evaluated open-loop before another controller policy is tried: identical initial robot state, footsteps, contact timing, sole geometry, and command; no tracked state, WBC status, solver output, or physics integration. The first contract checks contact continuity, reach, positive normal specific force, friction demand, and zero-angular-momentum CoP/DCM support margins. The current authored CMU reference already fails this layer, including 85.37 m/s² peak CoM acceleration and an 8.276 peak friction ratio. See `benchmarks/results/g1-reference-contract-r40/OPEN_LOOP_REFERENCE_CONTRACT.md`.

The source corpus still reports maximum retargeted endpoint speed `2.074 m/s`. Independent PlaCo, Pinocchio, and upstream Upkie comparisons remain in `benchmarks/results/reference-r38/REFERENCE_COMPARISON.md`; none exposes an equivalent measured capture-margin cadence law, so no unsupported reference-parity claim is made here.
