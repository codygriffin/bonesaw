# Upkie differential-wheel planar capture A/B · upkie-planar-capture-ab-r134

**Evaluation admission: PASS. Controller promotion: REJECTED.** Evaluation success means the candidate is measured repeatably and rejected honestly when it does not recover the declared lateral cases.

The candidate keeps persistent planar DCM and steering-direction state in Rust, slews bounded differential-wheel yaw commands, supplies heading-rotated contact bases to the same floating WBC, and exposes signed lateral support margin. Python owns only MuJoCo cases, timing, scoring, and artifacts.

## Frozen A/B

| case | baseline | baseline boundary s | candidate | candidate boundary s | Δ boundary s | candidate peak Δp mm | candidate peak tilt ° | minimum lateral margin mm | maximum heading ° | maximum yaw cmd rad/s | later nonadmitted | Rust p99 µs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | 6.000 | RECOVERED | 6.000 | +0.000 | 0.1 | 0.05 | 152.4 | 0.00 | 0.000 | 0 | 159.7 |
| forward_4n_reference | RECOVERED | 6.000 | RECOVERED | 6.000 | +0.000 | 107.6 | 24.91 | 152.4 | 0.00 | 0.000 | 0 | 149.9 |
| left_1n | FALL | 2.260 | FALL | 2.660 | +0.400 | 307.5 | 47.80 | 114.8 | 84.13 | 0.500 | 78 | 10019.9 |
| left_2n | FALL | 1.985 | FALL | 2.120 | +0.135 | 450.0 | 46.09 | 95.7 | 52.16 | 0.451 | 60 | 9039.4 |
| right_2n | FALL | 1.990 | FALL | 2.505 | +0.515 | 273.2 | 45.58 | 78.7 | 27.41 | 0.500 | 48 | 9261.9 |

## Gain sensitivity

| heading gain | left_1n | left_2n | right_2n |
|---|---|---|---|
| 0.40 | FALL @ 2.175s | FALL @ 2.160s | RECOVERED @ 6.000s |
| 0.50 | FALL @ 2.660s | FALL @ 2.120s | FALL @ 2.505s |
| 0.60 | FALL @ 2.220s | FALL @ 1.770s | FALL @ 1.695s |

## Gates

| gate | observed | pass |
|---|---|---|
| frozen baseline and candidate matrix complete | 2 × 5 cases | True |
| baseline reproduces r133 boundary | {'nominal': 'RECOVERED', 'forward_4n_reference': 'RECOVERED', 'left_1n': 'FALL', 'left_2n': 'FALL', 'right_2n': 'FALL'} | True |
| candidate preserves nominal and sagittal recovery | {'nominal': True, 'forward_4n_reference': True} | True |
| candidate delays every lateral first boundary | {'left_1n': 0.40000000000000036, 'left_2n': 0.13500000000000023, 'right_2n': 0.5149999999999999} | True |
| candidate replay is exact | True | True |
| neighboring gain sensitivity is discriminating | True | True |
| Rust timed regions remain allocation-free | True | True |
| no pre-boundary numeric fault | True | True |
| failed recovery prevents promotion | {'promote': False, 'blockers': ["no lateral recovery: ['left_1n', 'left_2n', 'right_2n']"]} | True |

## Decision

- Promotion blockers: **["no lateral recovery: ['left_1n', 'left_2n', 'right_2n']"]**.
- Candidate exact semantic replay: **True**.
- The three candidate falls retain positive DCM-to-track margins of +114.8/+95.7/+78.7 mm. That scalar is useful evidence, but it is not a nonlinear recovery certificate: pitch/yaw/contact coupling reaches the fall boundary first.
- At heading gain 0.40, the full 6.0 s sensitivity horizon recovers only right_2n (RECOVERED); the mirrored left rows still fall. The gain response is sign-specific and non-monotone.
- The declared candidate delays all three first-fall boundaries while preserving nominal and sagittal recovery, but it recovers none of the frozen lateral cases. It therefore remains an experimental diagnostics path, not the live controller.
- The next physical milestone must change available authority—an explicit support/contact transition, steering-aware nonholonomic program with a robust viability proof, or fall-safe behavior—not merely tune a per-tick QP.
