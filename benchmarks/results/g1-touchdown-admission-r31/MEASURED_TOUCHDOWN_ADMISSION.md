# Measured touchdown admission — r31

## Outcome

Revision r31 removes a physically invalid success path from the floating G1
controller. A motion schedule now requests touchdown; it does not establish
contact. Rust admits the planned sole material point only when all three
measured conditions hold:

- position error to the stored landing anchor is at most `0.025 m`;
- maximum whole-patch tangential speed is at most `0.20 m/s`;
- sole-centre normal speed is at most `0.20 m/s`.

If a scheduled handoff asks the current support to release while the incoming
support is not yet locked, the current support remains active. Once admitted,
the existing typed `TouchdownNormal → Locked` transition still requires four
ticks and at most `0.05 m/s` whole-patch tangential speed. No solver status is
used as a contact sensor.

This deliberately changes the interpretation of the 600-tick CMU stress. In
r30, the source asserted left touchdown at tick 428 while the measured foot was
about 34 cm from its landing anchor and moving about 3.6 m/s, then asserted
right-foot release four ticks later. The controller manufactured contact at the
measured mid-air pose and called 172 later ticks `TouchdownNormal`. In r31 the
same request is denied for the remaining 172 ticks and the right foot stays
locked. The trace remains behaviorally red, but it is no longer a false contact
success or an unsupported handoff.

## Architecture change

`bonesaw-core` owns the validated, serializable admission envelope through
`SupportTransitionConfig::accepts_touchdown`. `bonesaw-py` computes sole-centre
position/normal velocity and maximum finite-patch tangential speed from the
same Rust FK/Jacobian state used by the WBC. Delayed landings continue the
allocation-free cubic landing law against the stored anchor with a bounded
receding horizon. Contact arrays and nominal force sharing are assembled from
effective measured support, not authored contact flags.

The Python corpus remains eval land. It now reports both authored stance and
Rust-owned effective support, counts denied-contact ticks, records the longest
admission delay, and fails acceptance when that delay exceeds eight ticks. This
closes the previous vacuous pass where a trace with zero admitted touchdowns
could satisfy the touchdown-duration check.

## A/B: identical 600-tick CMU support-preview input

Both cases use the official Unitree G1 23-DOF URDF, CMU subject 37 trial 01,
`0.35×` forward/lateral scaling, a 200-tick precontact horizon, support-preview
CoM at Viability weight `0.1`, centroidal damping at Intent weight `1.0`, four
sole force points, strict `1e-8` hard-row acceptance, and the r30 cached-Jacobi
CPU kernel.

| Metric | r30 declaration-driven | r31 measured admission |
|---|---:|---:|
| left support phases | 29 swing / 200 precontact / 172 touchdown / 199 locked | 29 swing / 372 precontact / 0 touchdown / 199 locked |
| right support phases | 168 swing / 432 locked | 600 locked |
| delayed admission | not measured | 172 ticks, longest run 172 |
| fallback / release / infeasible / failed | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| root RMS | 23.670 cm | 20.026 cm |
| stance-foot RMS | 26.328 cm | 28.940 cm |
| swing-foot RMS | 28.588 cm | 27.249 cm |
| maximum root rotation | 9.339° | 13.921° |
| dynamics residual max | `1.622e-9` | `2.027e-9` |
| contact acceleration residual max | `7.067e-11` | `7.067e-11` |
| p50 / p95 / p99 | 2.552 / 3.830 / 4.808 ms | 2.423 / 3.615 / 4.763 ms |
| max tick | 6.175 ms | 6.073 ms |
| measured call wall / thread CPU | 1.600 / 1.599 s | 1.476 / 1.473 s |
| RSS delta / peak | 0.219 / 47.703 MiB | 0.168 / 48.035 MiB |
| Python GC collections | 0 | 0 |

These single-run timing and RSS figures describe the retained artifacts, not a
performance claim between revisions. Contact semantics change the active rows,
so behavior and work are expected to differ. The relevant result is that r31
remains below the predeclared 5 ms p99 gate while exposing the invalid landing.

## Accepted synthetic regression

The unchanged deterministic one-centimetre G1 toe-step proves that valid
landings are not blocked:

| Metric | r31 result |
|---|---:|
| functional / 5 ms / combined gate | PASS / PASS / PASS |
| support sequence | 40 precontact / 3 touchdown / 117 locked ticks |
| delayed admission | 0 ticks |
| root / stance / swing RMS | 3.567 / 0.002 / 0.639 cm |
| maximum root rotation | 0.139° |
| fallback / release / infeasible / failed | 0 / 0 / 0 / 0 |
| dynamics / contact residual max | `1.212e-9` / `4.585e-11` |
| p50 / p95 / p99 / max | 2.345 / 3.644 / 4.040 / 4.707 ms |
| call wall / thread CPU | 0.401 / 0.400 s |
| RSS delta / Python GC | 0.195 MB / 0 collections |

## Retained artifacts

- r30 control: `benchmarks/results/g1-cmu-transfer-r30-final`
- r31 CMU measured-admission trace:
  `benchmarks/results/g1-cmu-transfer-r31-touchdown-admission`
- r31 accepted toe-step:
  `benchmarks/results/g1-synthetic-step-r31-touchdown-admission`

The remaining failure is upstream of contact establishment: during the CMU
swing the tracked left foot departs laterally from its reference, reaches the
authored edge far from the landing anchor, and never enters the admission
envelope. The next controller/reference work must make the swing dynamically
reachable and trackable; widening the admission envelope would only recreate
the invalid-contact bug.
