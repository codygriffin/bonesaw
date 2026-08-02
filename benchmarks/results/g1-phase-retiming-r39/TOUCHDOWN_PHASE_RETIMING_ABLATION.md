# Bonesaw R39 measured touchdown phase-retiming ablation

This report evaluates one architectural change: Rust owns a persistent source-phase cursor and retimes root, CoM, every endpoint jet, and contact intent together from measured sole position and velocity. Python constructs the fixed corpus and renders this report; it does not reproduce the policy.

## Outcome

- Viable-reference no-op contract: **PASS**; enabled and disabled viable traces behaviorally exact: **PASS**.
- Stressed touchdown rescue: nominal transition `10` ticks → retimed `3` ticks; combined acceptance **PASS**.
- Retimed stressed trace reproducibility: **PASS**, three bit-for-bit behavioral repeats; median p99 `4698.5 µs`, all `3/3` below 5 ms.
- CMU H=200 failure boundary: 600-tick cursor ends at source tick `420.872` before authored touchdown tick `428`; the new progress gate correctly remains **FAIL**.

## Policy contract

1. A pure Rust policy estimates conservative landing time from position excess, whole-patch tangential speed, normal speed, acceleration capacity, and the unchanged 2.5 cm / 0.20 m/s / 0.20 m/s admission envelope.
2. The fastest admissible scalar phase rate is slew-bounded. Engagement can be immediate; recovery is bounded to avoid cadence snapping.
3. One Rust cursor samples quintic Hermite jets for root, CoM, and all effectors. Velocity and acceleration use the full time-warp chain rule.
4. Contact intent is sampled from the same cursor. The evaluator additionally requires the first post-liftoff authored touchdown edge to be reached, so holding before contact cannot score as a vacuous admission success.

## Contact and acceptance matrix

| case | phase | combined | first touchdown reached | transition ticks | admission delay | contingency/rejected | minimum rate | failed checks |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| viable synthetic | off | PASS | True | 3 | 0 | 0 | 1.0000 | none |
| viable synthetic | on | PASS | True | 3 | 0 | 0 | 1.0000 | none |
| 6 cm / 100 ms stress | off | FAIL | True | 10 | 0 | 0 | 1.0000 | touchdown_transition_completes_within_8_ticks |
| 6 cm / 100 ms stress | on | PASS | True | 3 | 0 | 0 | 0.4820 | none |
| CMU H=200, 600 ticks | off | FAIL | n/a | 0 | 172 | 120 | 1.0000 | no_contact_contingency_ticks, touchdown_admission_completes_within_8_ticks, root_tracking_rms_le_5cm, stance_foot_tracking_rms_le_2cm, swing_foot_tracking_rms_le_8cm, root_rotation_le_5deg, p99_tick_le_5ms |
| CMU H=200, 600 ticks | on | FAIL | False | 0 | 0 | 118 | 0.0705 | no_contact_contingency_ticks, root_tracking_rms_le_5cm, stance_foot_tracking_rms_le_2cm, swing_foot_tracking_rms_le_8cm, root_rotation_le_5deg, retiming_reaches_first_authored_touchdown, p99_tick_le_5ms |
| CMU H=200, 800 ticks | on | FAIL | False | 0 | 0 | 318 | 0.0090 | no_contact_contingency_ticks, root_tracking_rms_le_5cm, stance_foot_tracking_rms_le_2cm, swing_foot_tracking_rms_le_8cm, root_rotation_le_5deg, retiming_reaches_first_authored_touchdown, p99_tick_le_5ms |

The stressed off/on pair changes only phase retiming. The nominal trace crosses contact before the whole sole can settle and remains in touchdown transition for 10 ticks. Retiming bottoms at `0.4820×`, limits `24` physical ticks, crosses source touchdown tick 100, and locks in 3 ticks without a fallback, release, infeasible, or failed solve.

## Tracking

| case | root RMS | stance foot RMS | swing foot RMS | max root rotation | max joint speed |
|---|---:|---:|---:|---:|---:|
| stress off | 0.000 cm | 0.095 cm | 1.933 cm | 0.000° | 4.220 rad/s |
| stress on | 0.383 cm | 0.013 cm | 1.907 cm | 0.299° | 8.000 rad/s |
| CMU off | 59.631 cm | 32.232 cm | 52.251 cm | 179.887° | 8.000 rad/s |
| CMU on, 600 | 68.264 cm | 30.203 cm | 81.212 cm | 179.804° | 8.000 rad/s |
| CMU on, 800 | 150.373 cm | 123.898 cm | 159.555 cm | 179.966° | 8.000 rad/s |

## CPU, latency, jitter, memory, and GC

Single-process release traces include Python call overhead around one Rust batch. Scheduler context switches are retained rather than filtered.

| case | p50 | p99 | jitter p99 | thread CPU | RSS Δ | traced peak | GC collections | involuntary switches |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| viable off | 2318.3 µs | 3955.6 µs | 1523.2 µs | 389.9 ms | 156.0 KiB | 1424 B | 0 | 3 |
| stress off | 2280.2 µs | 3815.4 µs | 1013.8 µs | 562.4 ms | 224.0 KiB | 1424 B | 0 | 9 |
| stress on #1 | 2381.4 µs | 4698.5 µs | 1564.2 µs | 630.1 ms | 228.0 KiB | 1424 B | 0 | 52 |
| stress on #2 | 2384.0 µs | 4688.0 µs | 1543.4 µs | 630.2 ms | 220.0 KiB | 1424 B | 0 | 5 |
| stress on #3 | 2365.2 µs | 4944.4 µs | 1748.2 µs | 633.7 ms | 220.0 KiB | 1424 B | 0 | 4 |
| CMU on, 600 | 2505.4 µs | 186555.8 µs | 57860.0 µs | 6474.2 ms | 508.0 KiB | 1424 B | 0 | 85 |

The three stress-on thread-CPU measurements span `630.1–633.7 ms` for 240 ticks. All runs report zero Python GC collections in the measured call. RSS delta is process-level demand paging, not controller heap-allocation telemetry.

## Determinism and no-op proof

- Stress-on behavior digest: `a221fef6b69b771a9535550868c46cff7fd1f013a45f107a1f5cf93962ba0c7a`; all three equal: `True`.
- The digest covers state, force, physical residuals, status, support phase, phase cursor/rates, effective root/CoM/endpoint targets, and effective contact intent. Timing arrays are intentionally excluded.
- Disabled retiming reproduces every authored target/contact array and the integer source cursor exactly: `True`.
- On the already-viable synthetic trace, enabling retiming changes no behavioral byte: `True`.

## Honest failure boundary

R39 is not a claim that the current CMU-to-G1 transfer is solved. At 600 physical ticks the controller advances only to source tick `420.872`; at 800 it advances to `425.992`. Both remain before touchdown tick 428 as the measured robot diverges. The first-touchdown progress gate fails both, preventing the absence of delayed admission samples from being misreported as success. This separates a valid retiming mechanism from an unsolved sustained-balance/reference problem.

The independent PlaCo, Pinocchio, and upstream Upkie comparisons remain in `benchmarks/results/reference-r38/REFERENCE_COMPARISON.md`. Those references do not expose an equivalent coupled measured-phase policy, so this ablation compares identical Bonesaw controller configurations off/on and makes no unsupported cross-implementation phase-retiming claim.
