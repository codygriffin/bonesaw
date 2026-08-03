# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-multistep-reference-r53/reference-inputs.npz` from `Python-authored alternating sequence of Rust Bonesaw LIPM boundary plans`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.186 m` forward per `11.585 s` source cycle.
- Applied mean forward speed: `0.016 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `11.6 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: disabled; no loaded finite patch was declared.
- Balance task: `standalone-authored` `DCM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `standalone-authored`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.050`; morphology jet `enabled`.
- Protected coordinate posture: `intent` priority with weight `0.000` over 11 `upper-body` coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.250`, activating at `50.0%` of each effective limit with `2.000 Hz` response and `multi-support` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Joint-velocity envelope scope: `lower-body allowlist`; hard acceleration-bound intersection `enabled`.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `0` ticks (`0.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `disabled`; authored offset ≤ `0.120 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `40` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `disabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `disabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
- Touchdown admission limits: `0.025 m` position, `0.200 m/s` tangential, and `0.200 m/s` normal; the outgoing support is retained until its replacement locks.
- Reference touchdown blend: `0` ticks (`0.000 s`) with a quintic zero-velocity endpoint.
- Touchdown stabilization point: `ankle origin`.

## Acceptance

Functional: **FAIL**
5 ms p99 deadline: **FAIL**
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | FAIL |
| `touchdown_transition_completes_within_8_ticks` | FAIL |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 926 | 4.630 s | 10.649 cm | 3.156 cm | 15.555 cm | 37.005 cm | 54.881° | 8.000 rad/s | 5902.9 µs |

Nominal hard residual maxima: dynamics `2.073e-09`, contact acceleration `1.066e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1928.033 cm |
| authored reference vs measured CoM RMS / p95 | 1927.659 / 4070.897 cm |
| stance foot RMS | 1833.665 cm |
| swing foot RMS | 2174.682 cm |
| hand RMS | 1950.916 cm |
| maximum root rotation | 107.787° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.848e-09 |
| contact acceleration residual | 1.066e-10 |
| raw max dynamics residual, including rejected ticks | 3.848e-09 |
| raw max contact residual, including rejected ticks | 1.066e-10 |
| active normal force range | 0.000–631.410 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 138.937 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 162 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `38.553` / `103.235 cm`.
- Virtual ZMP clipped on `75.76%` of ticks; clip-distance RMS / max `70.966` / `230.235 cm`.
- Measured-height natural frequency min / p50 / max: `3.734` / `3.797` / `4.055 rad/s`.
- Minimum measured CoM height: `0.596 m`; height-floor ticks: `0`.
- CoM command acceleration p95 / max: `7.738` / `10.920 m/s²`; support hull `4–6` vertices.
- Signed measured DCM support margin min / p05: `-121.270` / `-99.320 cm`; inside on `31.82%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `2316.000`, progress `2316.000` ticks.
- First post-liftoff authored touchdown source tick: `529`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `0`.

## Capture-aware landing

- Active target-ticks: `0`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.0000 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1223`, single support `444`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `27` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `141` / `952` / `5` / `0` / `0` / `0` / `1219`.
- Bound-limited ticks / upper-bound ticks: `3` / `1`; maximum coordinate violation `9.370482016725724`.
- Named-linear-row-limited ticks / upper-row ticks: `5` / `2`; maximum row violation `107.87337785724134`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 9.1 µs | 4484.1 µs | 6725.6 µs | 181498.0 µs | 141 | 623 | 0 | 297 | 1,252 | 0 | 0 | 0 | 0 | 4 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1694.6 | 5520.1 | 3.4 | 3558.6 | 10960.6 | 180639.3 | 2512.6 | 1,097 | 67 | 2 | 590.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 141 | 2897.3 | 3029.0 | 3262.1 | 3474.6 |
| solved_with_slack | 623 | 3102.3 | 5033.5 | 6736.5 | 7436.8 |
| normal_contact_contingency | 1,252 | 6.6 | 10.0 | 3343.7 | 6096.8 |
| contact_release_contingency | 4 | 89933.0 | 180941.9 | 181386.8 | 181498.0 |
| touchdown_transition | 297 | 3114.2 | 6479.8 | 8777.1 | 11185.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.81 | 12.0 | 16.0 | 24 | 2.97 | 14.0 | 22 | 0.2519 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.35/8.0/8.0/8 | 302.32/1776.0/1872.0/1872 | 0.34/5.7/14 | 0.22/4.7/13 | 1.79/35.1/108 | 0.1906 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 141 | 4.03/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 623 | 8.44/16.0/23 | 6.81/16.0/21 |
| normal_contact_contingency | 1,252 | 0.24/9.0/18 | 0.23/9.0/18 |
| contact_release_contingency | 4 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 297 | 9.09/24.0/24 | 7.88/22.0/22 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.75/5.0/8 | 3.20/24.0/34 | 0.41/5.0/7 | 365 |
| viability | 0.45/5.0/11 | 2.06/25.0/77 | 0.32/5.0/11 | 357 |
| intent | 1.23/8.0/13 | 2.47/17.7/33 | 1.08/8.0/13 | 821 |
| preference | 0.86/8.8/19 | 7.61/75.7/157 | 0.78/8.8/19 | 938 |
| style | 0.53/2.0/3 | 7.68/33.0/50 | 0.37/2.0/3 | 757 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,537 | 0 | 212 | 535 | 33 |
| right_ankle_roll_link | 1,353 | 0 | 85 | 879 | 0 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `162` ticks, normal fallback `36` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.2570 m/s`, p95 `5.4990 m/s`, max `6.3725 m/s` over 293 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.927 | 3.926 | 3.926 | 1.000 | 1.000 | 52.934 | 56.242 | 3.309 | 56.242 | 0.002 | 0 | 801 | 0 | 0 | 37 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2953.3 | 6309.7 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2934.0 | 4959.2 | 8.59 | 50.93 | 6.10 | 2.18 | 486.75 | 0.913 | 0.628 | 9.69e-10 | 4.84e-11 | 0 |
| 464–695 | 3133.0 | 5666.2 | 9.15 | 51.97 | 7.43 | 3.17 | 712.89 | 15.429 | 5.427 | 1.64e-09 | 4.70e-11 | 0 |
| 696–927 | 3169.8 | 6998.7 | 8.50 | 48.90 | 7.69 | 2.57 | 590.41 | 14.908 | 13.094 | 2.07e-09 | 1.07e-10 | 2 |
| 928–1159 | 7.4 | 5705.8 | 2.75 | 18.66 | 2.66 | 1.88 | 405.00 | 285.267 | 268.037 | 2.49e-09 | 9.78e-11 | 192 |
| 1160–1391 | 6.6 | 12.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 950.046 | 903.078 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.5 | 12.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1708.006 | 1656.723 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.8 | 9086.3 | 1.97 | 11.24 | 1.94 | 1.37 | 295.48 | 2464.998 | 2431.185 | 2.01e-09 | 2.99e-11 | 181 |
| 1855–2085 | 6.7 | 11.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 3265.442 | 3236.393 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.9 | 8107.2 | 1.70 | 10.53 | 1.64 | 1.38 | 297.35 | 4077.725 | 4049.984 | 3.85e-09 | 8.35e-11 | 186 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1928.033 | 1905.918 | 1950.916 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
