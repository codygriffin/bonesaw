# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.361 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.276 m/s` (`0.35×` forward, `0.35×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.100`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.500`, guard `0.020 s`, engagement / release `50 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
- Coupled balance phase retiming: `enabled`; hold at signed DCM margin ≤ `-0.020 m`, recover nominal rate at `0.000 m`.
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
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | FAIL |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 490 | 2.450 s | 24.576 cm | 1.407 cm | 46.115 cm | 41.352 cm | 82.016° | 8.000 rad/s | 70737.3 µs |

Nominal hard residual maxima: dynamics `5.727e-09`, contact acceleration `2.985e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 223.658 cm |
| authored reference vs measured CoM RMS / p95 | 220.859 / 535.788 cm |
| stance foot RMS | 197.756 cm |
| swing foot RMS | 255.565 cm |
| hand RMS | 225.862 cm |
| maximum root rotation | 179.924° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.776e-09 |
| contact acceleration residual | 4.298e-10 |
| raw max dynamics residual, including rejected ticks | 9.776e-09 |
| raw max contact residual, including rejected ticks | 4.298e-10 |
| active normal force range | 0.000–801.849 N |
| centroidal momentum-rate residual RMS / max | 65.317 / 319.934 N·m |
| point-task acceleration RMS max | 205.216 m/s² |
| frame-angular acceleration RMS max | 378.476 rad/s² |
| longest pre-contact / touchdown transition | 262 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 239 / 239 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `246.149` / `608.527 cm`.
- Virtual ZMP clipped on `69.88%` of ticks; clip-distance RMS / max `371.457` / `1114.524 cm`.
- Measured-height natural frequency min / p50 / max: `3.665` / `3.774` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.959 m`; height-floor ticks: `299`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-613.001` / `-502.969 cm`; inside on `34.62%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True` (touchdown `True`, balance `True`); final source tick `547.087`, progress `547.087` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
- Target / applied minimum rate: `0.5000` / `0.5000`; mean / p50 applied `0.6851` / `0.5000`.
- Limited / zero-rate hold ticks: `523` / `0`; maximum required landing time `1.5932 s`.
- Position / tangential / normal limiting ticks: `572` / `0` / `0`; unsafe-edge ticks `239`.
- Balance-margin limited ticks: `213`.

## Capture-aware landing

- Active target-ticks: `572`; policy updates `333`, frozen `239`.
- Maximum applied offset / root reach: `0.0800 / 0.8153 m`.
- Authored-offset / reach / slew limited ticks: `333 / 0 / 82`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `239 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.6424 m / 3.3660 / 0.0352 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 6`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `571`, multi-support `199`.
- Joint-velocity envelope active on `458` ticks; maximum active coordinates `11`; mean target/applied scale `0.505` / `0.569`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 2594.0 µs | 99643.3 µs | 189063.5 µs | 214212.0 µs | 126 | 102 | 262 | 0 | 257 | 53 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 15370.5 | 38939.3 | 471.5 | 31790.0 | 206780.7 | 213468.9 | 145988.9 | 800 | 181 | 92 | 65.1 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 126 | 2434.6 | 2566.0 | 2603.8 | 2612.1 |
| solved_with_slack | 102 | 2520.2 | 2791.8 | 4009.3 | 4521.5 |
| normal_contact_contingency | 257 | 3293.0 | 24696.4 | 34610.0 | 214212.0 |
| contact_release_contingency | 53 | 155876.8 | 191212.6 | 203965.4 | 204911.3 |
| precontact_transition | 262 | 2600.4 | 60680.0 | 72547.5 | 91777.4 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.12 | 12.0 | 14.0 | 25 | 6.21 | 13.0 | 21 | 0.0642 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 111.54/8.0/4285.4/5732 | 24746.26/1776.0/951365.5/1272504 | 2.27/25.0/31 | 1.90/24.0/30 | 15.94/213.0/272 | 0.2202 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 126 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 102 | 7.22/14.0/25 | 4.25/11.0/21 |
| normal_contact_contingency | 257 | 9.27/14.0/15 | 8.37/14.0/15 |
| contact_release_contingency | 53 | 7.79/10.5/11 | 6.49/9.5/10 |
| precontact_transition | 262 | 8.90/13.0/17 | 7.79/13.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.20/7.0/10 | 10.10/35.0/50 | 1.76/7.0/10 | 481 |
| viability | 1.84/6.0/11 | 11.70/42.0/66 | 1.57/6.0/11 | 589 |
| intent | 1.55/5.0/8 | 7.96/26.0/38 | 1.38/5.0/8 | 669 |
| preference | 1.43/6.0/20 | 12.51/61.0/140 | 1.07/6.0/19 | 549 |
| style | 1.09/3.0/6 | 8.75/23.0/48 | 0.43/3.0/6 | 288 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 572 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 490 | 310 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `572` ticks, planned normal touchdown `0` ticks, normal fallback `313` ticks.
Precontact sole-center tangential speed: p50 `3.4258 m/s`, p95 `10.5989 m/s`, max `14.4702 m/s` over 572 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 12.297 | 12.293 | 12.293 | 1.000 | 1.000 | 48.438 | 49.332 | 0.895 | 50.109 | 0.001 | 0 | 212 | 0 | 0 | 206 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2421.5 | 2544.3 | 5.00 | 33.77 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| 80–159 | 2492.1 | 2942.1 | 5.65 | 37.64 | 1.35 | 1.00 | 234.00 | 0.607 | 0.000 | 1.27e-09 | 5.03e-11 | 0 |
| 160–239 | 2517.3 | 3314.3 | 7.72 | 49.48 | 5.11 | 1.00 | 227.85 | 5.686 | 1.100 | 1.35e-09 | 5.23e-11 | 12 |
| 240–319 | 2573.0 | 4103.9 | 8.71 | 55.10 | 7.40 | 2.05 | 455.10 | 7.817 | 24.035 | 1.20e-09 | 2.07e-11 | 80 |
| 320–399 | 2551.0 | 4062.2 | 8.43 | 52.23 | 7.21 | 3.10 | 688.20 | 15.558 | 40.137 | 7.60e-10 | 1.35e-11 | 80 |
| 400–479 | 2495.1 | 71841.9 | 9.31 | 57.00 | 8.55 | 943.74 | 209509.73 | 49.006 | 39.741 | 9.23e-10 | 1.23e-11 | 80 |
| 480–559 | 26728.8 | 205427.4 | 9.46 | 56.61 | 8.71 | 145.54 | 32252.62 | 111.174 | 33.007 | 9.78e-09 | 4.30e-10 | 80 |
| 560–639 | 2044.1 | 193277.9 | 8.78 | 53.26 | 7.49 | 3.71 | 793.27 | 197.953 | 162.481 | 4.01e-09 | 2.19e-10 | 80 |
| 640–719 | 2904.2 | 5614.8 | 8.26 | 52.19 | 7.16 | 6.49 | 1401.30 | 380.794 | 376.791 | 2.83e-09 | 6.18e-11 | 80 |
| 720–799 | 6357.3 | 99240.6 | 9.84 | 62.85 | 9.15 | 7.74 | 1666.50 | 548.582 | 561.222 | 5.53e-09 | 9.94e-11 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 223.658 | 220.953 | 225.862 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
