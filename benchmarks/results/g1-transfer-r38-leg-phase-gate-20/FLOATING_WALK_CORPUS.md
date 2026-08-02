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
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 409 | 2.045 s | 10.404 cm | 3.807 cm | 32.328 cm | 31.166 cm | 43.633° | 8.000 rad/s | 68664.4 µs |

Nominal hard residual maxima: dynamics `1.309e-09`, contact acceleration `5.511e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 100.009 cm |
| authored reference vs measured CoM RMS / p95 | 97.324 / 228.570 cm |
| stance foot RMS | 74.397 cm |
| swing foot RMS | 93.881 cm |
| hand RMS | 109.442 cm |
| maximum root rotation | 179.839° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.922e-09 |
| contact acceleration residual | 1.086e-10 |
| raw max dynamics residual, including rejected ticks | 8.922e-09 |
| raw max contact residual, including rejected ticks | 1.086e-10 |
| active normal force range | 0.000–643.242 N |
| centroidal momentum-rate residual RMS / max | 60.146 / 318.176 N·m |
| point-task acceleration RMS max | 124.136 m/s² |
| frame-angular acceleration RMS max | 263.557 rad/s² |
| longest pre-contact / touchdown transition | 161 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 152 / 152 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `88.366` / `231.620 cm`.
- Virtual ZMP clipped on `57.00%` of ticks; clip-distance RMS / max `133.076` / `428.330 cm`.
- Measured-height natural frequency min / p50 / max: `3.693` / `3.775` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.861 m`; height-floor ticks: `167`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-259.674` / `-207.112 cm`; inside on `45.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `352`; policy updates `200`, frozen `152`.
- Maximum applied offset / root reach: `0.0800 / 0.8263 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 51`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `152 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.9392 m / 0.6471 / 0.0037 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 5`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `351`, multi-support `219`.
- Joint-velocity envelope active on `403` ticks; maximum active coordinates `8`; mean target/applied scale `0.602` / `0.670`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2749.3 µs | 44072.6 µs | 118601.0 µs | 193550.6 µs | 129 | 119 | 161 | 0 | 182 | 9 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 8678.2 | 21941.9 | 524.7 | 8468.3 | 183641.2 | 192559.6 | 98990.7 | 600 | 95 | 52 | 115.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2554.9 | 2778.1 | 3516.2 | 3629.5 |
| solved_with_slack | 119 | 2647.1 | 3873.4 | 4395.9 | 4636.1 |
| normal_contact_contingency | 182 | 3603.6 | 28101.0 | 34908.4 | 120359.4 |
| contact_release_contingency | 9 | 168701.9 | 186933.3 | 192227.1 | 193550.6 |
| precontact_transition | 161 | 3339.0 | 66619.0 | 73096.4 | 81359.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.64 | 11.0 | 13.0 | 14 | 5.39 | 12.0 | 13 | 0.1363 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 194.16/1706.2/4052.2/4647 | 42892.78/375288.0/899595.1/1031634 | 1.23/10.0/16 | 0.93/9.0/15 | 7.63/72.0/131 | 0.5243 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 119 | 7.54/13.8/14 | 4.67/11.0/12 |
| normal_contact_contingency | 182 | 8.76/13.0/14 | 7.76/12.0/13 |
| contact_release_contingency | 9 | 7.89/9.0/9 | 6.78/7.9/8 |
| precontact_transition | 161 | 8.54/12.4/13 | 7.48/12.0/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.02/5.0/6 | 8.64/25.0/29 | 1.52/5.0/6 | 317 |
| viability | 1.71/7.0/10 | 10.56/42.0/50 | 1.30/7.0/9 | 372 |
| intent | 1.58/4.0/9 | 8.63/24.0/45 | 1.35/4.0/9 | 470 |
| preference | 1.26/5.0/7 | 10.42/41.0/56 | 0.82/5.0/6 | 359 |
| style | 1.08/3.0/6 | 9.41/23.0/41 | 0.39/2.0/6 | 207 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 352 | 0 | 219 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 409 | 191 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `352` ticks, planned normal touchdown `0` ticks, normal fallback `194` ticks.
Precontact sole-center tangential speed: p50 `2.2341 m/s`, p95 `7.9976 m/s`, max `11.1915 m/s` over 352 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.207 | 5.207 | 5.207 | 1.000 | 1.000 | 48.141 | 48.504 | 0.363 | 49.195 | 0.001 | 0 | 92 | 0 | 0 | 31 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2533.3 | 2676.2 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2645.5 | 3616.6 | 5.40 | 36.75 | 0.92 | 1.00 | 234.00 | 0.120 | 0.000 | 9.54e-10 | 5.24e-11 | 0 |
| 120–179 | 2617.4 | 3100.8 | 6.07 | 40.23 | 2.13 | 1.00 | 234.00 | 1.294 | 0.000 | 1.10e-09 | 5.51e-11 | 0 |
| 180–239 | 2692.4 | 4523.3 | 8.25 | 50.10 | 5.62 | 1.00 | 229.80 | 6.424 | 0.012 | 9.40e-10 | 4.25e-11 | 0 |
| 240–299 | 2690.2 | 4607.3 | 8.12 | 50.45 | 6.53 | 2.87 | 636.40 | 5.874 | 17.133 | 1.05e-09 | 1.59e-11 | 52 |
| 300–359 | 3333.0 | 40154.2 | 8.80 | 53.58 | 7.95 | 212.08 | 47082.50 | 10.754 | 22.806 | 1.10e-09 | 2.17e-11 | 60 |
| 360–419 | 28989.3 | 97349.3 | 8.78 | 52.37 | 7.77 | 1590.32 | 351696.10 | 30.562 | 39.624 | 6.27e-10 | 1.22e-11 | 60 |
| 420–479 | 3230.1 | 183790.1 | 8.80 | 51.47 | 7.55 | 118.43 | 25579.40 | 105.364 | 91.406 | 4.09e-09 | 9.72e-11 | 60 |
| 480–539 | 4062.0 | 49116.0 | 8.28 | 51.02 | 7.30 | 7.77 | 1676.80 | 187.442 | 148.031 | 8.92e-09 | 8.65e-11 | 60 |
| 540–599 | 3111.6 | 7650.9 | 8.88 | 57.15 | 8.12 | 6.13 | 1324.80 | 229.464 | 181.654 | 2.87e-09 | 1.09e-10 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 100.009 | 81.025 | 109.442 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
