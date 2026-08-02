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
- Joint-velocity envelope: `viability` priority with weight `0.220`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| `no_infeasible_or_failed_ticks` | FAIL |
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

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `touchdown_admission_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 433 | 2.165 s | 13.331 cm | 5.182 cm | 32.446 cm | 36.503 cm | 61.842° | 8.000 rad/s | 106328.3 µs |

Nominal hard residual maxima: dynamics `3.180e-09`, contact acceleration `6.044e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 76.426 cm |
| authored reference vs measured CoM RMS / p95 | 70.791 / 161.092 cm |
| stance foot RMS | 46.125 cm |
| swing foot RMS | 68.621 cm |
| hand RMS | 98.463 cm |
| maximum root rotation | 179.707° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 8.225e-09 |
| contact acceleration residual | 8.075e-11 |
| raw max dynamics residual, including rejected ticks | 9.701e+02 |
| raw max contact residual, including rejected ticks | 8.075e-11 |
| active normal force range | 0.000–875.715 N |
| centroidal momentum-rate residual RMS / max | 52.836 / 363.165 N·m |
| point-task acceleration RMS max | 186.492 m/s² |
| frame-angular acceleration RMS max | 403.441 rad/s² |
| longest pre-contact / touchdown transition | 205 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `20.444` / `33.770 cm`.
- Virtual ZMP clipped on `63.00%` of ticks; clip-distance RMS / max `48.941` / `89.710 cm`.
- Measured-height natural frequency min / p50 / max: `3.677` / `3.769` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.880 m`; height-floor ticks: `146`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-66.032` / `-60.928 cm`; inside on `43.00%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8584 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 48`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.5652 m / 0.2988 / 0.0017 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 16`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `432` ticks; maximum active coordinates `8`; mean target/applied scale `0.627` / `0.729`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2663.7 µs | 321294.3 µs | 326192.3 µs | 431918.0 µs | 130 | 98 | 205 | 0 | 63 | 19 | 85 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 56938.5 | 113931.5 | 517.6 | 320194.6 | 410017.2 | 429727.9 | 96389.2 | 600 | 152 | 128 | 17.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 130 | 2522.5 | 2680.5 | 2724.5 | 2993.4 |
| solved_with_slack | 98 | 2581.7 | 2787.3 | 3227.0 | 4504.2 |
| primal_infeasible | 85 | 320758.2 | 331336.8 | 401205.7 | 431918.0 |
| normal_contact_contingency | 63 | 3370.4 | 74026.7 | 104150.3 | 143721.2 |
| contact_release_contingency | 19 | 197829.4 | 223810.7 | 224184.3 | 224277.8 |
| precontact_transition | 205 | 3283.8 | 33725.3 | 110066.3 | 116414.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.53 | 12.0 | 13.0 | 23 | 4.39 | 12.0 | 20 | -0.7151 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1124.18/6720.0/6720.0/7080 | 237660.83/1411200.0/1411200.0/1571760 | 7.50/47.0/47 | 7.16/46.0/46 | 60.63/390.0/390 | 0.9238 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 130 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 98 | 6.92/9.4/23 | 3.56/8.4/20 |
| primal_infeasible | 85 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 63 | 9.90/14.0/14 | 9.13/13.0/13 |
| contact_release_contingency | 19 | 7.79/9.8/10 | 5.79/8.0/8 |
| precontact_transition | 205 | 8.86/12.0/14 | 7.81/12.0/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.62/5.0/7 | 7.17/25.0/32 | 1.13/5.0/7 | 242 |
| viability | 1.44/6.0/8 | 8.77/41.0/58 | 1.06/6.0/8 | 298 |
| intent | 1.36/4.0/9 | 7.25/25.0/49 | 1.09/4.0/9 | 368 |
| preference | 1.17/6.0/18 | 10.42/56.0/126 | 0.78/5.0/18 | 294 |
| style | 0.94/3.0/5 | 8.03/21.0/37 | 0.33/3.0/5 | 161 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 433 | 167 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `170` ticks.
Precontact sole-center tangential speed: p50 `2.5241 m/s`, p95 `11.7334 m/s`, max `11.7334 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 34.163 | 34.137 | 34.136 | 0.999 | 0.999 | 46.945 | 47.375 | 0.430 | 48.891 | 0.001 | 0 | 93 | 0 | 0 | 402 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2510.5 | 2674.1 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2563.1 | 2942.1 | 5.37 | 36.12 | 0.63 | 1.00 | 234.00 | 0.156 | 0.000 | 1.46e-09 | 4.89e-11 | 0 |
| 120–179 | 2604.3 | 3727.3 | 6.18 | 40.02 | 1.98 | 1.00 | 234.00 | 1.470 | 0.000 | 1.32e-09 | 5.10e-11 | 0 |
| 180–239 | 2355.5 | 3186.8 | 7.23 | 44.30 | 4.62 | 1.00 | 225.80 | 6.400 | 2.110 | 1.13e-09 | 4.98e-11 | 12 |
| 240–299 | 3387.8 | 4517.9 | 8.67 | 55.32 | 7.35 | 4.50 | 999.00 | 6.711 | 25.252 | 7.82e-10 | 1.32e-11 | 60 |
| 300–359 | 2491.8 | 4132.1 | 8.60 | 55.27 | 7.52 | 2.28 | 506.90 | 7.542 | 24.917 | 8.80e-10 | 1.92e-11 | 60 |
| 360–419 | 3487.7 | 43897.1 | 8.97 | 55.57 | 8.12 | 232.22 | 51552.10 | 26.174 | 23.632 | 8.67e-10 | 1.14e-11 | 60 |
| 420–479 | 10728.2 | 127610.3 | 10.33 | 68.98 | 9.57 | 1462.35 | 319882.50 | 72.468 | 46.566 | 3.18e-09 | 6.04e-11 | 60 |
| 480–539 | 218797.3 | 332747.1 | 4.92 | 27.38 | 4.15 | 2816.43 | 591540.00 | 154.614 | 105.968 | 9.70e+02 | 8.07e-11 | 60 |
| 540–599 | 320604.7 | 410346.3 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 168.587 | 120.814 | 9.70e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 76.426 | 54.603 | 98.463 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
