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
| 408 | 2.040 s | 13.893 cm | 0.815 cm | 26.966 cm | 31.256 cm | 70.384° | 8.000 rad/s | 14384.6 µs |

Nominal hard residual maxima: dynamics `8.820e-09`, contact acceleration `3.738e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 160.333 cm |
| authored reference vs measured CoM RMS / p95 | 159.441 / 327.898 cm |
| stance foot RMS | 156.718 cm |
| swing foot RMS | 173.166 cm |
| hand RMS | 171.778 cm |
| maximum root rotation | 174.911° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 9.747e-09 |
| contact acceleration residual | 3.738e-10 |
| raw max dynamics residual, including rejected ticks | 9.747e-09 |
| raw max contact residual, including rejected ticks | 3.738e-10 |
| active normal force range | 0.000–820.651 N |
| centroidal momentum-rate residual RMS / max | 70.283 / 315.216 N·m |
| point-task acceleration RMS max | 144.738 m/s² |
| frame-angular acceleration RMS max | 358.132 rad/s² |
| longest pre-contact / touchdown transition | 180 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 204 / 182 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `171.953` / `384.667 cm`.
- Virtual ZMP clipped on `75.12%` of ticks; clip-distance RMS / max `395.452` / `1082.838 cm`.
- Measured-height natural frequency min / p50 / max: `2.216` / `3.783` / `7.004 rad/s`.
- Minimum measured CoM height: `-1.125 m`; height-floor ticks: `214`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-415.651` / `-392.007 cm`; inside on `24.88%` of ticks.

## Capture-aware landing

- Active target-ticks: `572`; policy updates `368`, frozen `204`.
- Maximum applied offset / root reach: `0.0800 / 0.8969 m`.
- Authored-offset / reach / slew limited ticks: `368 / 0 / 85`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `204 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.5245 m / 1.8903 / 0.0311 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 9`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `571`, multi-support `199`.
- Joint-velocity envelope active on `292` ticks; maximum active coordinates `10`; mean target/applied scale `0.454` / `0.525`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 800 | 4.0 s | 2752.5 µs | 164277.7 µs | 201934.0 µs | 218044.6 µs | 160 | 68 | 180 | 0 | 327 | 65 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 22924.4 | 47538.0 | 856.5 | 61028.9 | 213268.8 | 217567.0 | 102938.9 | 800 | 289 | 159 | 43.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 160 | 2480.7 | 2606.2 | 3254.8 | 3325.4 |
| solved_with_slack | 68 | 2470.9 | 2893.0 | 3983.2 | 4019.0 |
| normal_contact_contingency | 327 | 9468.6 | 60126.7 | 74748.6 | 114813.0 |
| contact_release_contingency | 65 | 172007.5 | 210518.1 | 214219.2 | 218044.6 |
| precontact_transition | 180 | 2385.4 | 5240.0 | 106006.4 | 108381.1 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.25 | 12.0 | 13.0 | 16 | 6.13 | 13.0 | 14 | 0.0529 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 365.80/3336.5/4487.5/6793 | 79170.52/720684.0/969291.4/1467288 | 2.17/17.0/25 | 1.87/16.0/24 | 15.41/132.0/205 | 0.2396 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 160 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 68 | 7.43/13.0/13 | 4.41/10.0/10 |
| normal_contact_contingency | 327 | 9.78/14.7/16 | 8.86/13.7/14 |
| contact_release_contingency | 65 | 7.60/10.4/11 | 6.26/9.4/10 |
| precontact_transition | 180 | 8.89/13.0/13 | 7.23/12.2/13 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.22/6.0/12 | 9.80/29.0/48 | 1.81/6.0/12 | 487 |
| viability | 1.74/6.0/9 | 10.55/36.0/56 | 1.43/6.0/9 | 569 |
| intent | 1.57/5.0/9 | 7.58/27.0/45 | 1.35/5.0/8 | 634 |
| preference | 1.57/6.0/7 | 14.87/60.0/78 | 1.12/6.0/7 | 487 |
| style | 1.14/3.0/5 | 8.40/24.0/42 | 0.42/3.0/5 | 257 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 572 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 408 | 392 |
| left_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 800 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `572` ticks, planned normal touchdown `0` ticks, normal fallback `395` ticks.
Precontact sole-center tangential speed: p50 `2.6989 m/s`, p95 `6.9349 m/s`, max `11.6419 m/s` over 572 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 18.340 | 18.334 | 18.334 | 1.000 | 1.000 | 48.320 | 48.812 | 0.492 | 49.594 | 0.001 | 0 | 125 | 0 | 0 | 301 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–79 | 2481.3 | 2646.3 | 5.00 | 32.31 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.23e-09 | 5.86e-11 | 0 |
| 80–159 | 2479.4 | 3322.1 | 5.06 | 31.59 | 0.10 | 1.00 | 234.00 | 0.000 | 0.000 | 1.44e-09 | 5.70e-11 | 0 |
| 160–239 | 2332.5 | 3976.8 | 7.59 | 41.59 | 4.42 | 1.00 | 227.85 | 2.151 | 0.034 | 1.10e-09 | 4.65e-11 | 12 |
| 240–319 | 2325.0 | 3300.2 | 8.40 | 51.73 | 6.15 | 1.00 | 222.00 | 4.420 | 16.527 | 1.08e-09 | 1.74e-11 | 80 |
| 320–399 | 2512.6 | 7234.7 | 9.24 | 59.29 | 8.36 | 11.39 | 2528.03 | 26.845 | 23.953 | 1.01e-09 | 1.64e-11 | 80 |
| 400–479 | 50440.3 | 204170.1 | 10.07 | 64.49 | 9.15 | 1742.46 | 377847.22 | 107.446 | 48.687 | 8.82e-09 | 3.74e-10 | 80 |
| 480–559 | 4150.0 | 213103.0 | 8.14 | 48.49 | 7.14 | 4.76 | 1027.12 | 189.239 | 136.788 | 9.75e-09 | 2.36e-10 | 80 |
| 560–639 | 3213.5 | 76843.6 | 8.60 | 55.99 | 7.44 | 718.56 | 155209.50 | 187.629 | 176.435 | 1.81e-09 | 1.22e-11 | 80 |
| 640–719 | 16937.8 | 204048.1 | 9.39 | 55.12 | 8.45 | 7.72 | 1649.85 | 249.196 | 266.894 | 7.77e-09 | 2.11e-10 | 80 |
| 720–799 | 10168.1 | 60707.4 | 10.97 | 71.38 | 10.11 | 1169.10 | 252525.60 | 334.159 | 375.886 | 2.28e-09 | 7.91e-11 | 80 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 800 | 160.333 | 162.966 | 171.778 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
