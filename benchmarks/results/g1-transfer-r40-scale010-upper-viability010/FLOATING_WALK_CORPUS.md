# Bonesaw floating CMU walking corpus

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns source/reference construction, cadence, fixed-shape arrays, and metrics. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: CMU subject 37, trial 1 (`slow walk`, 120 Hz).
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.103 m` forward per `1.308 s` source cycle.
- Applied mean forward speed: `0.079 m/s` (`0.10×` forward, `0.10×` lateral retarget scale).
- Cadence schedule: `[0.75, 1.0, 1.25, 1.0]` in `8.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `dcm-backward-preview` `DCM` reference at `viability` priority with weight `0.100` and `2.000 Hz` response through `dcm-zmp` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `viability` priority with weight `0.100` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `1.000` and `1.000 Hz` response.
- Pre-contact viability preview: `200` ticks (`1.000 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
- Capture landing retarget: `enabled`; authored offset ≤ `0.080 m`, root reach ≤ `0.900 m`, anchor speed ≤ `0.500 m/s`.
- Landing commitment: freeze the retargeted anchor for the final `0` scheduled ticks and throughout delayed admission.
- Coupled touchdown phase retiming: `enabled`; minimum rate `0.000`, guard `0.020 s`, engagement / release `0 / 100` ticks. Rust samples root, CoM, all endpoint jets, and contact intent from one explicit cursor.
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
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | FAIL |
| `stance_foot_tracking_rms_le_2cm` | FAIL |
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `retiming_reaches_first_authored_touchdown` | FAIL |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `retiming_reaches_first_authored_touchdown`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 481 | 2.405 s | 11.749 cm | 1.430 cm | 30.390 cm | 24.364 cm | 44.629° | 8.000 rad/s | 30053.5 µs |

Nominal hard residual maxima: dynamics `1.493e-09`, contact acceleration `6.685e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 45.844 cm |
| authored reference vs measured CoM RMS / p95 | 38.347 / 97.499 cm |
| stance foot RMS | 36.254 cm |
| swing foot RMS | 36.940 cm |
| hand RMS | 60.548 cm |
| maximum root rotation | 162.519° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 6.246e-09 |
| contact acceleration residual | 1.381e-10 |
| raw max dynamics residual, including rejected ticks | 5.200e+02 |
| raw max contact residual, including rejected ticks | 1.381e-10 |
| active normal force range | 0.000–350.619 N |
| centroidal momentum-rate residual RMS / max | 59.721 / 149.985 N·m |
| point-task acceleration RMS max | 112.001 m/s² |
| frame-angular acceleration RMS max | 159.497 rad/s² |
| longest pre-contact / touchdown transition | 253 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `27.520` / `53.684 cm`.
- Virtual ZMP clipped on `59.50%` of ticks; clip-distance RMS / max `53.378` / `113.292 cm`.
- Measured-height natural frequency min / p50 / max: `3.686` / `3.768` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.129 m`; height-floor ticks: `89`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-72.412` / `-66.034 cm`; inside on `47.50%` of ticks.

## Coupled touchdown phase retiming

- Enabled: `True`; final source tick `424.879`, progress `424.879` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `False`.
- Target / applied minimum rate: `0.0268` / `0.0270`; mean / p50 applied `0.7098` / `1.0000`.
- Limited / zero-rate hold ticks: `213` / `0`; maximum required landing time `0.7197 s`.
- Position / tangential / normal limiting ticks: `372` / `0` / `0`; unsafe-edge ticks `0`.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `372`, frozen `0`.
- Maximum applied offset / root reach: `0.0800 / 0.8301 m`.
- Authored-offset / reach / slew limited ticks: `338 / 0 / 115`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2435.0 µs | 277236.9 µs | 279559.5 µs | 345476.1 µs | 199 | 29 | 253 | 0 | 29 | 26 | 64 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 40602.5 | 88982.6 | 327.0 | 276646.3 | 322552.6 | 343183.8 | 72308.1 | 600 | 140 | 106 | 24.6 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 199 | 2428.1 | 2516.4 | 2534.7 | 2568.8 |
| solved_with_slack | 29 | 1869.8 | 2822.8 | 3454.6 | 3608.4 |
| primal_infeasible | 64 | 277208.9 | 280346.4 | 321366.2 | 345476.1 |
| normal_contact_contingency | 29 | 15158.3 | 79217.1 | 161465.1 | 192595.0 |
| contact_release_contingency | 26 | 171686.4 | 196046.0 | 199354.9 | 200349.9 |
| precontact_transition | 253 | 2261.8 | 9458.0 | 63655.3 | 72162.5 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.02 | 10.0 | 12.0 | 14 | 2.74 | 10.0 | 13 | -0.6795 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 805.46/6720.0/6720.0/6720 | 169900.14/1411200.0/1411200.0/1411200 | 4.93/38.0/38 | 4.72/37.0/37 | 41.16/324.0/324 | 0.9122 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 199 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 29 | 7.14/13.2/14 | 4.14/11.3/13 |
| primal_infeasible | 64 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 29 | 9.00/12.0/12 | 7.17/10.7/11 |
| contact_release_contingency | 26 | 7.15/9.8/10 | 4.27/6.8/7 |
| precontact_transition | 253 | 7.75/12.0/13 | 4.77/9.5/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.55/6.0/8 | 6.67/25.0/40 | 1.01/6.0/8 | 210 |
| viability | 1.59/6.0/8 | 10.91/42.0/52 | 1.20/6.0/8 | 309 |
| intent | 0.94/2.0/5 | 3.71/11.0/17 | 0.11/2.0/4 | 43 |
| preference | 0.93/2.0/7 | 4.04/10.0/35 | 0.07/2.0/6 | 24 |
| style | 1.00/4.0/7 | 8.18/31.0/53 | 0.36/4.0/7 | 160 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 481 | 119 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `122` ticks.
Precontact sole-center tangential speed: p50 `2.6223 m/s`, p95 `6.9778 m/s`, max `10.2640 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 24.362 | 24.357 | 24.357 | 1.000 | 1.000 | 49.484 | 49.926 | 0.441 | 50.582 | 0.001 | 0 | 112 | 0 | 0 | 371 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2405.7 | 2551.0 | 5.00 | 29.63 | 0.00 | 1.00 | 234.00 | 0.279 | 0.000 | 1.32e-09 | 5.84e-11 | 0 |
| 60–119 | 2439.7 | 2528.1 | 5.00 | 30.47 | 0.00 | 1.00 | 234.00 | 1.171 | 0.000 | 1.42e-09 | 4.32e-11 | 0 |
| 120–179 | 2433.2 | 2530.5 | 5.00 | 29.98 | 0.00 | 1.00 | 234.00 | 3.342 | 0.000 | 7.74e-10 | 5.95e-11 | 0 |
| 180–239 | 2329.9 | 3284.3 | 6.43 | 35.72 | 2.63 | 1.23 | 277.60 | 8.361 | 3.061 | 9.86e-10 | 2.34e-11 | 12 |
| 240–299 | 2350.3 | 3384.2 | 7.43 | 41.85 | 3.75 | 1.00 | 222.00 | 8.285 | 17.183 | 1.23e-09 | 1.92e-11 | 60 |
| 300–359 | 2326.8 | 3294.4 | 7.75 | 43.05 | 4.68 | 1.00 | 222.00 | 5.322 | 30.572 | 1.13e-09 | 1.32e-11 | 60 |
| 360–419 | 2058.0 | 3279.3 | 7.65 | 40.12 | 5.10 | 1.00 | 222.00 | 5.748 | 18.164 | 7.52e-10 | 1.49e-11 | 60 |
| 420–479 | 2092.5 | 70944.5 | 8.28 | 45.83 | 5.83 | 360.45 | 80019.90 | 29.240 | 24.562 | 1.49e-09 | 6.69e-11 | 60 |
| 480–539 | 100976.3 | 277752.0 | 7.62 | 38.42 | 5.43 | 966.87 | 206135.90 | 84.462 | 52.999 | 5.20e+02 | 1.38e-10 | 60 |
| 540–599 | 277154.4 | 322897.0 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 113.206 | 91.269 | 5.20e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 45.844 | 36.485 | 60.548 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
