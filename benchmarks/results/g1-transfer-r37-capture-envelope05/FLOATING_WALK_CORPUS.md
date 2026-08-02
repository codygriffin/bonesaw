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
- Joint-velocity envelope: `viability` priority with weight `0.050`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `feedback` measured-phase policy with immediate engagement and bounded release over `150` ticks.
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
| 464 | 2.320 s | 15.683 cm | 15.856 cm | 26.050 cm | 41.820 cm | 41.075° | 8.000 rad/s | 5258.5 µs |

Nominal hard residual maxima: dynamics `3.633e-09`, contact acceleration `1.805e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 64.509 cm |
| authored reference vs measured CoM RMS / p95 | 58.057 / 141.713 cm |
| stance foot RMS | 37.967 cm |
| swing foot RMS | 70.979 cm |
| hand RMS | 82.819 cm |
| maximum root rotation | 179.871° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.889e-09 |
| contact acceleration residual | 2.662e-10 |
| raw max dynamics residual, including rejected ticks | 7.889e-09 |
| raw max contact residual, including rejected ticks | 2.662e-10 |
| active normal force range | 0.000–820.354 N |
| centroidal momentum-rate residual RMS / max | 50.805 / 220.846 N·m |
| point-task acceleration RMS max | 151.117 m/s² |
| frame-angular acceleration RMS max | 239.017 rad/s² |
| longest pre-contact / touchdown transition | 236 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 172 / 172 ticks |

## DCM and virtual-ZMP balance

- DCM RMS / p95: `54.354` / `112.587 cm`.
- Virtual ZMP clipped on `59.33%` of ticks; clip-distance RMS / max `85.173` / `191.949 cm`.
- Measured-height natural frequency min / p50 / max: `3.686` / `3.721` / `7.004 rad/s`.
- Minimum measured CoM height: `-0.432 m`; height-floor ticks: `91`.
- CoM command acceleration p95 / max: `25.000` / `25.000 m/s²`; support hull `4–4` vertices.
- Signed measured DCM support margin min / p05: `-115.249` / `-109.947 cm`; inside on `43.67%` of ticks.

## Capture-aware landing

- Active target-ticks: `372`; policy updates `200`, frozen `172`.
- Maximum applied offset / root reach: `0.0800 / 0.8632 m`.
- Authored-offset / reach / slew limited ticks: `200 / 0 / 35`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `172 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.4343 m / 0.6516 / 0.0045 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 26`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `29`, precontact `371`, multi-support `199`.
- Joint-velocity envelope active on `447` ticks; maximum active coordinates `10`; mean target/applied scale `0.690` / `0.759`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 2602.2 µs | 16139.6 µs | 178409.8 µs | 220584.8 µs | 129 | 99 | 236 | 0 | 113 | 23 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 10222.0 | 31723.5 | 343.9 | 11765.9 | 216601.5 | 220186.4 | 96389.9 | 600 | 92 | 28 | 97.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 129 | 2505.6 | 2645.8 | 2746.2 | 2781.1 |
| solved_with_slack | 99 | 2564.2 | 3085.5 | 3241.5 | 3824.7 |
| normal_contact_contingency | 113 | 5806.3 | 16533.9 | 74048.6 | 158252.0 |
| contact_release_contingency | 23 | 168791.2 | 213679.9 | 219121.8 | 220584.8 |
| precontact_transition | 236 | 2699.5 | 4568.6 | 7432.7 | 16447.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.70 | 12.0 | 15.0 | 16 | 5.34 | 13.0 | 16 | 0.0185 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 26.84/8.0/8.0/4741 | 5809.68/1776.0/1776.0/1024056 | 1.88/15.0/22 | 1.54/14.0/21 | 12.85/123.0/194 | 0.1442 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 129 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 99 | 6.86/11.0/11 | 3.46/8.0/8 |
| normal_contact_contingency | 113 | 9.68/15.0/16 | 8.89/14.0/15 |
| contact_release_contingency | 23 | 7.35/9.8/10 | 5.13/7.8/8 |
| precontact_transition | 236 | 8.62/13.0/16 | 7.38/13.0/16 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.91/6.0/7 | 7.98/25.0/35 | 1.37/6.0/7 | 302 |
| viability | 1.73/6.0/8 | 11.13/42.0/54 | 1.34/6.0/8 | 381 |
| intent | 1.60/5.0/9 | 8.35/25.0/45 | 1.33/5.0/9 | 448 |
| preference | 1.37/7.0/9 | 12.23/62.1/88 | 0.91/7.0/8 | 336 |
| style | 1.09/4.0/7 | 9.20/24.0/43 | 0.40/3.0/6 | 198 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 29 | 372 | 0 | 199 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 464 | 136 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `372` ticks, planned normal touchdown `0` ticks, normal fallback `139` ticks.
Precontact sole-center tangential speed: p50 `2.0468 m/s`, p95 `9.3193 m/s`, max `13.3287 m/s` over 372 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 6.133 | 6.132 | 6.132 | 1.000 | 1.000 | 46.984 | 47.352 | 0.367 | 49.305 | 0.001 | 0 | 93 | 0 | 0 | 74 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2482.4 | 2611.8 | 5.00 | 33.57 | 0.00 | 1.00 | 234.00 | 0.000 | 0.000 | 1.31e-09 | 5.04e-11 | 0 |
| 60–119 | 2555.2 | 2815.9 | 5.40 | 36.77 | 0.93 | 1.00 | 234.00 | 0.119 | 0.000 | 1.17e-09 | 4.37e-11 | 0 |
| 120–179 | 2605.2 | 3473.6 | 6.20 | 41.90 | 2.20 | 1.00 | 234.00 | 1.274 | 0.000 | 1.20e-09 | 5.60e-11 | 0 |
| 180–239 | 2361.0 | 3141.5 | 7.07 | 43.72 | 3.77 | 1.00 | 225.80 | 5.996 | 0.397 | 7.46e-10 | 4.97e-11 | 12 |
| 240–299 | 2467.0 | 3257.4 | 8.12 | 51.45 | 6.35 | 1.00 | 222.00 | 6.701 | 20.097 | 1.04e-09 | 3.46e-11 | 60 |
| 300–359 | 3503.7 | 4783.2 | 8.83 | 54.97 | 7.60 | 5.78 | 1283.90 | 9.304 | 24.289 | 7.78e-10 | 7.70e-12 | 60 |
| 360–419 | 2237.6 | 3256.7 | 8.90 | 54.42 | 7.83 | 1.00 | 222.00 | 22.342 | 17.648 | 1.14e-09 | 3.03e-11 | 60 |
| 420–479 | 4017.9 | 108591.8 | 8.48 | 53.63 | 7.83 | 240.62 | 52003.50 | 46.850 | 47.609 | 3.63e-09 | 1.81e-10 | 60 |
| 480–539 | 15352.0 | 216661.3 | 9.92 | 58.68 | 8.75 | 8.00 | 1709.60 | 112.218 | 69.236 | 7.89e-09 | 2.66e-10 | 60 |
| 540–599 | 3394.5 | 11940.2 | 9.12 | 59.88 | 8.18 | 8.00 | 1728.00 | 161.734 | 133.968 | 5.18e-09 | 3.36e-11 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 0.75x | 600 | 64.509 | 51.296 | 82.819 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
