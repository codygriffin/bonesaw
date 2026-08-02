# Bonesaw floating G1 synthetic-step acceptance

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: deterministic, predeclared G1-sized left step; no mocap or learned policy contributes contact labels or target motion.
- Contact schedule: liftoff tick `53`, touchdown tick `93`, step `0.010 m`, clearance `0.010 m`.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.010 m` forward per `0.800 s` source cycle.
- Applied mean forward speed: `0.012 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `constant synthetic phase rate` in `0.8 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Finite-support CoP constraint: `enabled`; required `5.000 mm`, measured minimum `5.000 mm` over `154` loaded ticks.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
- Centroidal angular-momentum damping: `intent` priority with weight `0.000` and `1.000 Hz` response.
- Pre-contact viability preview: `40` ticks (`0.200 s`) with a receding cubic landing law capped at `25.000 m/s²`; the authored edge requests touchdown, while measured sole proximity and velocity admit physical contact.
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
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `finite_support_margin_respected` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 134 | 0.670 s | 3.542 cm | 5.159 cm | 0.639 cm | 24.660 cm | 15.667° | 8.000 rad/s | 57175.7 µs |

Nominal hard residual maxima: dynamics `1.482e-09`, contact acceleration `5.332e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 6.921 cm |
| authored reference vs measured CoM RMS / p95 | 6.685 / 14.315 cm |
| stance foot RMS | 10.795 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 29.238 cm |
| maximum root rotation | 44.133° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.068e-09 |
| contact acceleration residual | 2.951e-10 |
| raw max dynamics residual, including rejected ticks | 7.068e-09 |
| raw max contact residual, including rejected ticks | 2.951e-10 |
| active normal force range | 0.000–362.800 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 130.958 m/s² |
| frame-angular acceleration RMS max | 94.607 rad/s² |
| longest pre-contact / touchdown transition | 40 / 41 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `159.000`, progress `159.000` ticks.
- First post-liftoff authored touchdown source tick: `93`; reached before trace end: `True`.
- Target / applied minimum rate: `1.0000` / `1.0000`; mean / p50 applied `1.0000` / `1.0000`.
- Limited / zero-rate hold ticks: `0` / `0`; maximum required landing time `0.0000 s`.
- Position / tangential / normal limiting ticks: `0` / `0` / `0`; unsafe-edge ticks `0`.
- Balance-margin limited ticks: `0`.

## Capture-aware landing

- Active target-ticks: `40`; policy updates `0`, frozen `0`.
- Maximum applied offset / root reach: `0.0000 / 0.7933 m`.
- Authored-offset / reach / slew limited ticks: `0 / 0 / 0`.
- Authored geometry outside configured reach: `0` target-ticks.
- Pending-touchdown observations / jointly viable: `0 / 0` target-ticks.
- Minimum position error / tangential speed / normal speed: `0.0000 m / 0.0000 / 0.0000 m/s`.
- Individually viable position / tangential / normal target-ticks: `0 / 0 / 0`.

## Measured contact-phase authority

- Phase ticks: unsupported `1`, single support `0`, precontact `40`, multi-support `119`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.8 s | 3314.7 µs | 61035.5 µs | 293170.9 µs | 378519.5 µs | 53 | 0 | 40 | 41 | 20 | 6 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20335.9 | 52450.6 | 1330.9 | 33970.5 | 368742.3 | 377541.8 | 183989.0 | 160 | 63 | 37 | 49.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2383.0 | 3340.6 | 3424.0 | 3501.6 |
| normal_contact_contingency | 20 | 11948.8 | 61511.9 | 157394.3 | 181364.9 |
| contact_release_contingency | 6 | 260289.5 | 363146.6 | 375444.9 | 378519.5 |
| touchdown_transition | 41 | 24858.2 | 49455.0 | 61662.4 | 62118.4 |
| precontact_transition | 40 | 2085.7 | 5821.2 | 6350.4 | 6561.6 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 8.50 | 17.0 | 19.4 | 20 | 6.23 | 19.4 | 20 | 0.1481 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 4.44/8.0/8.0/47 | 1023.98/1888.0/1888.0/9870 | 6.01/32.2/34 | 5.56/31.2/33 | 49.80/306.9/333 | 0.4900 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| normal_contact_contingency | 20 | 9.75/16.4/17 | 9.40/16.4/17 |
| contact_release_contingency | 6 | 7.83/9.0/9 | 6.83/8.0/8 |
| touchdown_transition | 41 | 13.24/20.0/20 | 12.98/20.0/20 |
| precontact_transition | 40 | 9.07/11.6/12 | 5.90/11.6/12 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 2.65/12.0/15 | 10.27/48.0/60 | 2.07/12.0/15 | 70 |
| viability | 1.27/7.0/11 | 5.25/23.9/44 | 1.05/7.0/11 | 78 |
| intent | 1.65/4.4/6 | 3.41/10.8/12 | 1.14/4.4/6 | 98 |
| preference | 1.46/5.4/6 | 10.28/37.8/41 | 1.11/5.4/6 | 107 |
| style | 1.46/6.4/9 | 14.36/66.5/99 | 0.86/6.4/8 | 77 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 41 | 53 | 26 |
| right_ankle_roll_link | 0 | 0 | 0 | 134 | 26 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `41` ticks, normal fallback `29` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `1.7066 m/s`, p95 `4.6725 m/s`, max `5.2791 m/s` over 41 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.254 | 3.252 | 3.252 | 0.999 | 0.999 | 45.633 | 45.797 | 0.164 | 45.797 | 0.001 | 0 | 33 | 0 | 0 | 117 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 3267.8 | 3479.2 | 4.00 | 24.00 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2341.1 | 2362.9 | 4.00 | 24.00 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 5.68e-14 | 4.89e-16 | 0 |
| 32–47 | 2491.5 | 3114.3 | 4.00 | 24.00 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 5.68e-14 | 5.01e-16 | 0 |
| 48–63 | 2292.5 | 2706.8 | 6.88 | 36.25 | 2.75 | 1.00 | 231.00 | 0.000 | 0.176 | 6.97e-11 | 1.13e-12 | 11 |
| 64–79 | 2002.3 | 2345.9 | 9.56 | 41.56 | 6.31 | 1.00 | 226.00 | 0.009 | 0.635 | 5.60e-12 | 6.93e-14 | 16 |
| 80–95 | 4900.3 | 17848.0 | 10.12 | 45.94 | 8.31 | 4.94 | 1130.88 | 0.195 | 0.280 | 4.79e-10 | 2.36e-11 | 13 |
| 96–111 | 31492.1 | 35212.2 | 15.50 | 70.31 | 15.50 | 8.00 | 1888.00 | 2.638 | 2.624 | 3.58e-10 | 1.83e-11 | 0 |
| 112–127 | 28018.0 | 61947.4 | 11.94 | 68.19 | 11.50 | 8.00 | 1888.00 | 7.603 | 10.315 | 1.48e-09 | 5.33e-11 | 0 |
| 128–143 | 19927.9 | 158256.2 | 10.56 | 58.50 | 10.25 | 8.00 | 1858.00 | 11.663 | 14.184 | 2.78e-09 | 9.37e-11 | 10 |
| 144–159 | 18936.2 | 369295.7 | 8.44 | 42.94 | 7.69 | 10.44 | 2291.88 | 16.678 | 26.556 | 7.07e-09 | 2.95e-10 | 16 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 6.921 | 10.100 | 29.238 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
