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
- Finite-support CoP constraint: `enabled`; required `5.000 mm`, measured minimum `5.000 mm` over `160` loaded ticks.
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

Functional: **PASS**  
5 ms p99 deadline: **FAIL**  
Combined: **FAIL**

| Check | Result |
|---|---:|
| `no_infeasible_or_failed_ticks` | PASS |
| `no_contact_contingency_ticks` | PASS |
| `touchdown_transition_completes_within_8_ticks` | PASS |
| `touchdown_admission_completes_within_8_ticks` | PASS |
| `root_tracking_rms_le_5cm` | PASS |
| `stance_foot_tracking_rms_le_2cm` | PASS |
| `swing_foot_tracking_rms_le_8cm` | PASS |
| `root_rotation_le_5deg` | PASS |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `finite_support_margin_respected` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 160 | 0.800 s | 4.528 cm | 0.159 cm | 0.639 cm | 27.274 cm | 1.837° | 8.000 rad/s | 7918.8 µs |

Nominal hard residual maxima: dynamics `1.212e-09`, contact acceleration `3.026e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 4.528 cm |
| authored reference vs measured CoM RMS / p95 | 5.035 / 10.604 cm |
| stance foot RMS | 0.159 cm |
| swing foot RMS | 0.639 cm |
| hand RMS | 27.274 cm |
| maximum root rotation | 1.837° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 1.212e-09 |
| contact acceleration residual | 3.026e-11 |
| raw max dynamics residual, including rejected ticks | 1.212e-09 |
| raw max contact residual, including rejected ticks | 3.026e-11 |
| active normal force range | 0.000–311.480 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 1.392 m/s² |
| frame-angular acceleration RMS max | 3.666 rad/s² |
| longest pre-contact / touchdown transition | 40 / 6 ticks |
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
| 160 | 0.8 s | 2315.4 µs | 6680.3 µs | 7918.8 µs | 9027.5 µs | 53 | 61 | 40 | 6 | 0 | 0 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3208.1 | 1615.2 | 158.5 | 6443.2 | 8914.0 | 9016.2 | 2218.7 | 160 | 27 | 0 | 311.7 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 53 | 2300.1 | 2326.7 | 2338.9 | 2344.2 |
| solved_with_slack | 61 | 4204.8 | 7292.4 | 8599.0 | 9027.5 |
| touchdown_transition | 6 | 2408.6 | 3236.5 | 3344.6 | 3371.7 |
| precontact_transition | 40 | 2185.3 | 2552.8 | 2757.5 | 2852.3 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 7.25 | 13.0 | 17.4 | 19 | 4.27 | 15.4 | 17 | 0.3599 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 1.96/8.0/8.0/8 | 470.70/1936.0/1936.0/1936 | 0.28/2.0/2 | 0.14/1.0/1 | 1.22/9.0/9 | 0.8238 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 53 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 61 | 8.80/15.2/17 | 7.38/14.6/17 |
| touchdown_transition | 6 | 10.67/18.9/19 | 7.67/15.9/16 |
| precontact_transition | 40 | 8.68/13.8/15 | 4.67/10.6/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.37/6.0/6 | 5.14/24.0/24 | 0.49/5.0/6 | 34 |
| viability | 0.29/1.0/1 | 1.38/5.0/5 | 0.00/0.0/0 | 0 |
| intent | 1.80/6.0/9 | 3.61/12.0/18 | 1.26/6.0/9 | 74 |
| preference | 2.71/9.8/11 | 19.03/69.6/73 | 2.27/9.8/11 | 106 |
| style | 1.09/2.0/2 | 11.06/23.0/24 | 0.25/2.0/2 | 37 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 0 | 40 | 6 | 114 | 0 |
| right_ankle_roll_link | 0 | 0 | 0 | 160 | 0 |
| left_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 160 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `40` ticks, planned normal touchdown `6` ticks, normal fallback `0` ticks.
Precontact sole-center tangential speed: p50 `0.0563 m/s`, p95 `0.0751 m/s`, max `0.0754 m/s` over 40 samples.
Touchdown Normal sole-center tangential speed: p50 `0.0615 m/s`, p95 `0.0717 m/s`, max `0.0729 m/s` over 6 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.514 | 0.513 | 0.513 | 1.000 | 0.999 | 45.680 | 45.809 | 0.129 | 45.809 | 0.001 | 0 | 33 | 0 | 0 | 3 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–15 | 2300.8 | 2333.1 | 4.00 | 23.88 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 16–31 | 2296.4 | 2324.2 | 4.00 | 23.88 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 32–47 | 2298.3 | 2339.0 | 4.00 | 23.81 | 0.00 | 1.00 | 242.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| 48–63 | 2265.8 | 2371.3 | 7.12 | 42.81 | 2.62 | 1.00 | 231.00 | 0.000 | 0.176 | 1.21e-09 | 3.03e-11 | 11 |
| 64–79 | 2236.1 | 2815.9 | 9.12 | 58.69 | 5.25 | 1.00 | 226.00 | 0.000 | 0.635 | 7.90e-10 | 1.04e-11 | 16 |
| 80–95 | 1985.8 | 2761.8 | 8.62 | 41.81 | 5.19 | 1.00 | 227.88 | 0.197 | 0.278 | 4.50e-10 | 1.16e-11 | 13 |
| 96–111 | 2702.8 | 3711.5 | 9.44 | 46.75 | 7.50 | 1.00 | 240.88 | 2.204 | 0.139 | 4.59e-10 | 1.76e-11 | 0 |
| 112–127 | 4043.0 | 6146.6 | 8.94 | 50.56 | 7.44 | 3.62 | 877.25 | 5.325 | 0.065 | 3.73e-10 | 2.65e-11 | 0 |
| 128–143 | 6692.1 | 8920.4 | 8.00 | 41.38 | 6.62 | 8.00 | 1936.00 | 8.017 | 0.021 | 2.41e-12 | 1.25e-13 | 0 |
| 144–159 | 4247.5 | 6345.3 | 9.25 | 48.62 | 8.06 | 1.00 | 242.00 | 10.370 | 0.444 | 6.59e-10 | 2.29e-11 | 0 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 160 | 4.528 | 0.271 | 27.274 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
