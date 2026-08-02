# Bonesaw floating G1 admitted-reference tracking

This is a moving-root, contact-aware trace through the Rust floating inverse-dynamics WBC. Python owns artifact transport, fixed-shape arrays, and metrics; the standalone path forbids source reconstruction. Rust owns measured-touchdown anchors, target shaping, Jacobians, hard contact rows, the solve, SE(3) integration, and every measured tick.

## Source and motion profile

- Source: immutable open-loop-admitted artifact `benchmarks/results/g1-bonesaw-lipm-r42/reference-inputs.npz` from `Bonesaw two-stage support-constrained LIPM`.
- Admission contract: authored root, CoM, foot jets, and contact schedule are consumed unchanged; eval-side reconstruction, projection, and retiming are rejected.
- Target model: `benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf` with 4 sole contact points per foot (`x=0.035±0.085 m`, `y=±0.0275 m`, `z=-0.035 m` in the foot frame).
- Applied target displacement: `0.345 m` forward per `3.000 s` source cycle.
- Applied mean forward speed: `0.115 m/s` (`1.00×` forward, `1.00×` lateral retarget scale).
- Cadence schedule: `immutable authored tick sequence` in `3.0 s` blocks with an applied `1.00×` multiplier.
- Each four-point sole retains four independent friction-limited force slots but emits a rank-minimal six-row rigid-foot lock. Touchdown begins in a planned normal-only transition before tangential lock; fallback and contact release remain separately observable.
- Balance task: `rooted` `CoM` reference at `viability` priority with weight `1.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
- Root horizontal reference: `mocap`.
- Root / swing-point response: `2.000` / `4.000 Hz` critically damped; root angular/height/horizontal task weights `1.000` / `10.000` / `1.000`.
- Whole-body posture: `preference` priority with weight `0.250`.
- Protected upper-body posture: `intent` priority with weight `0.000` over 11 waist/arm coordinates.
- Joint-velocity envelope: `viability` priority with weight `0.000`, activating at `75.0%` of each effective limit with `2.000 Hz` response and `always` measured-phase policy with immediate engagement and bounded release over `0` ticks.
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
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_infeasible_or_failed_ticks`, `no_contact_contingency_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 193 | 0.965 s | 13.746 cm | 0.180 cm | nan cm | 41.564 cm | 81.738° | 8.000 rad/s | 52961.2 µs |

Nominal hard residual maxima: dynamics `2.566e-09`, contact acceleration `1.539e-10`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 197.321 cm |
| authored reference vs measured CoM RMS / p95 | 191.107 / 268.446 cm |
| stance foot RMS | 163.945 cm |
| swing foot RMS | 169.088 cm |
| hand RMS | 196.855 cm |
| maximum root rotation | 179.640° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.160e-09 |
| contact acceleration residual | 2.858e-10 |
| raw max dynamics residual, including rejected ticks | 4.648e+02 |
| raw max contact residual, including rejected ticks | 2.858e-10 |
| active normal force range | 0.000–494.111 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 177.874 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 0 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Coupled touchdown phase retiming

- Enabled: `False` (touchdown `False`, balance `False`); final source tick `599.000`, progress `599.000` ticks.
- First post-liftoff authored touchdown source tick: `428`; reached before trace end: `True`.
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

- Phase ticks: unsupported `1`, single support `229`, precontact `0`, multi-support `370`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `1.000` / `1.000`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 600 | 3.0 s | 166168.9 µs | 205297.9 µs | 217399.1 µs | 318888.3 µs | 3 | 190 | 0 | 0 | 74 | 74 | 259 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 108971.0 | 92530.2 | 40336.4 | 204303.0 | 301351.8 | 317134.7 | 127931.4 | 600 | 449 | 351 | 9.2 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 3 | 2339.5 | 2413.0 | 2419.6 | 2421.2 |
| solved_with_slack | 190 | 3314.8 | 22277.0 | 53446.4 | 82267.4 |
| primal_infeasible | 259 | 203184.5 | 206296.0 | 219050.9 | 318888.3 |
| normal_contact_contingency | 74 | 6514.8 | 27966.5 | 126711.8 | 289612.0 |
| contact_release_contingency | 74 | 169459.4 | 211854.9 | 218221.3 | 220487.8 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 5.52 | 14.0 | 19.0 | 22 | 4.73 | 19.0 | 21 | -0.7866 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 2922.23/6720.0/6720.0/6720 | 613814.86/1411200.0/1411200.0/1411200 | 5.27/22.0/36 | 4.46/21.0/35 | 34.74/189.0/328 | 0.8153 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 3 | 5.00/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 190 | 10.36/21.0/22 | 8.61/20.1/21 |
| primal_infeasible | 259 | 0.00/0.0/0 | 0.00/0.0/0 |
| normal_contact_contingency | 74 | 9.82/13.5/15 | 9.24/12.5/14 |
| contact_release_contingency | 74 | 8.15/12.0/12 | 6.97/11.0/11 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 1.45/6.0/8 | 6.52/28.0/34 | 1.32/6.0/8 | 263 |
| viability | 1.36/8.0/16 | 6.26/32.0/62 | 1.34/8.0/16 | 333 |
| intent | 0.93/5.0/10 | 1.88/12.0/20 | 0.86/5.0/10 | 307 |
| preference | 0.97/9.0/13 | 9.26/86.1/126 | 0.77/9.0/13 | 239 |
| style | 0.81/6.0/15 | 7.30/74.2/164 | 0.44/6.0/15 | 148 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 229 | 0 | 172 | 193 | 6 |
| right_ankle_roll_link | 0 | 0 | 0 | 193 | 407 |
| left_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 600 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `172` ticks, normal fallback `410` ticks.
Touchdown Normal sole-center tangential speed: p50 `1.6019 m/s`, p95 `1.6019 m/s`, max `1.6019 m/s` over 171 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 65.383 | 65.353 | 65.353 | 1.000 | 1.000 | 45.336 | 46.078 | 0.742 | 46.078 | 0.001 | 0 | 174 | 0 | 0 | 2,152 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–59 | 2537.7 | 4340.6 | 9.42 | 47.63 | 6.13 | 1.00 | 234.00 | 0.434 | 0.001 | 1.60e-09 | 8.54e-11 | 0 |
| 60–119 | 3314.8 | 6535.4 | 9.97 | 51.18 | 8.43 | 4.03 | 943.80 | 2.326 | 0.271 | 1.07e-09 | 4.49e-11 | 0 |
| 120–179 | 5420.9 | 16738.2 | 10.25 | 65.58 | 9.28 | 6.37 | 1489.80 | 18.960 | 0.167 | 2.45e-09 | 7.10e-11 | 0 |
| 180–239 | 9287.2 | 228107.0 | 11.22 | 74.32 | 10.73 | 43.38 | 9404.40 | 56.818 | 12.347 | 5.17e-09 | 1.84e-10 | 47 |
| 240–299 | 83835.7 | 207697.9 | 8.82 | 46.47 | 7.95 | 154.27 | 33297.60 | 151.889 | 93.447 | 7.16e-09 | 2.86e-10 | 60 |
| 300–359 | 176915.1 | 218656.0 | 5.57 | 26.92 | 4.73 | 2133.23 | 447979.00 | 246.296 | 205.184 | 4.65e+02 | 0.00e+00 | 60 |
| 360–419 | 177003.5 | 180229.1 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 274.787 | 235.369 | 4.65e+02 | 0.00e+00 | 60 |
| 420–479 | 203640.4 | 209405.3 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 274.787 | 234.993 | 4.65e+02 | 0.00e+00 | 60 |
| 480–539 | 203973.3 | 264089.2 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 274.787 | 234.993 | 4.65e+02 | 0.00e+00 | 60 |
| 540–599 | 203464.9 | 208153.5 | 0.00 | 0.00 | 0.00 | 6720.00 | 1411200.00 | 274.787 | 234.993 | 4.65e+02 | 0.00e+00 | 60 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 600 | 197.321 | 164.939 | 196.855 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
