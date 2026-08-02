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
- Balance task: `standalone-authored` `CoM` reference at `viability` priority with weight `0.000` and `2.000 Hz` response through `tracking` horizontal control (`full position/velocity/acceleration jet`). Foot tracking: `viability` priority at weight `1.000`. Hand task weight: `0.000` (hand error is observational when zero).
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
5 ms p99 deadline: **PASS**
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
| `p99_tick_le_5ms` | PASS |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 875 | 4.375 s | 2.099 cm | 0.355 cm | 0.110 cm | 34.930 cm | 1.218° | 8.000 rad/s | 5357.4 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `5.972e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 1482.074 cm |
| authored reference vs measured CoM RMS / p95 | 1478.360 / 3308.258 cm |
| stance foot RMS | 1415.438 cm |
| swing foot RMS | 1708.354 cm |
| hand RMS | 1486.916 cm |
| maximum root rotation | 19.968° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 3.055e-09 |
| contact acceleration residual | 5.972e-11 |
| raw max dynamics residual, including rejected ticks | 3.055e-09 |
| raw max contact residual, including rejected ticks | 5.972e-11 |
| active normal force range | 0.000–912.555 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 123.544 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 45 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

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

- Phase ticks: unsupported `1127`, single support `540`, precontact `0`, multi-support `650`.
- Joint-velocity envelope active on `16` ticks; maximum active coordinates `1`; mean target/applied scale `0.281` / `0.281`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 1723.8 µs | 3577.7 µs | 4715.7 µs | 85266.2 µs | 312 | 560 | 0 | 85 | 1,354 | 0 | 3 | 0 | 0 | 3 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1464.4 | 2290.3 | 1715.6 | 3207.7 | 6914.7 | 67268.9 | 2042.0 | 1,193 | 19 | 1 | 682.9 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 312 | 1858.5 | 2994.0 | 3049.3 | 3472.8 |
| solved_with_slack | 560 | 3092.3 | 4581.3 | 5918.9 | 7557.4 |
| normal_contact_contingency | 1,354 | 6.6 | 2846.5 | 3233.6 | 4485.3 |
| contact_release_contingency | 3 | 1395.5 | 1456.9 | 1462.3 | 1463.7 |
| touchdown_transition | 85 | 1945.8 | 3330.6 | 4175.5 | 4498.8 |
| normal_fallback_relock_probe_rejected | 3 | 3806.4 | 77120.3 | 83637.0 | 85266.2 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.72 | 11.0 | 15.0 | 23 | 2.57 | 13.8 | 21 | 0.5873 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.93/8.0/8.0/8 | 206.30/1728.0/1728.0/1776 | 0.12/2.0/5 | 0.06/1.0/4 | 0.51/9.0/32 | 0.3873 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 312 | 4.55/5.0/5 | 0.00/0.0/0 |
| solved_with_slack | 560 | 7.97/18.4/23 | 6.20/17.4/21 |
| normal_contact_contingency | 1,354 | 1.46/13.0/17 | 1.29/12.0/16 |
| contact_release_contingency | 3 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 85 | 8.46/13.3/15 | 8.16/13.3/15 |
| normal_fallback_relock_probe_rejected | 3 | 11.67/14.9/15 | 11.00/13.9/14 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.65/4.0/6 | 2.58/16.0/24 | 0.20/4.0/6 | 204 |
| viability | 0.44/6.0/12 | 2.13/30.0/65 | 0.33/6.0/12 | 319 |
| intent | 1.16/9.0/15 | 2.35/18.0/30 | 0.93/9.0/14 | 680 |
| preference | 0.94/8.8/19 | 7.99/71.7/157 | 0.78/8.0/18 | 876 |
| style | 0.53/2.0/3 | 7.43/31.0/49 | 0.33/1.0/3 | 740 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,393 | 0 | 48 | 643 | 233 |
| right_ankle_roll_link | 1,401 | 0 | 37 | 875 | 4 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `45` ticks, normal fallback `236` ticks.
Touchdown Normal sole-center tangential speed: p50 `4.8314 m/s`, p95 `8.6217 m/s`, max `9.3164 m/s` over 82 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 3.394 | 3.393 | 3.393 | 1.000 | 1.000 | 52.457 | 55.789 | 3.332 | 55.789 | 0.002 | 0 | 792 | 0 | 0 | 33 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2969.8 | 6561.9 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 2063.7 | 4534.7 | 6.72 | 37.97 | 3.28 | 1.57 | 352.78 | 0.665 | 0.334 | 1.29e-09 | 4.22e-11 | 0 |
| 464–695 | 3041.4 | 4870.1 | 6.98 | 41.75 | 4.09 | 1.00 | 230.56 | 0.221 | 0.286 | 1.11e-09 | 5.97e-11 | 0 |
| 696–927 | 2989.3 | 5703.3 | 8.38 | 47.35 | 7.21 | 1.45 | 327.75 | 11.013 | 6.991 | 1.27e-09 | 4.09e-11 | 53 |
| 928–1159 | 1940.5 | 4259.2 | 6.63 | 41.25 | 5.94 | 3.10 | 669.41 | 136.778 | 117.749 | 1.02e-09 | 6.79e-12 | 232 |
| 1160–1391 | 6.5 | 12.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 550.245 | 530.114 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.5 | 11.6 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 1167.451 | 1152.266 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.7 | 3716.2 | 1.67 | 10.01 | 1.65 | 0.92 | 199.17 | 1766.272 | 1760.530 | 3.05e-09 | 1.86e-11 | 186 |
| 1855–2085 | 6.5 | 11.8 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2488.893 | 2487.448 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 2357.6 | 1.37 | 8.55 | 1.33 | 0.22 | 47.69 | 3320.519 | 3315.571 | 1.47e-09 | 1.60e-11 | 194 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 1482.074 | 1477.950 | 1486.916 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
