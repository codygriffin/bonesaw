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
| `swing_foot_tracking_rms_le_8cm` | FAIL |
| `root_rotation_le_5deg` | FAIL |
| `joint_velocity_le_8rad_s` | PASS |
| `dynamics_residual_le_1e_8` | PASS |
| `contact_residual_le_1e_8` | PASS |
| `p99_tick_le_5ms` | FAIL |

Failed checks: `no_contact_contingency_ticks`, `touchdown_transition_completes_within_8_ticks`, `root_tracking_rms_le_5cm`, `stance_foot_tracking_rms_le_2cm`, `swing_foot_tracking_rms_le_8cm`, `root_rotation_le_5deg`, `p99_tick_le_5ms`.

## Nominal prefix before first contingency

This window ends immediately before the first normal-only, contact-release, infeasible, or numerical-failure status.

| ticks | duration | root RMS | stance foot RMS | swing foot RMS | hand RMS | max root rotation | max joint speed | p99 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 309 | 1.545 s | 0.669 cm | 0.404 cm | 0.344 cm | 26.744 cm | 0.055° | 8.000 rad/s | 6212.3 µs |

Nominal hard residual maxima: dynamics `1.685e-09`, contact acceleration `4.840e-11`.

## Tracking and physical residuals

| Metric | Result |
|---|---:|
| root RMS | 2714.708 cm |
| authored reference vs measured CoM RMS / p95 | 2707.370 / 4970.985 cm |
| stance foot RMS | 2574.369 cm |
| swing foot RMS | 2905.202 cm |
| hand RMS | 2751.803 cm |
| maximum root rotation | 136.698° |
| maximum joint velocity | 8.000 rad/s |
| dynamics residual | 7.980e-09 |
| contact acceleration residual | 1.530e-10 |
| raw max dynamics residual, including rejected ticks | 7.980e-09 |
| raw max contact residual, including rejected ticks | 1.530e-10 |
| active normal force range | 0.000–764.764 N |
| centroidal momentum-rate residual RMS / max | 0.000 / 0.000 N·m |
| point-task acceleration RMS max | 119.956 m/s² |
| frame-angular acceleration RMS max | 0.000 rad/s² |
| longest pre-contact / touchdown transition | 0 / 62 ticks |
| delayed touchdown admission ticks / longest delay | 0 / 0 ticks |

## Support-transfer trajectory tube

- Hard tube active on `936` ticks. Optional intent projection is `True` and changed the authored CoM/root request on `936` ticks.
- Measured joint-headroom slew scale minimum / p05: `1.000` / `1.000`; projected target displacement RMS is `5.312 cm`.
- Hard acceleration-tube minimum margin: `0.8186525348695257` m/s²; violations beyond tolerance: `0` ticks; limiting face counts (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 0, '3': 14}`.
- First-hard-solve witness margin minimum across all active attempts: `-15.17970790569261` m/s²; witness violations: `1` ticks; limiting witness faces (+x/-x/+y/-y): `{'0': 0, '1': 0, '2': 0, '3': 17}`.
- Hard rows returned an admitted solve on `14` active ticks; `922` active requests remained unresolved and are not misreported as boundary violations.
- Four allocation-free hard WBC rows bound realized CoM acceleration. The separately switchable preview projector may shape intent; neither layer admits contact or grants actuator authority, and failed solves remain visible.

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

- Phase ticks: unsupported `1883`, single support `134`, precontact `0`, multi-support `300`.
- Joint-velocity envelope active on `0` ticks; maximum active coordinates `0`; mean target/applied scale `0.129` / `0.129`.
- Joint-position capture active on `0` ticks; maximum active coordinates `0`.

## Pre-contingency hard-feasibility witness

The witness is sampled immediately after the first full contact solve and before any retry or release contingency. It is diagnostic only: it never changes the executable state.
- First-solve statuses solved/slack/max-iterations/primal-infeasible/numerical/invalid/no-contact: `137` / `296` / `7` / `0` / `0` / `0` / `1877`.
- Bound-limited ticks / upper-bound ticks: `4` / `0`; maximum coordinate violation `34.690497390330634`.
- Named-linear-row-limited ticks / upper-row ticks: `7` / `4`; maximum row violation `1546.347244454654`.

## Runtime

| ticks | duration | p50 | p95 | p99 | max | solved | slack | pre-contact | touchdown | normal fallback | relock admitted | relock rejected | solve hold | localized handoff | release fallback | infeasible | failed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 2,317 | 11.6 s | 6.5 µs | 3221.7 µs | 5113.7 µs | 165629.8 µs | 137 | 172 | 0 | 124 | 1,878 | 0 | 0 | 0 | 0 | 6 | 0 | 0 |

### Latency distribution and deadlines

The per-tick timer is inside the Rust batch loop. Call-level wall/CPU measurements wrap the single PyO3 call and therefore include only one Python→Rust boundary crossing, not per-tick Python work.

| mean µs | std µs | MAD µs | p90 µs | p99.9 µs | p99.99 µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 752.6 | 4543.6 | 0.9 | 2939.9 | 12339.1 | 156743.8 | 2155.9 | 439 | 26 | 2 | 1328.8 |

### Latency by solver/contact status

| status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---:|---:|---:|---:|---:|
| solved | 137 | 2886.9 | 2982.0 | 3010.6 | 3025.6 |
| solved_with_slack | 172 | 3215.8 | 5112.7 | 6792.6 | 6908.4 |
| normal_contact_contingency | 1,878 | 6.4 | 9.6 | 13.9 | 3599.7 |
| contact_release_contingency | 6 | 2238.1 | 156037.9 | 163711.4 | 165629.8 |
| touchdown_transition | 124 | 2797.0 | 9554.0 | 11169.7 | 12811.7 |

### Strict-solver work attribution

A task pseudoinverse is the dominant dense kernel. Each active priority normally needs one call after exact projected-inverse reuse; accepted bound/inequality truncations require another projected solve.

| pseudoinverse mean | pseudoinverse p95 | pseudoinverse p99 | pseudoinverse max | clipped-step mean | clipped-step p99 | clipped-step max | calls↔latency correlation |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.32 | 9.0 | 12.0 | 20 | 0.94 | 12.0 | 19 | 0.2624 |

The feasibility seed is a cyclic hard-halfspace projection before semantic task solving. Near-feasible seeds may then enter a dense active-set polish; these counters make that previously hidden work visible without timing inside the solver.

| projection sweeps mean/p95/p99/max | halfspace projections mean/p95/p99/max | polish iterations mean/p99/max | polish pseudoinverses mean/p99/max | polish Jacobi sweeps mean/p99/max | projections↔latency correlation |
|---:|---:|---:|---:|---:|---:|
| 0.47/1.0/8.0/8 | 104.94/234.0/1728.0/1808 | 0.15/3.0/14 | 0.11/2.0/13 | 0.88/16.0/108 | 0.1973 |

| status | ticks | pseudoinverse mean/p99/max | clipped-step mean/p99/max |
|---|---:|---:|---:|
| solved | 137 | 4.00/4.0/4 | 0.00/0.0/0 |
| solved_with_slack | 172 | 8.06/16.6/18 | 6.29/15.3/16 |
| normal_contact_contingency | 1,878 | 0.00/0.0/8 | 0.00/0.0/6 |
| contact_release_contingency | 6 | 0.00/0.0/0 | 0.00/0.0/0 |
| touchdown_transition | 124 | 9.07/14.8/20 | 8.80/14.0/19 |

| priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---:|---:|---:|---:|
| invariant | 0.30/4.0/9 | 1.24/16.0/36 | 0.16/4.0/9 | 128 |
| viability | 0.10/2.0/12 | 0.51/10.8/65 | 0.09/2.0/12 | 125 |
| intent | 0.41/6.8/13 | 0.82/13.7/26 | 0.31/6.0/13 | 221 |
| preference | 0.32/4.0/14 | 2.74/35.0/112 | 0.26/4.0/14 | 295 |
| style | 0.20/1.0/6 | 2.87/18.0/64 | 0.11/1.0/6 | 244 |

### Per-target support-phase telemetry

These are Rust-owned phase states sampled after each tick; solver contingencies remain separate from planned touchdown.

| target | swing | precontact | touchdown normal | locked | normal fallback |
|---|---:|---:|---:|---:|---:|
| left_ankle_roll_link | 1,931 | 0 | 86 | 300 | 0 |
| right_ankle_roll_link | 1,969 | 0 | 38 | 309 | 1 |
| left_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |
| right_wrist_roll_rubber_hand | 2,317 | 0 | 0 | 0 | 0 |

Maximum contiguous phase ages: precontact `0` ticks, planned normal touchdown `62` ticks, normal fallback `4` ticks.
Touchdown Normal sole-center tangential speed: p50 `2.7180 m/s`, p95 `4.1115 m/s`, max `4.4453 m/s` over 120 samples.

### CPU, memory, faults, context switches, and Python runtime

All large NumPy input/output buffers and the Rust session are constructed before this measurement. Python `tracemalloc` does not observe native Rust allocations; the standalone native controller sentinels remain the authoritative hot-loop allocation gate.

| call wall s | process CPU s | thread CPU s | process CPU/wall | thread CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor faults | major faults | voluntary ctx | involuntary ctx |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1.744 | 1.741 | 1.741 | 0.998 | 0.998 | 53.430 | 56.980 | 3.551 | 56.980 | 0.002 | 0 | 817 | 0 | 0 | 132 |

### Execution over time

Each row is one tenth of the run so warm drift, scheduler tails, tracking loss, and the exact onset of contact contingency remain visible.

| ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | feasibility sweeps mean | halfspace projections mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0–231 | 2946.4 | 6560.3 | 5.41 | 37.75 | 2.17 | 1.00 | 234.00 | 0.345 | 0.404 | 1.69e-09 | 4.84e-11 | 0 |
| 232–463 | 7.4 | 4794.8 | 2.97 | 14.64 | 2.52 | 0.46 | 105.48 | 103.349 | 104.876 | 9.69e-10 | 4.22e-11 | 155 |
| 464–695 | 7.2 | 10442.1 | 2.34 | 14.22 | 2.28 | 1.14 | 246.72 | 464.312 | 432.591 | 6.53e-09 | 1.09e-10 | 170 |
| 696–927 | 6.4 | 12.2 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 972.645 | 907.088 | 0.00e+00 | 0.00e+00 | 232 |
| 928–1159 | 6.3 | 3804.8 | 0.82 | 4.90 | 0.78 | 0.72 | 156.41 | 1616.474 | 1545.368 | 7.98e-09 | 8.25e-11 | 211 |
| 1160–1391 | 6.6 | 15.4 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2273.642 | 2188.025 | 0.00e+00 | 0.00e+00 | 232 |
| 1392–1623 | 6.3 | 14.9 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 2930.246 | 2852.363 | 0.00e+00 | 0.00e+00 | 232 |
| 1624–1854 | 6.6 | 4698.2 | 0.92 | 5.34 | 0.90 | 0.83 | 179.53 | 3602.166 | 3507.862 | 4.52e-09 | 8.61e-11 | 207 |
| 1855–2085 | 6.5 | 13.0 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 4295.880 | 4189.694 | 0.00e+00 | 0.00e+00 | 231 |
| 2086–2316 | 5.8 | 4054.9 | 0.77 | 4.84 | 0.74 | 0.59 | 127.17 | 4987.237 | 4879.240 | 3.04e-09 | 1.53e-10 | 214 |

## Cadence sensitivity

| cadence | ticks | root RMS cm | foot RMS cm | hand RMS cm |
|---|---:|---:|---:|---:|
| 1.00x | 2,317 | 2714.708 | 2643.051 | 2751.803 |

Raw arrays are retained in `floating-walk-raw.npz`; exact metrics and source/retarget metadata are in `floating-walk-metrics.json`.
