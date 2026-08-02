# Bonesaw reference implementation comparison

This report compares the CPU kinematic WBC path against PlaCo, keeps
Pinocchio as the rigid-body correctness oracle, and executes Upkie's
pinned C++ WheelBalancer as a direct rolling-controller oracle. Raw
per-step traces are stored beside this report in compressed NumPy archives.
The official Unitree G1 section adds the current floating inverse-dynamics
liftoff proof and deliberately red full-transfer stress, including execution
windows and process-resource measurements.

## Method and comparability

- Same `models/toy_humanoid.urdf`, fixed base, 18 coordinates, 50 Hz, 5,000 measured ticks per scenario.
- 250 unreported warmup ticks precede every measured run.
- Targets, initial posture, sample times, joint/velocity limits, and tracked frame names are identical.
- Bonesaw uses strict lexicographic priorities and a bandwidth-shaped velocity target plus the finite-difference velocity jet of the shared position reference.
- PlaCo uses its independent C++ QP kinematics solver with soft task weights, a 1e-3 near-null posture regularizer, and a weight-10 CoM task in the conflict case.
- Walking targets come from the pinned CMU Graphics Lab subject-37/trial-1 slow walk, not a procedural sine wave. A periodic six-harmonic reconstruction is morphology-scaled and time-warped through 0.75×, 1.0×, and 1.25× cadence blocks.
- PlaCo's public position task accepts the shared position reference but exposes no target-velocity field. Therefore timing and closed-loop tracking are measurable, while exact command equality and identical feedforward are not expected: the optimization semantics and task interfaces are deliberately different.
- Bonesaw `latency` is timed inside the Rust batch loop. PlaCo `latency` includes Python target assignment, kinematic updates, the C++ solve, and frame extraction; its separate solver-only distribution is also reported.
- Python `tracemalloc` excludes native allocations. RSS/USS and `ru_maxrss` include native memory.

PlaCo describes itself as a C++ whole-body inverse-kinematics/dynamics QP implementation built on Pinocchio and eiquadprog; its documented loop updates kinematics, solves, integrates, and updates kinematics again.

## Reference scope matrix

These references answer different questions; rows that do not share a model,
decision vector, or task API are not presented as command-for-command parity.

| Reference | Shared boundary | What it proves | Deliberate limitation |
|---|---|---|---|
| PlaCo | Same toy-humanoid model, targets, initial state, frames, and 50 Hz scenario corpus | Independent WBC tracking, latency, jitter, and process-resource comparison | Soft weighted QP and public position-only task API differ from Bonesaw's strict hierarchy and target jets |
| Pinocchio 4.0 | Same URDF states and rigid-body convention adapters | FK, Jacobian, CoM, mass, bias, inverse dynamics, and centroidal product correctness | Product oracle, not a closed-loop controller |
| Upkie C++ WheelBalancer | Same sequential balance inputs and official parameter profile | Canonical bitwise controller-law parity plus live-tuning delta | Compares the typed rolling law, not the whole inverse-dynamics WBC |
| Official Unitree G1 + CMU 37/01 | Authored G1 mass/inertia/limits/sole geometry and a pinned walking source | Floating contact behavior, strict residuals, transitions, and scaling cost on a realistic 23-DOF morphology | Behavior/evaluation reference; no public G1 WBC exposes identical strict task semantics for direct command parity |

## Environment

| Field | Value |
|---|---|
| CPU | AMD Ryzen 7 3700X 8-Core Processor |
| OS | Linux-6.18.7-76061807-generic-x86_64-with-glibc2.39 |
| Python | 3.12.3 |
| Bonesaw | 0.1.0 |
| PlaCo | 0.9.23 |
| PlaCo Pinocchio | 3.8.0 |
| Logical CPUs | 16 |
| Physical CPUs | 8 |

## Aggregate outcome

| Scenario | Implementation | RMS error cm | steady RMS cm | p50 µs | p99 µs | p99.9 µs | max µs | >20 ms | steps/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0.630 | 0.129 | 40.2 | 49.9 | 61.1 | 76.0 | 0 | 24260 |
| end_effector_reach | PlaCo | 0.809 | 0.737 | 94.1 | 113.4 | 125.2 | 236.7 | 0 | 9822 |
| bimanual_priority_conflict | Bonesaw | 0.475 | 0.000 | 90.1 | 124.0 | 136.1 | 142.4 | 0 | 10962 |
| bimanual_priority_conflict | PlaCo | 0.363 | 0.250 | 106.8 | 127.3 | 136.5 | 237.6 | 0 | 8692 |
| walking_motion_retarget | Bonesaw | 4.048 | 4.062 | 67.1 | 76.4 | 89.3 | 247.6 | 0 | 14893 |
| walking_motion_retarget | PlaCo | 2.634 | 2.645 | 129.6 | 155.2 | 162.0 | 316.6 | 0 | 7268 |

## Official G1 floating inverse-dynamics WBC

Both profiles use the checksum-pinned Unitree 23-DOF mode-10 URDF, four
friction-limited force points per sole, six rank-minimal kinematic rows per
locked foot, and the same CMU 37/01 retarget. The short profile proves a
moving liftoff without fallback. The longer profile is retained precisely
because it exposes the current single-support/contact-transfer failure.
Hand error is observational because the canonical hand task weight is zero.

### Behavior, feasibility, and tracking

| profile | behavior gate | overall gate | ticks | nominal prefix | root RMS cm | stance RMS cm | swing RMS cm | max root deg | max joint rad/s | solved/slack/touchdown | fallback/release/infeasible/failed | dynamics max | contact max |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | PASS | FAIL | 260 | 260 | 0.744 | 0.000 | 0.371 | 2.914 | 8.000 | 196/61/3 | 0/0/0/0 | 1.22e-09 | 5.15e-11 |
| full transfer stress | FAIL | FAIL | 600 | 330 | 156.891 | 106.353 | 143.447 | 144.209 | 8.000 | 196/131/4 | 27/111/131/0 | 1.22e-09 | 5.32e-11 |

The moving-liftoff row is behaviorally green but fails the unchanged 5 ms
p99 CPU gate. The transfer row fails behavior and timing. Rejected ticks
retain raw residuals and state rather than being omitted from aggregates.

### G1 latency, jitter, and deadlines

| profile | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 2933.7 | 2253.8 | 10.7 | 2381.0 | 9881.5 | 13038.9 | 14663.9 | 14910.9 | 14938.3 | 3783.0 | 260 | 19 | 0 | 340.9 |
| full transfer stress | 81269.5 | 91184.0 | 9135.6 | 11130.3 | 221212.9 | 225065.7 | 281104.2 | 284159.8 | 284499.4 | 129059.4 | 600 | 333 | 272 | 12.3 |

### G1 process resources

Buffers and the Rust session are allocated before this measurement. Python
`tracemalloc` excludes native allocation; zero hot-loop allocation is gated
separately by the native Rust sentinels later in this report.

| profile | wall s | process CPU s | thread CPU s | process CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor/major faults | voluntary/involuntary ctx |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0.763 | 0.763 | 0.763 | 1.000 | 46.41 | 46.41 | 0.00 | 46.41 | 0.00 | 0 | 0/0 | 0/10 |
| full transfer stress | 48.762 | 48.748 | 48.748 | 1.000 | 46.11 | 46.20 | 0.08 | 46.37 | 0.00 | 0 | 21/0 | 0/735 |

### G1 latency by solver/contact status

| profile | status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---|---:|---:|---:|---:|---:|
| moving liftoff | solved | 196 | 2381.2 | 2411.4 | 2473.7 | 2480.4 |
| moving liftoff | solved_with_slack | 61 | 2282.5 | 12792.7 | 14302.5 | 14938.3 |
| moving liftoff | touchdown_transition | 3 | 2233.5 | 2234.4 | 2234.5 | 2234.5 |
| full transfer stress | solved | 196 | 2382.8 | 2407.6 | 2428.2 | 2494.6 |
| full transfer stress | solved_with_slack | 131 | 5749.7 | 116180.3 | 136170.7 | 137605.0 |
| full transfer stress | primal_infeasible | 131 | 220178.5 | 223448.3 | 244098.6 | 284499.4 |
| full transfer stress | normal_contact_contingency | 27 | 72901.2 | 129575.9 | 130261.9 | 130267.8 |
| full transfer stress | contact_release_contingency | 111 | 133502.3 | 174667.3 | 252775.9 | 278831.3 |
| full transfer stress | touchdown_transition | 4 | 2250.9 | 5314.6 | 5745.7 | 5853.5 |

### G1 strict-solver work attribution

Task-level pseudoinverse and Jacobi-sweep counts expose the dense-kernel
work behind each tick. The solver reuses the feasibility seed's exact
equality factorization, reuses a projected inverse while its nullspace is
unchanged, and skips the terminal Style projector because no lower priority
can consume it. Near-feasible seeds still polish and refactor. These are
algebraically exact changes: both G1 behavior/status traces are unchanged.

| profile | pseudoinverse mean/p95/p99/max | Jacobi sweeps mean/p95/p99/max | sweeps/pseudoinverse | clipped-step mean/p95/p99/max | calls↔latency correlation |
|---|---:|---:|---:|---:|---:|
| moving liftoff | 4.98/9.0/11.4/12 | 28.08/48.0/64.4/69 | 5.63 | 1.19/6.0/9.0/10 | 0.4672 |
| full transfer stress | 5.08/10.0/12.0/14 | 26.43/53.0/61.0/79 | 5.20 | 3.07/10.0/11.0/13 | -0.4705 |

| profile | priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---|---:|---:|---:|---:|
| moving liftoff | invariant | 1.11/3.0/3 | 3.67/12.0/12 | 0.15/3.0/3 | 19 |
| moving liftoff | viability | 0.26/1.4/3 | 0.94/5.6/12 | 0.06/1.4/3 | 11 |
| moving liftoff | intent | 1.17/4.0/5 | 2.34/8.0/10 | 0.28/4.0/5 | 32 |
| moving liftoff | preference | 1.45/6.4/7 | 10.50/48.4/56 | 0.68/6.0/7 | 61 |
| moving liftoff | style | 1.00/1.0/1 | 10.63/12.0/12 | 0.02/1.0/1 | 4 |
| full transfer stress | invariant | 1.46/5.0/7 | 5.96/25.0/35 | 1.04/5.0/7 | 231 |
| full transfer stress | viability | 0.73/5.0/7 | 3.75/24.0/32 | 0.61/5.0/7 | 213 |
| full transfer stress | intent | 1.02/4.0/7 | 2.06/8.0/14 | 0.63/4.0/7 | 240 |
| full transfer stress | preference | 1.07/5.0/8 | 8.55/40.0/58 | 0.70/5.0/7 | 247 |
| full transfer stress | style | 0.80/2.0/4 | 6.11/12.0/26 | 0.09/2.0/3 | 42 |

### G1 execution over time

| profile | ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0–25 | 2380.2 | 2395.4 | 4.12 | 24.54 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 26–51 | 2380.0 | 2389.5 | 4.00 | 23.92 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 52–77 | 2378.0 | 2394.8 | 4.00 | 23.81 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 78–103 | 2382.3 | 2395.8 | 4.00 | 23.81 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 104–129 | 2386.5 | 2403.5 | 4.00 | 23.92 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 130–155 | 2378.0 | 2398.3 | 4.00 | 23.77 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 156–181 | 2382.8 | 2479.5 | 4.00 | 23.69 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 182–207 | 2330.0 | 2678.4 | 5.50 | 32.08 | 1.50 | 0.007 | 0.001 | 8.52e-10 | 5.15e-11 | 0 |
| moving liftoff | 208–233 | 2131.9 | 2665.6 | 7.96 | 44.27 | 4.19 | 0.033 | 0.010 | 6.48e-10 | 1.48e-11 | 0 |
| moving liftoff | 234–259 | 10018.2 | 14673.4 | 8.27 | 36.96 | 6.19 | 2.353 | 0.402 | 5.47e-10 | 3.19e-12 | 0 |
| full transfer stress | 0–59 | 2384.1 | 2454.0 | 4.05 | 24.17 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| full transfer stress | 60–119 | 2384.0 | 2404.0 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| full transfer stress | 120–179 | 2381.3 | 2435.9 | 4.00 | 23.78 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| full transfer stress | 180–239 | 2177.0 | 8706.6 | 6.45 | 34.60 | 2.75 | 0.096 | 0.012 | 1.03e-09 | 5.32e-11 | 0 |
| full transfer stress | 240–299 | 10654.9 | 17029.1 | 8.90 | 41.53 | 7.65 | 10.522 | 2.113 | 6.08e-13 | 6.61e-14 | 0 |
| full transfer stress | 300–359 | 133018.5 | 195507.8 | 9.10 | 44.02 | 7.97 | 55.370 | 7.825 | 3.56e-10 | 6.46e-12 | 30 |
| full transfer stress | 360–419 | 131125.8 | 134756.8 | 7.98 | 39.73 | 6.93 | 146.514 | 73.232 | 1.10e-09 | 1.18e-11 | 60 |
| full transfer stress | 420–479 | 138777.8 | 237094.5 | 6.33 | 32.63 | 5.43 | 247.715 | 193.628 | 8.48e+02 | 3.55e-14 | 59 |
| full transfer stress | 480–539 | 219848.9 | 225066.2 | 0.00 | 0.00 | 0.00 | 283.474 | 225.088 | 8.48e+02 | 0.00e+00 | 60 |
| full transfer stress | 540–599 | 221030.2 | 265464.6 | 0.00 | 0.00 | 0.00 | 282.462 | 224.008 | 8.48e+02 | 0.00e+00 | 60 |

### G1 artifact integrity

| profile | metrics SHA-256 | raw NPZ SHA-256 | directory |
|---|---|---|---|
| moving liftoff | `3747718d0e59e27e5ca540b74569a031e6f1d926032d8afdb0707409a0c4f17a` | `8edfc8d16685d38d0eac25d65c1a3352bfd2035201545f284b5f3273a0c1f5a0` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-liftoff-latest` |
| full transfer stress | `6d9f9cbf4e0d3e6762cead0f278782efa22460c9695904241314cd5db2a0f2d8` | `a465918e1852035b0ea867a48990fdcd0df988a750cdf99259d56ddbdb3f7084` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-transfer-latest` |

## Pinocchio 4.0 rigid-body differential oracles

The Rust fixture and Pinocchio run in separate processes. Every row compares
fixed and floating products at identical deterministic states after explicit
conversion between Bonesaw's `[angular; linear]` world-classical root tangent
and Pinocchio's free-flyer convention.

| model | gate | states | frame samples | frame position | frame rotation | Jacobian | CoM | mass | gravity | inverse dynamics | centroidal map | floating mass | floating bias | floating inverse dynamics |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| upkie | PASS | 50 | 2,050 | 2.22e-16 | 7.77e-16 | 8.98e-16 | 1.11e-16 | 9.71e-17 | 1.11e-15 | 1.11e-15 | 1.67e-16 | 8.88e-16 | 2.13e-14 | 2.13e-14 |
| g1_23dof_mode_10 | PASS | 50 | 1,500 | 3.99e-16 | 9.99e-16 | 9.99e-16 | 6.79e-17 | 6.66e-16 | 1.07e-14 | 1.07e-14 | 1.11e-15 | 7.11e-15 | 1.71e-13 | 1.71e-13 |

## Latency and jitter distributions

| Scenario | Impl | mean µs | std µs | MAD µs | p90 µs | p95 µs | p99 µs | p99.99 µs | jitter p99 µs |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 40.6 | 2.2 | 0.5 | 41.6 | 44.4 | 49.9 | 74.6 | 5.9 |
| end_effector_reach | PlaCo | 95.5 | 5.1 | 1.6 | 101.2 | 105.1 | 113.4 | 187.9 | 17.5 |
| bimanual_priority_conflict | Bonesaw | 90.6 | 8.1 | 3.1 | 96.7 | 99.1 | 124.0 | 141.1 | 14.2 |
| bimanual_priority_conflict | PlaCo | 108.6 | 5.7 | 1.6 | 115.6 | 121.3 | 127.3 | 193.2 | 19.0 |
| walking_motion_retarget | Bonesaw | 66.5 | 5.0 | 3.0 | 71.6 | 72.5 | 76.4 | 171.4 | 11.6 |
| walking_motion_retarget | PlaCo | 131.2 | 7.1 | 2.8 | 138.6 | 144.4 | 155.2 | 281.0 | 25.1 |

## Tracking quality

| Scenario | Impl | median cm | p95 cm | p99 cm | max cm | IAE m·s | ISE m²·s | >1 cm | >3 cm | >5 cm |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0.129 | 0.133 | 0.133 | 24.569 | 0.304 | 0.0079 | 0.2% | 0.2% | 0.1% |
| end_effector_reach | PlaCo | 0.737 | 0.797 | 0.798 | 14.460 | 1.494 | 0.0131 | 0.2% | 0.2% | 0.1% |
| bimanual_priority_conflict | Bonesaw | 0.000 | 0.000 | 0.000 | 17.858 | 0.043 | 0.0045 | 0.3% | 0.2% | 0.2% |
| bimanual_priority_conflict | PlaCo | 0.253 | 0.253 | 0.253 | 16.967 | 0.511 | 0.0026 | 0.1% | 0.0% | 0.0% |
| walking_motion_retarget | Bonesaw | 1.790 | 9.498 | 14.537 | 15.573 | 11.001 | 0.6555 | 73.7% | 25.2% | 14.5% |
| walking_motion_retarget | PlaCo | 0.423 | 6.222 | 11.387 | 11.595 | 5.255 | 0.2776 | 23.3% | 13.7% | 7.7% |

## Data-backed walking retarget

Source: CMU Graphics Lab Motion Capture Database, subject 37, trial 1 (`slow walk`, 120 Hz). The ASF/AMC bytes are checksum-pinned and fetched into the ignored benchmark cache.

The selected cycle is 1.308 s; raw same-phase endpoint closure is 1.125 cm RMS. Leg/arm morphology scales are 1.034/0.616. Maximum reference speed/acceleration across cadence blocks are 3.045 m/s and 56.718 m/s².

Because the toy benchmark holds the pelvis fixed, the retarget removes the source pelvis-bob component by grounding the lower foot at every phase. Corpus construction rejects flight or missing bilateral stance/swing phases. The retained labels contain 25.7% double support, 0.0% flight, and 606 bilateral contact transitions.

The predeclared acceptance envelope requires overall foot/hand RMS ≤5/3 cm, stance and swing foot RMS ≤6 cm, clearance RMS ≤3 cm, minimum swing clearance ≥−1 cm, each cadence foot RMS ≤6 cm, and no contingency/rejected ticks.

| Impl | Gate | foot RMS cm | hand RMS cm | stance foot RMS cm | swing foot RMS cm | clearance RMS cm | min/peak achieved clearance cm | reference peak cm | transitions | target/tracked transition p99 cm |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Bonesaw | FAIL | 5.549 | 1.411 | 5.819 | 5.058 | 2.178 | 0.014/10.858 | 10.732 | 606 | 5.757/6.593 |
| PlaCo | FAIL | 3.701 | 0.427 | 3.584 | 3.890 | 3.301 | 0.741/13.282 | 10.732 | 606 | 5.757/5.496 |

| Cadence | Impl | all-effector RMS cm | foot RMS cm | hand RMS cm | ticks |
|---|---|---:|---:|---:|---:|
| 0.75x | Bonesaw | 3.857 | 5.276 | 1.384 | 1,400 |
| 0.75x | PlaCo | 2.540 | 3.567 | 0.426 | 1,400 |
| 1.00x | Bonesaw | 4.061 | 5.566 | 1.412 | 2,400 |
| 1.00x | PlaCo | 2.630 | 3.695 | 0.426 | 2,400 |
| 1.25x | Bonesaw | 4.237 | 5.816 | 1.439 | 1,200 |
| 1.25x | PlaCo | 2.748 | 3.863 | 0.428 | 1,200 |
- Bonesaw walking gate: FAIL — foot_tracking_rms_le_5cm
- PlaCo walking gate: FAIL — swing_clearance_error_rms_le_3cm

## Memory, CPU, faults, context switches, and Python GC

| Scenario | Impl | RSS before MB | peak RSS MB | RSS Δ MB | thread CPU/wall | process CPU/wall | minor faults | major faults | voluntary ctx | involuntary ctx | Python peak MB | GC collections |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 41.73 | 43.09 | 1.37 | 1.000 | 1.000 | 353 | 0 | 0 | 1 | 0.03 | 0 |
| end_effector_reach | PlaCo | 95.55 | 95.82 | 0.26 | 1.000 | 1.000 | 69 | 0 | 0 | 7 | 0.03 | 0 |
| bimanual_priority_conflict | Bonesaw | 45.83 | 47.20 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 4 | 0.03 | 0 |
| bimanual_priority_conflict | PlaCo | 99.33 | 99.59 | 0.26 | 1.000 | 1.000 | 68 | 0 | 0 | 12 | 0.03 | 0 |
| walking_motion_retarget | Bonesaw | 48.25 | 49.62 | 1.37 | 0.999 | 0.999 | 350 | 0 | 0 | 4 | 0.03 | 0 |
| walking_motion_retarget | PlaCo | 101.72 | 102.20 | 0.49 | 1.000 | 1.000 | 126 | 0 | 0 | 6 | 0.03 | 0 |

## Native controller heap-allocation sentinel

The standalone Rust executable samples its counting global allocator immediately
around each reusable controller transition. Corpus construction, statistics, and
state copying outside the controller call are excluded. Collision mode evaluates
all compiled pairs while reusing distance, task, constraint, solver, trajectory,
and output workspaces.

| Scenario | allocation calls/tick | allocated bytes/tick |
|---|---:|---:|
| end_effector_reach | 0.0 | 0.0 |
| bimanual_priority_conflict | 0.0 | 0.0 |
| walking_motion_retarget | 0.0 | 0.0 |
| collision-enabled controller (97 pairs) | 0.0 | 0.0 |
| compiled signal graph (13 nodes, 3 memory slots) | 0.0 | 0.0 |
| compiled signal→task→controller rig (3 task slots) | 0.0 | 0.0 |
| dynamic WBC (42 variables, 2 contacts) | 0.0 | 0.0 |
| floating dynamic WBC (48 variables, 2 contacts) | 0.0 | 0.0 |
| Upkie contact-IK floating squat preview | 0.0 | 0.0 |
| Upkie raw integrated balance + squat | 0.0 | 0.0 |

## Compiled signal-jet graph sentinel

This profile evaluates a flat scalar/vector graph with input and constant jets,
add, scale, analytic blend derivatives, deadband, clamp, low-pass, and critically
damped spring nodes. Stateful memory is explicit and double-buffered; graph topology,
stable IDs, types, memory slots, and outputs are frozen into MotionProgram archive v5
and its SHA-256 content fingerprint.

| ticks | nodes | outputs | state slots | max output accel | p50 us | p99 us | max us | bitwise repeat |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 13 | 2 | 3 | 10.250 | 0.261 | 0.330 | 3.076 | yes |

## Compiled signal-to-task controller sentinel

This profile runs the complete explicit transition: vector and rotation inputs,
low-pass and SO(3) spring state, resolved point, CoM, and orientation task slots,
FK/Jacobians, strict hierarchy,
quintic synthesis, and next-state signal-memory commit. Two independently sized
state/scratch/output streams receive identical inputs.

| ticks | signal nodes | task slots | point RMS cm | orientation RMS deg | p50 us | p99 us | max us | bitwise repeat |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 6 | 3 | 0.9214 | 0.1929 | 74.0 | 102.4 | 258.9 | yes |

## Unified inverse-dynamics WBC sentinel

The CPU dynamic profiles solve one strict problem over `[q̈, τ, contact force]`.
Hard rows enforce the rigid-body equation and locked/rolling contact acceleration;
bounds and inequalities enforce acceleration, torque, unilateral normal load, and
a four-sided friction pyramid. The floating profile prepends six unactuated root
accelerations and enforces all six free-body equilibrium rows without a root torque.

| profile | ticks | variables | contacts | accel tracking RMS | max abs(q̈) | max abs(τ) | max abs(f) | dynamics max | contact accel max | min friction margin | friction active | min torque margin | p50 µs | p99 µs | max µs | infeasible | bitwise repeat |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 5,000 | 42 | 2 | 1.021e-01 | 2.500e-01 | 1.555e+02 | 2.356e+02 | 1.04e-13 | 5.13e-18 | -2.84e-14 | 4,674 | 9.84e+03 | 687.1 | 714.0 | 934.3 | 0 | yes |
| floating | 5,000 | 48 | 2 | 1.058e-02 | 3.723e-02 | 7.838e-01 | 1.972e+02 | 5.14e-10 | 5.19e-11 | 1.57e+02 | 0 | 1.00e+04 | 650.2 | 668.6 | 989.7 | 0 | yes |

## Upkie floating balance and squat

Both 200 Hz sentinels generate a two-contact planar posture with allocation-free
damped Gauss-Newton IK and solve the same floating WBC. The projected row is
retained as an explicit guided diagnostic; the raw row is the default browser
and dynamic regression path. Root attitude, root translation, and CoM feedback are emitted by
the same fixed-slot compiled policy used by the server. A compiled 1 Hz
critically damped root-translation spring shapes discontinuous editor targets. The adapter
supplies target jets and the articulation-aware axle-to-CoM pitch observation.
The raw row integrates only the solver output and uses canonical wheel-center
rolling constraints plus a bounded wheel-acceleration task derived from Upkie's
velocity PI law.
`Slip`
measures only constrained lateral/normal motion; wheel-axis travel is reported
separately and is physically permitted.

| path | ticks | target cm | achieved cm | final z err mm | z RMS cm | xy RMS cm | CoM RMS cm | slip mm | wheel travel cm | max rotation deg | max qdd | max tau | max force | dynamics max | contact accel max | min friction | min torque | degraded | infeasible | p50 us | p99 us | max us |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| projected adapter | 5,000 | 12.00 | 12.00 | 0.000 | 0.00 | 0.00 | 0.01 | 0.000 | 0.00 | 0.000 | 200.00 | 2.03 | 29.66 | 1.77e-11 | 5.20e-12 | -9.55e-15 | 1.67e+00 | 4,778 | 0 | 138.9 | 161.4 | 210.7 |
| raw integrated WBC | 5,000 | 12.00 | 12.00 | -0.000 | 0.54 | 0.52 | 0.58 | 0.006 | 1.73 | 0.000 | 2.02 | 2.40 | 26.41 | 2.19e-11 | 4.99e-12 | 2.07e+01 | 1.70e+00 | 0 | 0 | 198.0 | 230.0 | 375.5 |

The raw policy contains 4 signal nodes and 3 resolved floating task slots. Its emitted root/CoM acceleration differs from the direct formula oracle by at most `3.469e-17`; the full IK → signals → task emission → WBC → SE(3) integration loop reports zero allocator calls and bytes per step.

## Official Upkie wheel-controller oracle

A separate 100,000-step worker links the pinned upstream
`WheelBalancer.cpp` class unchanged. On the common floor-contact,
stationary-target subset, Bonesaw is evaluated once with upstream
parameters for an exact implementation gate and once with live tuned
parameters to expose the intentional policy delta.

| Gate | Result |
|---|---:|
| Official-parameter wheel commands | PASS · 0 canonical bit mismatches |
| Live-tuned ground-velocity delta | RMS 0.1177 m/s · max 0.3772 m/s · correlation 0.995085 |
| Official C++ adapter latency | p50 1.372 µs · p99 1.643 µs |
| Rust typed-law latency | p50 0.040 µs · p99 0.060 µs |
| Rust hot-loop allocation | 0 calls · 0 bytes |

See [UPKIE_CONTROLLER_COMPARISON.md](UPKIE_CONTROLLER_COMPARISON.md)
for per-region behavior, p99.99 latency, jitter, ten temporal windows,
RSS/CPU/fault/context-switch metrics, provenance hashes, and raw artifacts.

## PlaCo solver-only latency

This removes Python target assignment, both kinematics updates, and frame
extraction from PlaCo's end-to-end step timing.

| Scenario | mean µs | p50 µs | p95 µs | p99 µs | p99.9 µs | max µs |
|---|---:|---:|---:|---:|---:|---:|
| end_effector_reach | 37.5 | 37.0 | 42.1 | 44.9 | 57.3 | 107.6 |
| bimanual_priority_conflict | 47.2 | 46.5 | 52.3 | 55.9 | 64.1 | 118.6 |
| walking_motion_retarget | 42.1 | 41.5 | 46.9 | 50.0 | 57.3 | 228.5 |

## Temporal drift by execution window

Each row is one tenth of a run. This exposes warm drift, allocator/GC episodes,
thermal/scheduler outliers, and tracking degradation hidden by one aggregate.

| Scenario | Impl | tick range | p50 µs | p99 µs | RMS cm | p99 error cm |
|---|---|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0–499 | 40.5 | 57.1 | 1.955 | 8.660 |
| end_effector_reach | Bonesaw | 500–999 | 40.1 | 45.7 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1000–1499 | 40.1 | 45.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1500–1999 | 40.1 | 44.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2000–2499 | 40.5 | 54.2 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2500–2999 | 40.1 | 45.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3000–3499 | 40.2 | 45.1 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3500–3999 | 40.1 | 44.7 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4000–4499 | 40.1 | 44.7 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4500–4999 | 40.0 | 45.4 | 0.129 | 0.133 |
| end_effector_reach | PlaCo | 0–499 | 93.7 | 114.5 | 1.290 | 5.696 |
| end_effector_reach | PlaCo | 500–999 | 93.5 | 112.1 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 1000–1499 | 95.0 | 114.1 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 1500–1999 | 93.5 | 113.1 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 2000–2499 | 93.8 | 113.5 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 2500–2999 | 94.5 | 114.4 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 3000–3499 | 93.9 | 111.1 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 3500–3999 | 94.2 | 113.2 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 4000–4499 | 94.1 | 112.8 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 4500–4999 | 94.6 | 112.3 | 0.731 | 0.798 |
| bimanual_priority_conflict | Bonesaw | 0–499 | 86.6 | 103.4 | 1.501 | 8.894 |
| bimanual_priority_conflict | Bonesaw | 500–999 | 90.8 | 101.9 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1000–1499 | 90.2 | 119.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1500–1999 | 89.8 | 136.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2000–2499 | 90.4 | 110.0 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2500–2999 | 92.9 | 105.4 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3000–3499 | 91.2 | 105.0 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3500–3999 | 88.8 | 95.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4000–4499 | 89.0 | 99.9 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4500–4999 | 93.2 | 105.7 | 0.000 | 0.000 |
| bimanual_priority_conflict | PlaCo | 0–499 | 106.4 | 127.0 | 0.869 | 0.286 |
| bimanual_priority_conflict | PlaCo | 500–999 | 106.9 | 126.6 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 1000–1499 | 106.8 | 127.9 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 1500–1999 | 106.8 | 127.1 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 2000–2499 | 106.5 | 126.6 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 2500–2999 | 106.6 | 128.6 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 3000–3499 | 107.3 | 128.3 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 3500–3999 | 106.7 | 127.2 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 4000–4499 | 106.3 | 126.8 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 4500–4999 | 107.3 | 127.1 | 0.250 | 0.253 |
| walking_motion_retarget | Bonesaw | 0–499 | 66.0 | 75.7 | 3.882 | 14.242 |
| walking_motion_retarget | Bonesaw | 500–999 | 67.2 | 75.7 | 4.075 | 14.573 |
| walking_motion_retarget | Bonesaw | 1000–1499 | 67.1 | 74.1 | 4.221 | 14.647 |
| walking_motion_retarget | Bonesaw | 1500–1999 | 66.7 | 75.2 | 3.937 | 14.271 |
| walking_motion_retarget | Bonesaw | 2000–2499 | 67.4 | 88.6 | 4.088 | 14.573 |
| walking_motion_retarget | Bonesaw | 2500–2999 | 67.3 | 75.7 | 4.186 | 14.753 |
| walking_motion_retarget | Bonesaw | 3000–3499 | 66.9 | 73.8 | 3.859 | 14.187 |
| walking_motion_retarget | Bonesaw | 3500–3999 | 67.2 | 78.2 | 3.997 | 14.366 |
| walking_motion_retarget | Bonesaw | 4000–4499 | 67.3 | 76.4 | 4.284 | 14.839 |
| walking_motion_retarget | Bonesaw | 4500–4999 | 66.9 | 74.6 | 3.929 | 14.187 |
| walking_motion_retarget | PlaCo | 0–499 | 128.9 | 156.2 | 2.546 | 11.321 |
| walking_motion_retarget | PlaCo | 500–999 | 128.9 | 147.0 | 2.626 | 11.387 |
| walking_motion_retarget | PlaCo | 1000–1499 | 131.0 | 150.9 | 2.754 | 11.416 |
| walking_motion_retarget | PlaCo | 1500–1999 | 129.8 | 150.0 | 2.590 | 11.331 |
| walking_motion_retarget | PlaCo | 2000–2499 | 128.9 | 153.4 | 2.643 | 11.399 |
| walking_motion_retarget | PlaCo | 2500–2999 | 131.1 | 157.7 | 2.714 | 11.398 |
| walking_motion_retarget | PlaCo | 3000–3499 | 128.8 | 154.4 | 2.504 | 11.290 |
| walking_motion_retarget | PlaCo | 3500–3999 | 129.8 | 155.6 | 2.593 | 11.381 |
| walking_motion_retarget | PlaCo | 4000–4499 | 130.0 | 154.6 | 2.801 | 11.398 |
| walking_motion_retarget | PlaCo | 4500–4999 | 127.9 | 153.6 | 2.554 | 11.290 |

## Process startup and resident footprint

| Implementation | import ms | setup ms | baseline RSS MB | after import MB | after setup MB | import Δ MB | setup Δ MB |
|---|---:|---:|---:|---:|---:|---:|---:|
| Bonesaw | 0.97 | 0.57 | 39.23 | 39.90 | 40.67 | 0.66 | 0.77 |
| PlaCo | 57.69 | 6.41 | 39.16 | 88.27 | 101.58 | 49.11 | 13.31 |

## Artifacts and interpretation

- `input-corpus.npz`: exact shared targets, activation masks, CMU gait phase, cadence, and reference stance labels.
- `input-manifest.json`: CMU source/license/checksum provenance plus cycle, morphology, and target-bandwidth metadata.
- `bonesaw-raw.npz`: native per-step latency, status, q/v, tracked positions, and errors.
- `placo-raw.npz`: end-to-end and solver-only per-step latency, status, tracked positions, and errors.
- `native-cpu-eval.json`: Rust allocator, dynamics, collision, constraint, determinism, and query sentinels.
- `native-upkie-eval.json`: Upkie floating dynamics and contact-consistent interactive squat sentinel.
- `reference-metrics.json`: complete machine-readable aggregates and ten temporal windows.
- `UPKIE_CONTROLLER_COMPARISON.md`: direct pinned C++ controller-law oracle with per-step traces.
- `upkie-controller-raw.npz`: shared balance inputs, commands, integral states, and latency samples.
- `pinocchio-upkie.json` and `pinocchio-g1.json`: complete fixed/floating product maxima, thresholds, state/frame counts, and coordinate provenance.
- `../floating-g1-liftoff-latest/`: official-G1 green moving-liftoff metrics, full state/residual/latency trace, and standalone report.
- `../floating-g1-transfer-latest/`: official-G1 red full-transfer stress with every fallback/release/rejected tick retained.
- `../../../docs/PINOCCHIO_ORACLE.md`: independent FK/Jacobian/mass/gravity/inverse-dynamics correctness.

The comparison is a regression instrument, not a claim that one formulation is
universally better. Bonesaw's strict hierarchy is the intended product behavior;
PlaCo's QP is an intentionally independent reference with different tradeoffs.

Sources: [PlaCo repository](https://github.com/Rhoban/placo), [PlaCo kinematics loop documentation](https://placo.readthedocs.io/en/stable/kinematics/getting_started.html), [Pinocchio repository](https://github.com/stack-of-tasks/pinocchio), [Upkie repository](https://github.com/upkie/upkie).
