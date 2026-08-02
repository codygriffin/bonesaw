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
| end_effector_reach | Bonesaw | 0.630 | 0.129 | 39.4 | 52.8 | 61.1 | 70.6 | 0 | 24770 |
| end_effector_reach | PlaCo | 0.809 | 0.737 | 95.2 | 170.4 | 198.3 | 246.8 | 0 | 9427 |
| bimanual_priority_conflict | Bonesaw | 0.475 | 0.000 | 87.6 | 102.8 | 111.6 | 140.5 | 0 | 11345 |
| bimanual_priority_conflict | PlaCo | 0.365 | 0.252 | 108.3 | 124.8 | 136.2 | 252.3 | 0 | 8637 |
| walking_motion_retarget | Bonesaw | 4.048 | 4.062 | 63.1 | 72.3 | 83.6 | 239.4 | 0 | 15811 |
| walking_motion_retarget | PlaCo | 2.634 | 2.645 | 132.3 | 159.3 | 168.6 | 335.5 | 0 | 7097 |

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
| moving liftoff | PASS | PASS | 260 | 260 | 0.787 | 0.000 | 0.342 | 2.024 | 8.000 | 199/61/0 | 0/0/0/0 | 1.22e-09 | 5.11e-11 |
| full transfer stress | FAIL | FAIL | 600 | 480 | 59.631 | 32.232 | 52.251 | 179.887 | 8.000 | 126/102/0 | 96/24/0/0 | 9.87e-09 | 5.03e-10 |

The moving-liftoff row passes both the behavioral and unchanged 5 ms p99 CPU gates.
The transfer row remains a deliberately retained failing stress case. Rejected ticks
retain raw residuals and state rather than being omitted from aggregates.

### G1 latency, jitter, and deadlines

| profile | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 2383.5 | 434.0 | 23.3 | 2333.2 | 3611.7 | 4124.6 | 4637.8 | 4768.1 | 4782.5 | 898.4 | 260 | 0 | 0 | 419.5 |
| full transfer stress | 12486.0 | 33434.6 | 260.6 | 2607.8 | 81672.5 | 183856.6 | 197108.6 | 200789.4 | 201198.4 | 171153.4 | 600 | 101 | 57 | 80.1 |

### G1 process resources

Buffers and the Rust session are allocated before this measurement. Python
`tracemalloc` excludes native allocation; zero hot-loop allocation is gated
separately by the native Rust sentinels later in this report.

| profile | wall s | process CPU s | thread CPU s | process CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor/major faults | voluntary/involuntary ctx |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0.620 | 0.620 | 0.620 | 0.999 | 47.28 | 47.28 | 0.00 | 48.97 | 0.00 | 0 | 0/0 | 0/8 |
| full transfer stress | 7.492 | 7.474 | 7.474 | 0.998 | 48.26 | 48.71 | 0.45 | 49.39 | 0.00 | 0 | 93/0 | 0/1,494 |

### G1 latency by solver/contact status

| profile | status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---|---:|---:|---:|---:|---:|
| moving liftoff | solved | 199 | 2334.0 | 2387.9 | 2419.2 | 2474.9 |
| moving liftoff | solved_with_slack | 61 | 2190.4 | 4114.6 | 4447.1 | 4782.5 |
| full transfer stress | solved | 126 | 2514.8 | 2633.2 | 2678.2 | 3111.8 |
| full transfer stress | solved_with_slack | 102 | 2580.7 | 2918.8 | 4136.5 | 4670.3 |
| full transfer stress | normal_contact_contingency | 96 | 5113.6 | 32032.4 | 48643.3 | 183919.4 |
| full transfer stress | contact_release_contingency | 24 | 165366.9 | 193722.8 | 199628.0 | 201198.4 |
| full transfer stress | precontact_transition | 252 | 2646.9 | 74764.5 | 88026.2 | 99046.7 |

### G1 strict-solver work attribution

Task-level pseudoinverse and Jacobi-sweep counts expose the dense-kernel
work behind each tick. The solver reuses the feasibility seed's exact
equality factorization, reuses a projected inverse while its nullspace is
unchanged, and skips the terminal Style projector because no lower priority
can consume it. Near-feasible seeds still polish and refactor. Optimizations
that change floating-point summation order are evaluated as algorithmic
revisions; this report does not claim bitwise behavior/status-trace parity.

| profile | pseudoinverse mean/p95/p99/max | Jacobi sweeps mean/p95/p99/max | sweeps/pseudoinverse | clipped-step mean/p95/p99/max | calls↔latency correlation |
|---|---:|---:|---:|---:|---:|
| moving liftoff | 4.95/9.0/11.0/14 | 27.92/48.0/58.9/85 | 5.64 | 1.17/7.0/9.0/11 | 0.4426 |
| full transfer stress | 7.75/12.0/13.0/25 | 49.80/79.0/91.1/165 | 6.43 | 5.51/11.0/12.0/21 | 0.1094 |

| profile | priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---|---:|---:|---:|---:|
| moving liftoff | invariant | 1.12/3.0/3 | 3.71/12.0/12 | 0.16/3.0/3 | 18 |
| moving liftoff | viability | 0.26/2.0/4 | 0.93/8.0/16 | 0.08/2.0/4 | 13 |
| moving liftoff | intent | 1.15/3.4/6 | 2.31/6.8/12 | 0.28/3.4/6 | 32 |
| moving liftoff | preference | 1.41/5.4/8 | 10.25/43.3/64 | 0.64/5.4/8 | 61 |
| moving liftoff | style | 1.00/1.0/1 | 10.71/12.0/13 | 0.02/1.0/1 | 4 |
| full transfer stress | invariant | 1.85/5.0/8 | 8.19/25.0/40 | 1.28/5.0/8 | 270 |
| full transfer stress | viability | 1.89/6.0/8 | 12.00/42.0/56 | 1.52/6.0/8 | 384 |
| full transfer stress | intent | 1.54/5.0/8 | 8.12/29.0/38 | 1.31/5.0/8 | 468 |
| full transfer stress | preference | 1.41/6.0/20 | 12.59/63.0/140 | 0.99/6.0/19 | 379 |
| full transfer stress | style | 1.06/2.0/4 | 8.90/20.0/33 | 0.40/2.0/4 | 213 |

### G1 execution over time

| profile | ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0–25 | 2340.9 | 2393.4 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 26–51 | 2341.9 | 2397.8 | 4.00 | 23.96 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 52–77 | 2331.9 | 2427.4 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 78–103 | 2334.1 | 2366.8 | 4.00 | 23.77 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 104–129 | 2337.4 | 2373.2 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 130–155 | 2325.1 | 2448.1 | 4.00 | 23.81 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 156–181 | 2323.0 | 2405.8 | 4.00 | 23.65 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 182–207 | 2281.7 | 2553.0 | 5.23 | 30.42 | 1.27 | 0.007 | 0.001 | 9.09e-10 | 5.11e-11 | 0 |
| moving liftoff | 208–233 | 2047.8 | 2838.7 | 7.69 | 42.85 | 3.96 | 0.043 | 0.009 | 8.98e-10 | 1.45e-11 | 0 |
| moving liftoff | 234–259 | 3630.0 | 4642.8 | 8.54 | 39.15 | 6.50 | 2.488 | 0.370 | 5.11e-10 | 4.23e-12 | 0 |
| full transfer stress | 0–59 | 2491.4 | 2828.6 | 5.00 | 33.53 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 5.55e-11 | 0 |
| full transfer stress | 60–119 | 2593.1 | 3315.1 | 5.47 | 37.28 | 1.05 | 0.151 | 0.000 | 1.27e-09 | 3.93e-11 | 0 |
| full transfer stress | 120–179 | 2557.2 | 2863.2 | 5.98 | 39.17 | 2.03 | 1.369 | 0.000 | 1.03e-09 | 5.03e-11 | 0 |
| full transfer stress | 180–239 | 2531.2 | 3725.3 | 8.05 | 51.20 | 5.53 | 6.458 | 1.270 | 1.35e-09 | 5.23e-11 | 12 |
| full transfer stress | 240–299 | 2597.9 | 4073.1 | 8.35 | 53.00 | 6.87 | 7.203 | 23.318 | 8.57e-10 | 2.38e-11 | 60 |
| full transfer stress | 300–359 | 2524.9 | 3947.7 | 8.92 | 56.68 | 7.58 | 13.835 | 25.111 | 1.12e-09 | 2.29e-11 | 60 |
| full transfer stress | 360–419 | 2560.7 | 32747.7 | 8.92 | 55.33 | 7.82 | 29.366 | 61.577 | 8.20e-10 | 9.52e-12 | 60 |
| full transfer stress | 420–479 | 3721.1 | 96418.4 | 9.65 | 64.95 | 8.70 | 59.372 | 40.263 | 8.00e-09 | 5.03e-10 | 60 |
| full transfer stress | 480–539 | 21543.1 | 197170.1 | 8.83 | 54.12 | 8.07 | 122.140 | 38.812 | 9.87e-09 | 1.97e-10 | 60 |
| full transfer stress | 540–599 | 3495.5 | 180620.4 | 8.33 | 52.70 | 7.45 | 126.356 | 88.820 | 5.05e-09 | 1.57e-10 | 60 |

### G1 artifact integrity

| profile | metrics SHA-256 | raw NPZ SHA-256 | directory |
|---|---|---|---|
| moving liftoff | `296925294571e4270f2b6a42b2a8bfa17b8764df7f91ba72208382f67e091534` | `5970133b91a1ed7578fae4c269ea14e23f741367788130975d930035ff180ed9` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-liftoff-r38-reference-fix` |
| full transfer stress | `28bb97a4d666ca0190a1362c87e1e1c2b63646a2505eaa861dd5d258707795a5` | `1bbaf8f2f7dcac1907c1f4ff2abfe5f86a5a5f72c9ebda7a06ede0239c3af34d` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/g1-transfer-r38-preview-200` |

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
| end_effector_reach | Bonesaw | 39.7 | 2.5 | 1.1 | 41.1 | 43.6 | 52.8 | 68.8 | 9.0 |
| end_effector_reach | PlaCo | 99.6 | 15.4 | 1.5 | 103.5 | 132.7 | 170.4 | 241.7 | 59.8 |
| bimanual_priority_conflict | Bonesaw | 87.5 | 5.5 | 2.1 | 91.6 | 94.1 | 102.8 | 130.1 | 16.0 |
| bimanual_priority_conflict | PlaCo | 109.4 | 4.2 | 1.2 | 114.2 | 116.0 | 124.8 | 202.1 | 18.1 |
| walking_motion_retarget | Bonesaw | 62.6 | 4.7 | 2.8 | 67.5 | 68.3 | 72.3 | 174.5 | 13.1 |
| walking_motion_retarget | PlaCo | 134.3 | 6.9 | 2.0 | 140.6 | 146.0 | 159.3 | 306.1 | 24.5 |

## Tracking quality

| Scenario | Impl | median cm | p95 cm | p99 cm | max cm | IAE m·s | ISE m²·s | >1 cm | >3 cm | >5 cm |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0.129 | 0.133 | 0.133 | 24.569 | 0.304 | 0.0079 | 0.2% | 0.2% | 0.1% |
| end_effector_reach | PlaCo | 0.737 | 0.797 | 0.798 | 14.460 | 1.494 | 0.0131 | 0.2% | 0.2% | 0.1% |
| bimanual_priority_conflict | Bonesaw | 0.000 | 0.000 | 0.000 | 17.858 | 0.043 | 0.0045 | 0.3% | 0.2% | 0.2% |
| bimanual_priority_conflict | PlaCo | 0.255 | 0.256 | 0.256 | 16.967 | 0.515 | 0.0027 | 0.1% | 0.0% | 0.0% |
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
| end_effector_reach | Bonesaw | 41.97 | 43.34 | 1.37 | 1.000 | 1.000 | 352 | 0 | 0 | 1 | 0.03 | 0 |
| end_effector_reach | PlaCo | 95.69 | 95.95 | 0.27 | 1.000 | 1.000 | 70 | 0 | 0 | 7 | 0.03 | 0 |
| bimanual_priority_conflict | Bonesaw | 46.09 | 47.46 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 5 | 0.03 | 0 |
| bimanual_priority_conflict | PlaCo | 99.46 | 99.72 | 0.26 | 1.000 | 1.000 | 68 | 0 | 0 | 8 | 0.03 | 0 |
| walking_motion_retarget | Bonesaw | 48.50 | 49.87 | 1.37 | 0.999 | 0.999 | 350 | 0 | 0 | 7 | 0.03 | 0 |
| walking_motion_retarget | PlaCo | 101.87 | 102.35 | 0.50 | 1.000 | 1.000 | 129 | 0 | 0 | 7 | 0.03 | 0 |

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
| 5,000 | 13 | 2 | 3 | 10.250 | 0.271 | 0.291 | 2.545 | yes |

## Compiled signal-to-task controller sentinel

This profile runs the complete explicit transition: vector and rotation inputs,
low-pass and SO(3) spring state, resolved point, CoM, and orientation task slots,
FK/Jacobians, strict hierarchy,
quintic synthesis, and next-state signal-memory commit. Two independently sized
state/scratch/output streams receive identical inputs.

| ticks | signal nodes | task slots | point RMS cm | orientation RMS deg | p50 us | p99 us | max us | bitwise repeat |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 6 | 3 | 0.9254 | 0.1587 | 74.3 | 103.2 | 343.6 | yes |

## Unified inverse-dynamics WBC sentinel

The CPU dynamic profiles solve one strict problem over `[q̈, τ, contact force]`.
Hard rows enforce the rigid-body equation and locked/rolling contact acceleration;
bounds and inequalities enforce acceleration, torque, unilateral normal load, and
a four-sided friction pyramid. The floating profile prepends six unactuated root
accelerations and enforces all six free-body equilibrium rows without a root torque.

| profile | ticks | variables | contacts | accel tracking RMS | max abs(q̈) | max abs(τ) | max abs(f) | dynamics max | contact accel max | min friction margin | friction active | min torque margin | p50 µs | p99 µs | max µs | infeasible | bitwise repeat |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 5,000 | 42 | 2 | 1.021e-01 | 2.500e-01 | 1.555e+02 | 2.356e+02 | 7.59e-14 | 5.21e-18 | -2.84e-14 | 4,674 | 9.84e+03 | 584.9 | 990.1 | 1221.3 | 0 | yes |
| floating | 5,000 | 48 | 2 | 1.058e-02 | 3.723e-02 | 7.838e-01 | 1.972e+02 | 5.14e-10 | 5.19e-11 | 1.57e+02 | 0 | 1.00e+04 | 631.4 | 653.2 | 932.7 | 0 | yes |

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
| projected adapter | 5,000 | 12.00 | 12.00 | 0.000 | 0.00 | 0.00 | 0.01 | 0.000 | 0.00 | 0.000 | 200.00 | 2.03 | 29.66 | 1.77e-11 | 5.19e-12 | -7.37e-15 | 1.67e+00 | 4,778 | 0 | 140.6 | 162.7 | 384.3 |
| raw integrated WBC | 5,000 | 12.00 | 12.00 | -0.000 | 0.54 | 0.52 | 0.58 | 0.006 | 1.73 | 0.000 | 2.02 | 2.40 | 26.41 | 2.19e-11 | 4.99e-12 | 2.07e+01 | 1.70e+00 | 0 | 0 | 200.9 | 230.6 | 309.8 |

The raw policy contains 4 signal nodes and 3 resolved floating task slots. Its emitted root/CoM acceleration differs from the direct formula oracle by at most `4.163e-17`; the full IK → signals → task emission → WBC → SE(3) integration loop reports zero allocator calls and bytes per step.

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
| Official C++ adapter latency | p50 1.393 µs · p99 2.314 µs |
| Rust typed-law latency | p50 0.030 µs · p99 0.031 µs |
| Rust hot-loop allocation | 0 calls · 0 bytes |

See [UPKIE_CONTROLLER_COMPARISON.md](UPKIE_CONTROLLER_COMPARISON.md)
for per-region behavior, p99.99 latency, jitter, ten temporal windows,
RSS/CPU/fault/context-switch metrics, provenance hashes, and raw artifacts.

## PlaCo solver-only latency

This removes Python target assignment, both kinematics updates, and frame
extraction from PlaCo's end-to-end step timing.

| Scenario | mean µs | p50 µs | p95 µs | p99 µs | p99.9 µs | max µs |
|---|---:|---:|---:|---:|---:|---:|
| end_effector_reach | 39.7 | 37.9 | 55.3 | 69.3 | 77.3 | 117.5 |
| bimanual_priority_conflict | 47.7 | 47.1 | 52.7 | 56.5 | 69.9 | 127.0 |
| walking_motion_retarget | 42.7 | 42.2 | 47.8 | 50.6 | 56.8 | 127.6 |

## Temporal drift by execution window

Each row is one tenth of a run. This exposes warm drift, allocator/GC episodes,
thermal/scheduler outliers, and tracking degradation hidden by one aggregate.

| Scenario | Impl | tick range | p50 µs | p99 µs | RMS cm | p99 error cm |
|---|---|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0–499 | 39.9 | 55.0 | 1.955 | 8.660 |
| end_effector_reach | Bonesaw | 500–999 | 39.2 | 55.5 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1000–1499 | 39.9 | 48.8 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1500–1999 | 39.3 | 47.7 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2000–2499 | 39.4 | 45.1 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2500–2999 | 39.0 | 45.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3000–3499 | 39.5 | 44.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3500–3999 | 39.1 | 47.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4000–4499 | 39.3 | 44.8 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4500–4999 | 39.0 | 45.7 | 0.129 | 0.133 |
| end_effector_reach | PlaCo | 0–499 | 95.5 | 166.4 | 1.290 | 5.696 |
| end_effector_reach | PlaCo | 500–999 | 95.1 | 165.8 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 1000–1499 | 95.2 | 170.5 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 1500–1999 | 96.6 | 167.7 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 2000–2499 | 95.2 | 166.7 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 2500–2999 | 95.3 | 170.2 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 3000–3499 | 94.9 | 166.2 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 3500–3999 | 94.3 | 165.2 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 4000–4499 | 94.9 | 172.4 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 4500–4999 | 94.6 | 189.6 | 0.731 | 0.798 |
| bimanual_priority_conflict | Bonesaw | 0–499 | 86.5 | 101.7 | 1.501 | 8.894 |
| bimanual_priority_conflict | Bonesaw | 500–999 | 88.0 | 103.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1000–1499 | 87.9 | 108.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1500–1999 | 87.4 | 97.5 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2000–2499 | 87.7 | 96.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2500–2999 | 87.5 | 101.3 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3000–3499 | 87.6 | 96.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3500–3999 | 87.8 | 100.4 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4000–4499 | 87.2 | 98.7 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4500–4999 | 87.7 | 95.5 | 0.000 | 0.000 |
| bimanual_priority_conflict | PlaCo | 0–499 | 109.2 | 128.3 | 0.870 | 0.286 |
| bimanual_priority_conflict | PlaCo | 500–999 | 109.1 | 125.1 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 1000–1499 | 108.5 | 121.2 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 1500–1999 | 108.1 | 121.9 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 2000–2499 | 107.8 | 118.0 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 2500–2999 | 108.0 | 121.7 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 3000–3499 | 107.8 | 121.7 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 3500–3999 | 107.7 | 122.6 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 4000–4499 | 108.4 | 120.8 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 4500–4999 | 107.5 | 128.0 | 0.252 | 0.256 |
| walking_motion_retarget | Bonesaw | 0–499 | 62.7 | 72.6 | 3.882 | 14.242 |
| walking_motion_retarget | Bonesaw | 500–999 | 63.3 | 71.8 | 4.075 | 14.573 |
| walking_motion_retarget | Bonesaw | 1000–1499 | 63.3 | 69.4 | 4.221 | 14.647 |
| walking_motion_retarget | Bonesaw | 1500–1999 | 62.6 | 75.0 | 3.937 | 14.271 |
| walking_motion_retarget | Bonesaw | 2000–2499 | 63.0 | 72.1 | 4.088 | 14.573 |
| walking_motion_retarget | Bonesaw | 2500–2999 | 63.3 | 71.9 | 4.186 | 14.753 |
| walking_motion_retarget | Bonesaw | 3000–3499 | 62.7 | 70.0 | 3.859 | 14.187 |
| walking_motion_retarget | Bonesaw | 3500–3999 | 63.1 | 79.1 | 3.997 | 14.366 |
| walking_motion_retarget | Bonesaw | 4000–4499 | 63.2 | 71.9 | 4.284 | 14.839 |
| walking_motion_retarget | Bonesaw | 4500–4999 | 63.0 | 71.5 | 3.929 | 14.187 |
| walking_motion_retarget | PlaCo | 0–499 | 130.9 | 156.9 | 2.546 | 11.321 |
| walking_motion_retarget | PlaCo | 500–999 | 130.9 | 157.3 | 2.626 | 11.387 |
| walking_motion_retarget | PlaCo | 1000–1499 | 132.6 | 159.8 | 2.754 | 11.416 |
| walking_motion_retarget | PlaCo | 1500–1999 | 132.9 | 162.7 | 2.590 | 11.331 |
| walking_motion_retarget | PlaCo | 2000–2499 | 133.2 | 160.3 | 2.643 | 11.399 |
| walking_motion_retarget | PlaCo | 2500–2999 | 132.7 | 159.7 | 2.714 | 11.398 |
| walking_motion_retarget | PlaCo | 3000–3499 | 133.0 | 153.5 | 2.504 | 11.290 |
| walking_motion_retarget | PlaCo | 3500–3999 | 131.7 | 156.1 | 2.593 | 11.381 |
| walking_motion_retarget | PlaCo | 4000–4499 | 133.0 | 159.0 | 2.801 | 11.398 |
| walking_motion_retarget | PlaCo | 4500–4999 | 131.7 | 156.0 | 2.554 | 11.290 |

## Process startup and resident footprint

| Implementation | import ms | setup ms | baseline RSS MB | after import MB | after setup MB | import Δ MB | setup Δ MB |
|---|---:|---:|---:|---:|---:|---:|---:|
| Bonesaw | 1.63 | 0.57 | 39.38 | 40.15 | 40.86 | 0.77 | 0.70 |
| PlaCo | 54.46 | 6.67 | 39.33 | 88.44 | 101.72 | 49.11 | 13.28 |

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
