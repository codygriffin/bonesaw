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
| end_effector_reach | Bonesaw | 0.630 | 0.129 | 40.5 | 46.3 | 55.1 | 70.6 | 0 | 24260 |
| end_effector_reach | PlaCo | 0.809 | 0.737 | 96.7 | 165.8 | 176.2 | 247.1 | 0 | 9376 |
| bimanual_priority_conflict | Bonesaw | 0.475 | 0.000 | 88.6 | 107.1 | 116.8 | 130.9 | 0 | 11223 |
| bimanual_priority_conflict | PlaCo | 0.363 | 0.250 | 106.8 | 184.2 | 195.1 | 250.2 | 0 | 8515 |
| walking_motion_retarget | Bonesaw | 4.048 | 4.062 | 65.1 | 75.2 | 84.9 | 87.2 | 0 | 15309 |
| walking_motion_retarget | PlaCo | 2.634 | 2.645 | 128.6 | 221.8 | 236.0 | 250.8 | 0 | 7128 |

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
| moving liftoff | PASS | PASS | 260 | 260 | 0.748 | 0.000 | 0.173 | 2.256 | 8.000 | 199/61/0 | 0/0/0/0 | 1.22e-09 | 5.16e-11 |
| full transfer stress | FAIL | FAIL | 600 | 335 | 89.271 | 63.121 | 82.264 | 179.946 | 8.000 | 199/136/95 | 67/38/65/0 | 7.08e-09 | 3.47e-10 |

The moving-liftoff row passes both the behavioral and unchanged 5 ms p99 CPU gates.
The transfer row remains a deliberately retained failing stress case. Rejected ticks
retain raw residuals and state rather than being omitted from aggregates.

### G1 latency, jitter, and deadlines

| profile | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 2356.3 | 396.2 | 19.0 | 2320.2 | 3535.8 | 3904.2 | 4298.6 | 4370.9 | 4379.0 | 829.1 | 260 | 0 | 0 | 424.4 |
| full transfer stress | 47737.6 | 96432.4 | 925.6 | 3209.1 | 293301.2 | 295760.7 | 346550.1 | 407883.8 | 414698.7 | 144366.9 | 600 | 207 | 122 | 20.9 |

### G1 process resources

Buffers and the Rust session are allocated before this measurement. Python
`tracemalloc` excludes native allocation; zero hot-loop allocation is gated
separately by the native Rust sentinels later in this report.

| profile | wall s | process CPU s | thread CPU s | process CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor/major faults | voluntary/involuntary ctx |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0.613 | 0.613 | 0.613 | 1.000 | 47.78 | 47.78 | 0.00 | 47.78 | 0.00 | 0 | 0/0 | 0/7 |
| full transfer stress | 28.643 | 28.639 | 28.639 | 1.000 | 47.61 | 47.86 | 0.25 | 48.02 | 0.00 | 0 | 40/0 | 0/183 |

### G1 latency by solver/contact status

| profile | status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---|---:|---:|---:|---:|---:|
| moving liftoff | solved | 199 | 2322.1 | 2373.8 | 2435.6 | 3120.1 |
| moving liftoff | solved_with_slack | 61 | 2044.2 | 3830.1 | 4192.8 | 4379.0 |
| full transfer stress | solved | 199 | 2337.7 | 2383.2 | 2423.9 | 2672.6 |
| full transfer stress | solved_with_slack | 136 | 3195.1 | 50316.9 | 98599.7 | 102765.8 |
| full transfer stress | primal_infeasible | 65 | 293277.3 | 297650.6 | 299293.8 | 300928.0 |
| full transfer stress | normal_contact_contingency | 67 | 5149.8 | 16238.0 | 123289.7 | 165029.5 |
| full transfer stress | contact_release_contingency | 38 | 166216.5 | 262852.4 | 367883.4 | 414698.7 |
| full transfer stress | touchdown_transition | 95 | 4057.5 | 55603.7 | 75559.5 | 87316.8 |

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
| moving liftoff | 4.96/10.0/12.0/13 | 27.68/46.1/62.2/77 | 5.58 | 1.17/7.0/9.4/12 | 0.4057 |
| full transfer stress | 6.19/11.0/13.0/18 | 32.75/59.0/71.0/101 | 5.29 | 4.19/11.0/12.0/17 | -0.5304 |

| profile | priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---|---:|---:|---:|---:|
| moving liftoff | invariant | 1.10/3.0/3 | 3.63/12.0/12 | 0.14/3.0/3 | 17 |
| moving liftoff | viability | 0.26/2.0/4 | 0.93/8.0/14 | 0.06/1.4/4 | 10 |
| moving liftoff | intent | 1.21/4.4/6 | 2.42/8.8/12 | 0.33/4.4/6 | 32 |
| moving liftoff | preference | 1.38/6.0/8 | 10.03/46.0/64 | 0.62/6.0/8 | 61 |
| moving liftoff | style | 1.00/1.0/1 | 10.67/12.0/12 | 0.02/1.0/1 | 6 |
| full transfer stress | invariant | 1.86/7.0/9 | 7.81/30.0/40 | 1.43/7.0/9 | 289 |
| full transfer stress | viability | 0.92/5.0/7 | 4.59/25.0/35 | 0.80/5.0/7 | 275 |
| full transfer stress | intent | 1.17/4.0/6 | 2.38/9.0/12 | 0.78/4.0/6 | 303 |
| full transfer stress | preference | 1.28/5.0/6 | 9.69/40.0/64 | 0.93/5.0/6 | 328 |
| full transfer stress | style | 0.96/4.0/5 | 8.28/27.0/38 | 0.25/3.0/5 | 115 |

### G1 execution over time

| profile | ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0–25 | 2322.5 | 2954.2 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 26–51 | 2318.2 | 2356.3 | 4.00 | 23.96 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 52–77 | 2316.1 | 2333.9 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 78–103 | 2326.8 | 2355.7 | 4.00 | 23.77 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 104–129 | 2319.1 | 2343.9 | 4.00 | 23.85 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 130–155 | 2323.3 | 2335.0 | 4.00 | 23.81 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 156–181 | 2330.4 | 2431.4 | 4.00 | 23.69 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 182–207 | 2267.9 | 2403.2 | 5.12 | 29.38 | 1.12 | 0.007 | 0.001 | 8.52e-10 | 5.16e-11 | 0 |
| moving liftoff | 208–233 | 1981.8 | 2828.0 | 7.31 | 40.42 | 3.65 | 0.036 | 0.010 | 9.34e-10 | 1.40e-11 | 0 |
| moving liftoff | 234–259 | 3536.8 | 4301.4 | 9.15 | 40.27 | 6.92 | 2.366 | 0.187 | 9.04e-10 | 5.60e-12 | 0 |
| full transfer stress | 0–59 | 2340.1 | 2540.2 | 4.00 | 23.92 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| full transfer stress | 60–119 | 2337.8 | 2396.7 | 4.00 | 23.82 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| full transfer stress | 120–179 | 2336.9 | 2404.6 | 4.00 | 23.70 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| full transfer stress | 180–239 | 2120.3 | 2549.4 | 6.25 | 34.18 | 2.45 | 0.122 | 0.009 | 1.24e-09 | 5.32e-11 | 0 |
| full transfer stress | 240–299 | 3663.2 | 86834.4 | 8.50 | 39.20 | 7.13 | 10.536 | 1.714 | 8.50e-10 | 4.83e-12 | 0 |
| full transfer stress | 300–359 | 6551.6 | 131209.6 | 9.63 | 48.77 | 8.85 | 46.506 | 7.361 | 6.50e-09 | 3.47e-10 | 25 |
| full transfer stress | 360–419 | 5635.5 | 204405.5 | 8.60 | 44.88 | 7.82 | 104.540 | 79.629 | 7.08e-09 | 1.66e-10 | 60 |
| full transfer stress | 420–479 | 5621.1 | 270596.9 | 8.50 | 45.35 | 7.85 | 122.368 | 111.524 | 2.96e-09 | 8.14e-11 | 12 |
| full transfer stress | 480–539 | 4993.5 | 343240.2 | 8.45 | 43.68 | 7.78 | 146.860 | 123.728 | 8.87e+02 | 2.01e-10 | 13 |
| full transfer stress | 540–599 | 293298.9 | 299421.5 | 0.00 | 0.00 | 0.00 | 173.055 | 122.090 | 8.87e+02 | 0.00e+00 | 60 |

### G1 artifact integrity

| profile | metrics SHA-256 | raw NPZ SHA-256 | directory |
|---|---|---|---|
| moving liftoff | `7d798198eccd3193dafecfcd2e15b80fdf60cf33d6977643138d78079ea00003` | `0071228b9a8589967db8674b94fce3965cf8d3344d40e10c1d44074903969e7e` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-liftoff-latest` |
| full transfer stress | `945621a3dff9235021f9a61199a51eb4e13dbcce720e1ac7ff61c0a47cc1767f` | `a786209ea8ddf69c2c68b8b3a92445c27af24d79a8320751e9bce78a55f1b6d8` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-transfer-latest` |

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
| end_effector_reach | Bonesaw | 40.6 | 1.7 | 1.0 | 42.0 | 43.2 | 46.3 | 66.2 | 6.1 |
| end_effector_reach | PlaCo | 100.1 | 12.6 | 2.7 | 107.2 | 115.4 | 165.8 | 213.3 | 52.5 |
| bimanual_priority_conflict | Bonesaw | 88.5 | 6.0 | 2.1 | 92.5 | 94.8 | 107.1 | 125.6 | 12.6 |
| bimanual_priority_conflict | PlaCo | 111.0 | 13.5 | 2.1 | 118.9 | 127.4 | 184.2 | 225.6 | 59.4 |
| walking_motion_retarget | Bonesaw | 64.7 | 4.1 | 2.7 | 69.6 | 70.5 | 75.2 | 87.1 | 11.2 |
| walking_motion_retarget | PlaCo | 133.7 | 16.0 | 2.7 | 144.0 | 157.6 | 221.8 | 244.4 | 73.6 |

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
| end_effector_reach | Bonesaw | 41.97 | 43.34 | 1.37 | 1.000 | 1.000 | 352 | 0 | 0 | 0 | 0.03 | 0 |
| end_effector_reach | PlaCo | 95.79 | 96.05 | 0.26 | 1.000 | 1.000 | 69 | 0 | 0 | 0 | 0.03 | 0 |
| bimanual_priority_conflict | Bonesaw | 46.07 | 47.44 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 3 | 0.03 | 0 |
| bimanual_priority_conflict | PlaCo | 99.56 | 99.82 | 0.26 | 1.000 | 1.000 | 68 | 0 | 0 | 1 | 0.03 | 0 |
| walking_motion_retarget | Bonesaw | 48.49 | 49.86 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 0 | 0.03 | 0 |
| walking_motion_retarget | PlaCo | 101.96 | 102.44 | 0.49 | 1.000 | 1.000 | 126 | 0 | 0 | 1 | 0.03 | 0 |

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
| 5,000 | 13 | 2 | 3 | 10.250 | 0.270 | 0.281 | 8.246 | yes |

## Compiled signal-to-task controller sentinel

This profile runs the complete explicit transition: vector and rotation inputs,
low-pass and SO(3) spring state, resolved point, CoM, and orientation task slots,
FK/Jacobians, strict hierarchy,
quintic synthesis, and next-state signal-memory commit. Two independently sized
state/scratch/output streams receive identical inputs.

| ticks | signal nodes | task slots | point RMS cm | orientation RMS deg | p50 us | p99 us | max us | bitwise repeat |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 6 | 3 | 0.9254 | 0.1587 | 74.2 | 100.6 | 125.4 | yes |

## Unified inverse-dynamics WBC sentinel

The CPU dynamic profiles solve one strict problem over `[q̈, τ, contact force]`.
Hard rows enforce the rigid-body equation and locked/rolling contact acceleration;
bounds and inequalities enforce acceleration, torque, unilateral normal load, and
a four-sided friction pyramid. The floating profile prepends six unactuated root
accelerations and enforces all six free-body equilibrium rows without a root torque.

| profile | ticks | variables | contacts | accel tracking RMS | max abs(q̈) | max abs(τ) | max abs(f) | dynamics max | contact accel max | min friction margin | friction active | min torque margin | p50 µs | p99 µs | max µs | infeasible | bitwise repeat |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 5,000 | 42 | 2 | 1.021e-01 | 2.500e-01 | 1.555e+02 | 2.356e+02 | 7.59e-14 | 5.21e-18 | -2.84e-14 | 4,674 | 9.84e+03 | 581.5 | 955.1 | 6794.0 | 0 | yes |
| floating | 5,000 | 48 | 2 | 1.058e-02 | 3.723e-02 | 7.838e-01 | 1.972e+02 | 5.14e-10 | 5.19e-11 | 1.57e+02 | 0 | 1.00e+04 | 630.2 | 659.3 | 837.6 | 0 | yes |

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
| projected adapter | 5,000 | 12.00 | 12.00 | 0.000 | 0.00 | 0.00 | 0.01 | 0.000 | 0.00 | 0.000 | 200.00 | 2.03 | 29.66 | 1.77e-11 | 5.19e-12 | -7.37e-15 | 1.67e+00 | 4,778 | 0 | 140.3 | 162.1 | 209.8 |
| raw integrated WBC | 5,000 | 12.00 | 12.00 | -0.000 | 0.54 | 0.52 | 0.58 | 0.006 | 1.73 | 0.000 | 2.02 | 2.40 | 26.41 | 2.19e-11 | 4.99e-12 | 2.07e+01 | 1.70e+00 | 0 | 0 | 200.2 | 224.3 | 244.6 |

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
| Official C++ adapter latency | p50 1.423 µs · p99 2.344 µs |
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
| end_effector_reach | 39.9 | 38.4 | 46.7 | 67.5 | 75.0 | 110.6 |
| bimanual_priority_conflict | 48.9 | 47.1 | 55.5 | 82.9 | 91.2 | 124.3 |
| walking_motion_retarget | 43.1 | 41.7 | 50.5 | 73.9 | 82.3 | 108.5 |

## Temporal drift by execution window

Each row is one tenth of a run. This exposes warm drift, allocator/GC episodes,
thermal/scheduler outliers, and tracking degradation hidden by one aggregate.

| Scenario | Impl | tick range | p50 µs | p99 µs | RMS cm | p99 error cm |
|---|---|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0–499 | 40.6 | 55.1 | 1.955 | 8.660 |
| end_effector_reach | Bonesaw | 500–999 | 40.2 | 45.3 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1000–1499 | 40.9 | 45.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1500–1999 | 40.2 | 45.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2000–2499 | 40.7 | 49.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2500–2999 | 40.3 | 45.1 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3000–3499 | 40.7 | 45.6 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3500–3999 | 40.4 | 45.6 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4000–4499 | 40.8 | 45.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4500–4999 | 40.3 | 45.4 | 0.129 | 0.133 |
| end_effector_reach | PlaCo | 0–499 | 97.0 | 160.2 | 1.290 | 5.696 |
| end_effector_reach | PlaCo | 500–999 | 97.5 | 165.8 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 1000–1499 | 96.1 | 166.9 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 1500–1999 | 97.6 | 130.7 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 2000–2499 | 96.9 | 162.3 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 2500–2999 | 95.8 | 168.1 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 3000–3499 | 96.9 | 165.7 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 3500–3999 | 94.6 | 121.3 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 4000–4499 | 96.2 | 162.5 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 4500–4999 | 96.1 | 166.9 | 0.731 | 0.798 |
| bimanual_priority_conflict | Bonesaw | 0–499 | 87.9 | 104.6 | 1.501 | 8.894 |
| bimanual_priority_conflict | Bonesaw | 500–999 | 88.9 | 116.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1000–1499 | 88.9 | 108.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1500–1999 | 88.3 | 106.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2000–2499 | 88.6 | 96.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2500–2999 | 88.4 | 96.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3000–3499 | 88.6 | 95.7 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3500–3999 | 88.8 | 95.3 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4000–4499 | 88.3 | 100.0 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4500–4999 | 88.7 | 97.4 | 0.000 | 0.000 |
| bimanual_priority_conflict | PlaCo | 0–499 | 106.2 | 186.0 | 0.869 | 0.286 |
| bimanual_priority_conflict | PlaCo | 500–999 | 105.7 | 180.5 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 1000–1499 | 107.0 | 143.9 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 1500–1999 | 105.6 | 182.0 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 2000–2499 | 106.6 | 172.9 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 2500–2999 | 107.2 | 181.0 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 3000–3499 | 108.7 | 186.7 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 3500–3999 | 107.3 | 185.5 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 4000–4499 | 106.0 | 181.5 | 0.250 | 0.253 |
| bimanual_priority_conflict | PlaCo | 4500–4999 | 107.5 | 185.5 | 0.250 | 0.253 |
| walking_motion_retarget | Bonesaw | 0–499 | 64.4 | 73.4 | 3.882 | 14.242 |
| walking_motion_retarget | Bonesaw | 500–999 | 65.6 | 73.3 | 4.075 | 14.573 |
| walking_motion_retarget | Bonesaw | 1000–1499 | 65.4 | 72.8 | 4.221 | 14.647 |
| walking_motion_retarget | Bonesaw | 1500–1999 | 64.7 | 72.3 | 3.937 | 14.271 |
| walking_motion_retarget | Bonesaw | 2000–2499 | 65.3 | 82.0 | 4.088 | 14.573 |
| walking_motion_retarget | Bonesaw | 2500–2999 | 65.3 | 73.0 | 4.186 | 14.753 |
| walking_motion_retarget | Bonesaw | 3000–3499 | 64.6 | 72.7 | 3.859 | 14.187 |
| walking_motion_retarget | Bonesaw | 3500–3999 | 64.9 | 79.8 | 3.997 | 14.366 |
| walking_motion_retarget | Bonesaw | 4000–4499 | 65.3 | 81.7 | 4.284 | 14.839 |
| walking_motion_retarget | Bonesaw | 4500–4999 | 64.9 | 73.8 | 3.929 | 14.187 |
| walking_motion_retarget | PlaCo | 0–499 | 128.7 | 214.9 | 2.546 | 11.321 |
| walking_motion_retarget | PlaCo | 500–999 | 129.0 | 227.7 | 2.626 | 11.387 |
| walking_motion_retarget | PlaCo | 1000–1499 | 128.7 | 218.5 | 2.754 | 11.416 |
| walking_motion_retarget | PlaCo | 1500–1999 | 127.3 | 221.7 | 2.590 | 11.331 |
| walking_motion_retarget | PlaCo | 2000–2499 | 128.1 | 226.3 | 2.643 | 11.399 |
| walking_motion_retarget | PlaCo | 2500–2999 | 128.0 | 218.8 | 2.714 | 11.398 |
| walking_motion_retarget | PlaCo | 3000–3499 | 128.3 | 219.1 | 2.504 | 11.290 |
| walking_motion_retarget | PlaCo | 3500–3999 | 129.6 | 215.1 | 2.593 | 11.381 |
| walking_motion_retarget | PlaCo | 4000–4499 | 131.0 | 188.4 | 2.801 | 11.398 |
| walking_motion_retarget | PlaCo | 4500–4999 | 127.8 | 220.5 | 2.554 | 11.290 |

## Process startup and resident footprint

| Implementation | import ms | setup ms | baseline RSS MB | after import MB | after setup MB | import Δ MB | setup Δ MB |
|---|---:|---:|---:|---:|---:|---:|---:|
| Bonesaw | 0.91 | 0.48 | 39.42 | 40.08 | 40.91 | 0.66 | 0.82 |
| PlaCo | 54.05 | 6.83 | 39.42 | 88.52 | 101.82 | 49.10 | 13.30 |

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
