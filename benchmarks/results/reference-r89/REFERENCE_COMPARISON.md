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
| end_effector_reach | Bonesaw | 0.630 | 0.129 | 35.5 | 40.5 | 46.6 | 57.2 | 0 | 27640 |
| end_effector_reach | PlaCo | 0.809 | 0.737 | 97.5 | 108.1 | 122.7 | 242.2 | 0 | 9547 |
| bimanual_priority_conflict | Bonesaw | 0.475 | 0.000 | 71.9 | 83.4 | 91.4 | 156.9 | 0 | 13717 |
| bimanual_priority_conflict | PlaCo | 0.365 | 0.252 | 108.6 | 118.4 | 128.6 | 320.9 | 0 | 8628 |
| walking_motion_retarget | Bonesaw | 4.048 | 4.062 | 55.6 | 67.1 | 73.5 | 83.8 | 0 | 17802 |
| walking_motion_retarget | PlaCo | 2.634 | 2.645 | 131.1 | 143.0 | 152.8 | 271.1 | 0 | 7226 |

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
| moving liftoff | PASS | PASS | 260 | 260 | 0.777 | 0.000 | 0.355 | 3.135 | 8.000 | 199/61/0 | 0/0/0/0 | 1.22e-09 | 5.87e-11 |
| full transfer stress | FAIL | FAIL | 600 | 351 | 46.420 | 25.665 | 32.730 | 32.481 | 8.000 | 199/152/0 | 249/0/0/0 | 3.04e+02 | 4.39e+01 |

The moving-liftoff row passes both the behavioral and unchanged 5 ms p99 CPU gates.
The transfer row remains a deliberately retained failing stress case. Rejected ticks
retain raw residuals and state rather than being omitted from aggregates.

### G1 latency, jitter, and deadlines

| profile | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs | >1 ms | >5 ms | >20 ms | ticks/s |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 2879.4 | 385.1 | 40.6 | 2970.6 | 3339.1 | 3738.0 | 4446.8 | 4596.7 | 4613.3 | 1076.6 | 260 | 0 | 0 | 347.3 |
| full transfer stress | 110403.4 | 122175.9 | 3200.8 | 4883.8 | 271820.6 | 276773.8 | 328433.0 | 371391.6 | 376164.7 | 50939.0 | 600 | 300 | 281 | 9.1 |

### G1 process resources

Buffers and the Rust session are allocated before this measurement. Python
`tracemalloc` excludes native allocation; zero hot-loop allocation is gated
separately by the native Rust sentinels later in this report.

| profile | wall s | process CPU s | thread CPU s | process CPU/wall | RSS before MB | RSS after MB | RSS delta MB | peak RSS MB | Python trace peak MB | GC collections | minor/major faults | voluntary/involuntary ctx |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0.749 | 0.749 | 0.749 | 1.000 | 52.64 | 52.96 | 0.32 | 52.96 | 0.00 | 0 | 50/0 | 0/4 |
| full transfer stress | 66.242 | 66.229 | 66.229 | 1.000 | 52.89 | 53.56 | 0.68 | 53.56 | 0.00 | 0 | 178/0 | 2/605 |

### G1 latency by solver/contact status

| profile | status | ticks | p50 µs | p95 µs | p99 µs | max µs |
|---|---|---:|---:|---:|---:|---:|
| moving liftoff | solved | 199 | 2984.8 | 3043.3 | 3056.6 | 3072.1 |
| moving liftoff | solved_with_slack | 61 | 2234.7 | 3698.8 | 4227.6 | 4613.3 |
| full transfer stress | solved | 199 | 3018.5 | 3085.2 | 3110.3 | 3148.6 |
| full transfer stress | solved_with_slack | 152 | 3554.0 | 92335.1 | 98998.6 | 100508.8 |
| full transfer stress | normal_contact_contingency | 249 | 270424.0 | 273950.9 | 290017.2 | 376164.7 |

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
| moving liftoff | 4.82/9.0/11.8/13 | 32.46/51.0/79.2/90 | 6.73 | 1.21/7.0/10.4/11 | -0.1970 |
| full transfer stress | 3.43/10.0/13.0/17 | 21.83/57.0/80.0/107 | 6.36 | 1.83/9.0/11.0/17 | -0.7856 |

| profile | priority | pseudoinverse mean/p99/max | Jacobi sweeps mean/p99/max | clipped-step mean/p99/max | ticks clipped |
|---|---|---:|---:|---:|---:|
| moving liftoff | invariant | 1.09/3.0/3 | 3.60/12.0/12 | 0.14/3.0/3 | 17 |
| moving liftoff | viability | 0.24/1.0/3 | 0.86/4.0/11 | 0.05/1.0/3 | 11 |
| moving liftoff | intent | 1.21/4.4/9 | 2.42/8.8/18 | 0.33/4.4/9 | 33 |
| moving liftoff | preference | 1.28/5.8/7 | 10.48/56.2/72 | 0.51/5.8/7 | 61 |
| moving liftoff | style | 1.00/1.0/1 | 15.10/17.0/17 | 0.17/1.0/1 | 44 |
| full transfer stress | invariant | 0.93/5.0/10 | 3.58/20.0/50 | 0.51/5.0/10 | 108 |
| full transfer stress | viability | 0.36/4.0/5 | 1.47/15.0/24 | 0.27/4.0/5 | 99 |
| full transfer stress | intent | 0.75/4.0/10 | 1.54/8.0/30 | 0.37/4.0/10 | 124 |
| full transfer stress | preference | 0.78/5.0/8 | 6.75/40.1/82 | 0.45/5.0/8 | 152 |
| full transfer stress | style | 0.61/2.0/4 | 8.48/22.0/42 | 0.23/2.0/3 | 125 |

### G1 execution over time

| profile | ticks | p50 µs | p99 µs | pseudoinverse mean | Jacobi-sweep mean | clipped-step mean | root RMS cm | foot RMS cm | dynamics max | contact max | contingency/rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| moving liftoff | 0–25 | 2990.3 | 3051.1 | 4.00 | 28.31 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| moving liftoff | 26–51 | 2934.1 | 3039.4 | 4.00 | 28.12 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| moving liftoff | 52–77 | 2996.6 | 3051.0 | 4.00 | 28.31 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| moving liftoff | 78–103 | 2984.4 | 3049.8 | 4.00 | 28.27 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 104–129 | 2976.3 | 3030.2 | 4.00 | 28.31 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 130–155 | 2961.7 | 3047.1 | 4.00 | 28.23 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.02e-11 | 0 |
| moving liftoff | 156–181 | 3006.3 | 3070.6 | 4.00 | 28.81 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| moving liftoff | 182–207 | 2940.2 | 3039.8 | 4.81 | 34.00 | 0.85 | 0.007 | 0.001 | 8.04e-10 | 5.87e-11 | 0 |
| moving liftoff | 208–233 | 2198.4 | 3241.9 | 7.27 | 50.19 | 4.19 | 0.042 | 0.010 | 1.14e-09 | 1.45e-11 | 0 |
| moving liftoff | 234–259 | 3228.7 | 4452.6 | 8.15 | 42.08 | 7.04 | 2.456 | 0.384 | 4.12e-10 | 8.40e-12 | 0 |
| full transfer stress | 0–59 | 3020.7 | 3124.5 | 4.00 | 28.25 | 0.00 | 0.000 | 0.000 | 1.21e-09 | 3.03e-11 | 0 |
| full transfer stress | 60–119 | 3022.7 | 3124.3 | 4.00 | 28.28 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| full transfer stress | 120–179 | 3015.2 | 3097.4 | 4.00 | 28.45 | 0.00 | 0.000 | 0.000 | 1.22e-09 | 3.03e-11 | 0 |
| full transfer stress | 180–239 | 2381.6 | 3179.6 | 6.12 | 41.40 | 2.73 | 0.145 | 0.009 | 1.14e-09 | 5.87e-11 | 0 |
| full transfer stress | 240–299 | 3520.6 | 4843.2 | 8.27 | 45.37 | 7.78 | 10.474 | 2.662 | 4.12e-10 | 1.14e-11 | 0 |
| full transfer stress | 300–359 | 62619.8 | 210358.4 | 7.93 | 46.57 | 7.82 | 52.714 | 25.910 | 3.04e+02 | 4.39e+01 | 9 |
| full transfer stress | 360–419 | 209507.3 | 212865.3 | 0.00 | 0.00 | 0.00 | 75.038 | 37.404 | 3.04e+02 | 4.39e+01 | 60 |
| full transfer stress | 420–479 | 268042.2 | 329150.2 | 0.00 | 0.00 | 0.00 | 69.097 | 39.754 | 3.04e+02 | 4.39e+01 | 60 |
| full transfer stress | 480–539 | 271178.2 | 280931.5 | 0.00 | 0.00 | 0.00 | 64.032 | 42.501 | 3.04e+02 | 4.39e+01 | 60 |
| full transfer stress | 540–599 | 270715.9 | 276389.0 | 0.00 | 0.00 | 0.00 | 64.454 | 49.877 | 3.04e+02 | 4.39e+01 | 60 |

### G1 artifact integrity

| profile | metrics SHA-256 | raw NPZ SHA-256 | directory |
|---|---|---|---|
| moving liftoff | `bdcff63ad0b5ea6cf3322eb1137ba001ac3e420e8c0daf1fbd8586d742569db9` | `a9a39e7889807c2b18853874b7fe331caf97150760106b353eea5a6cd4f43378` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-liftoff-latest` |
| full transfer stress | `d035db90bcbb40a271cec5b3a90f9ea54b2e2edd67b74a6d9a7254fcdafcc21a` | `800bdd60b56644dc55aa25690fddd8c0f3afb0c499273270c5684a6c129a997a` | `/home/codygriffin/Documents/bonepilot/benchmarks/results/floating-g1-transfer-latest` |

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
| end_effector_reach | Bonesaw | 35.6 | 1.4 | 0.7 | 36.7 | 38.2 | 40.5 | 56.3 | 5.5 |
| end_effector_reach | PlaCo | 98.3 | 3.5 | 1.1 | 102.4 | 104.0 | 108.1 | 189.1 | 12.2 |
| bimanual_priority_conflict | Bonesaw | 72.3 | 4.6 | 1.8 | 76.3 | 78.6 | 83.4 | 127.8 | 11.2 |
| bimanual_priority_conflict | PlaCo | 109.5 | 4.5 | 1.0 | 113.6 | 115.1 | 118.4 | 282.0 | 11.6 |
| walking_motion_retarget | Bonesaw | 55.6 | 3.7 | 2.2 | 59.6 | 61.4 | 67.1 | 81.1 | 9.5 |
| walking_motion_retarget | PlaCo | 131.9 | 3.9 | 1.6 | 136.5 | 138.4 | 143.0 | 216.2 | 12.8 |

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
| end_effector_reach | Bonesaw | 42.45 | 43.82 | 1.37 | 1.000 | 1.000 | 352 | 0 | 0 | 0 | 0.03 | 0 |
| end_effector_reach | PlaCo | 95.68 | 95.94 | 0.26 | 1.000 | 1.000 | 69 | 0 | 0 | 3 | 0.03 | 0 |
| bimanual_priority_conflict | Bonesaw | 46.57 | 47.94 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 4 | 0.03 | 0 |
| bimanual_priority_conflict | PlaCo | 99.45 | 99.71 | 0.26 | 0.999 | 1.000 | 68 | 0 | 0 | 5 | 0.03 | 0 |
| walking_motion_retarget | Bonesaw | 48.99 | 50.36 | 1.37 | 1.000 | 1.000 | 350 | 0 | 0 | 1 | 0.03 | 0 |
| walking_motion_retarget | PlaCo | 101.85 | 102.34 | 0.50 | 1.000 | 1.000 | 128 | 0 | 0 | 4 | 0.03 | 0 |

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
| 5,000 | 13 | 2 | 3 | 10.250 | 0.261 | 0.331 | 2.545 | yes |

## Compiled signal-to-task controller sentinel

This profile runs the complete explicit transition: vector and rotation inputs,
low-pass and SO(3) spring state, resolved point, CoM, and orientation task slots,
FK/Jacobians, strict hierarchy,
quintic synthesis, and next-state signal-memory commit. Two independently sized
state/scratch/output streams receive identical inputs.

| ticks | signal nodes | task slots | point RMS cm | orientation RMS deg | p50 us | p99 us | max us | bitwise repeat |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 5,000 | 6 | 3 | 0.9254 | 0.1587 | 68.3 | 89.8 | 115.5 | yes |

## Unified inverse-dynamics WBC sentinel

The CPU dynamic profiles solve one strict problem over `[q̈, τ, contact force]`.
Hard rows enforce the rigid-body equation and locked/rolling contact acceleration;
bounds and inequalities enforce acceleration, torque, unilateral normal load, and
a four-sided friction pyramid. The floating profile prepends six unactuated root
accelerations and enforces all six free-body equilibrium rows without a root torque.

| profile | ticks | variables | contacts | accel tracking RMS | max abs(q̈) | max abs(τ) | max abs(f) | dynamics max | contact accel max | min friction margin | friction active | min torque margin | p50 µs | p99 µs | max µs | infeasible | bitwise repeat |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| fixed | 5,000 | 42 | 2 | 1.021e-01 | 2.500e-01 | 1.555e+02 | 2.356e+02 | 7.59e-14 | 5.21e-18 | -2.84e-14 | 4,674 | 9.84e+03 | 513.0 | 533.5 | 709.7 | 0 | yes |
| floating | 5,000 | 48 | 2 | 1.058e-02 | 3.723e-02 | 7.838e-01 | 1.972e+02 | 5.14e-10 | 5.19e-11 | 1.57e+02 | 0 | 1.00e+04 | 764.0 | 877.0 | 889.3 | 0 | yes |

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
| projected adapter | 5,000 | 12.00 | 12.00 | 0.000 | 0.00 | 0.00 | 0.01 | 0.000 | 0.00 | 0.000 | 200.00 | 2.03 | 29.64 | 1.77e-11 | 5.20e-12 | -7.46e-15 | 1.67e+00 | 4,893 | 0 | 132.1 | 164.5 | 317.0 |
| raw integrated WBC | 5,000 | 12.00 | 12.00 | -0.000 | 0.54 | 0.52 | 0.58 | 0.006 | 1.73 | 0.000 | 2.02 | 2.40 | 26.41 | 2.19e-11 | 5.32e-12 | 2.03e+01 | 1.70e+00 | 0 | 0 | 190.9 | 263.7 | 292.4 |

The raw policy contains 4 signal nodes and 3 resolved floating task slots. Its emitted root/CoM acceleration differs from the direct formula oracle by at most `5.551e-17`; the full IK → signals → task emission → WBC → SE(3) integration loop reports zero allocator calls and bytes per step.

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
| Official C++ adapter latency | p50 1.393 µs · p99 2.385 µs |
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
| end_effector_reach | 38.6 | 38.3 | 42.5 | 45.4 | 52.7 | 116.2 |
| bimanual_priority_conflict | 48.0 | 47.6 | 52.3 | 54.7 | 59.8 | 125.2 |
| walking_motion_retarget | 41.8 | 41.4 | 45.7 | 48.6 | 54.6 | 121.7 |

## Temporal drift by execution window

Each row is one tenth of a run. This exposes warm drift, allocator/GC episodes,
thermal/scheduler outliers, and tracking degradation hidden by one aggregate.

| Scenario | Impl | tick range | p50 µs | p99 µs | RMS cm | p99 error cm |
|---|---|---:|---:|---:|---:|---:|
| end_effector_reach | Bonesaw | 0–499 | 35.6 | 46.7 | 1.955 | 8.660 |
| end_effector_reach | Bonesaw | 500–999 | 35.2 | 40.1 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1000–1499 | 35.6 | 40.1 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 1500–1999 | 35.4 | 40.5 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2000–2499 | 35.5 | 39.9 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 2500–2999 | 35.4 | 40.4 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3000–3499 | 35.6 | 40.8 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 3500–3999 | 35.5 | 40.7 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4000–4499 | 35.5 | 39.8 | 0.129 | 0.133 |
| end_effector_reach | Bonesaw | 4500–4999 | 35.3 | 39.9 | 0.129 | 0.133 |
| end_effector_reach | PlaCo | 0–499 | 98.1 | 108.6 | 1.290 | 5.696 |
| end_effector_reach | PlaCo | 500–999 | 98.0 | 106.1 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 1000–1499 | 97.7 | 108.2 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 1500–1999 | 97.1 | 109.7 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 2000–2499 | 97.3 | 107.1 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 2500–2999 | 97.4 | 110.2 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 3000–3499 | 97.4 | 107.3 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 3500–3999 | 97.2 | 106.4 | 0.731 | 0.798 |
| end_effector_reach | PlaCo | 4000–4499 | 97.4 | 106.8 | 0.743 | 0.798 |
| end_effector_reach | PlaCo | 4500–4999 | 97.1 | 104.9 | 0.731 | 0.798 |
| bimanual_priority_conflict | Bonesaw | 0–499 | 71.0 | 87.4 | 1.501 | 8.894 |
| bimanual_priority_conflict | Bonesaw | 500–999 | 72.9 | 84.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1000–1499 | 72.2 | 81.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 1500–1999 | 71.6 | 83.3 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2000–2499 | 72.0 | 82.3 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 2500–2999 | 71.8 | 81.5 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3000–3499 | 72.1 | 79.9 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 3500–3999 | 72.1 | 80.1 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4000–4499 | 71.9 | 83.6 | 0.000 | 0.000 |
| bimanual_priority_conflict | Bonesaw | 4500–4999 | 73.0 | 82.8 | 0.000 | 0.000 |
| bimanual_priority_conflict | PlaCo | 0–499 | 108.9 | 122.9 | 0.870 | 0.286 |
| bimanual_priority_conflict | PlaCo | 500–999 | 108.9 | 118.9 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 1000–1499 | 108.7 | 119.4 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 1500–1999 | 108.4 | 120.1 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 2000–2499 | 108.3 | 116.9 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 2500–2999 | 108.6 | 116.4 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 3000–3499 | 108.3 | 116.1 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 3500–3999 | 108.7 | 117.3 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 4000–4499 | 108.5 | 116.8 | 0.252 | 0.256 |
| bimanual_priority_conflict | PlaCo | 4500–4999 | 108.6 | 117.5 | 0.252 | 0.256 |
| walking_motion_retarget | Bonesaw | 0–499 | 55.3 | 67.4 | 3.882 | 14.242 |
| walking_motion_retarget | Bonesaw | 500–999 | 56.1 | 70.9 | 4.075 | 14.573 |
| walking_motion_retarget | Bonesaw | 1000–1499 | 55.7 | 62.6 | 4.221 | 14.647 |
| walking_motion_retarget | Bonesaw | 1500–1999 | 55.2 | 63.2 | 3.937 | 14.271 |
| walking_motion_retarget | Bonesaw | 2000–2499 | 55.5 | 61.8 | 4.088 | 14.573 |
| walking_motion_retarget | Bonesaw | 2500–2999 | 56.2 | 70.4 | 4.186 | 14.753 |
| walking_motion_retarget | Bonesaw | 3000–3499 | 55.3 | 65.6 | 3.859 | 14.187 |
| walking_motion_retarget | Bonesaw | 3500–3999 | 55.3 | 61.9 | 3.997 | 14.366 |
| walking_motion_retarget | Bonesaw | 4000–4499 | 55.5 | 63.6 | 4.284 | 14.839 |
| walking_motion_retarget | Bonesaw | 4500–4999 | 55.5 | 62.7 | 3.929 | 14.187 |
| walking_motion_retarget | PlaCo | 0–499 | 131.6 | 145.1 | 2.546 | 11.321 |
| walking_motion_retarget | PlaCo | 500–999 | 132.4 | 148.0 | 2.626 | 11.387 |
| walking_motion_retarget | PlaCo | 1000–1499 | 131.6 | 144.2 | 2.754 | 11.416 |
| walking_motion_retarget | PlaCo | 1500–1999 | 130.4 | 139.9 | 2.590 | 11.331 |
| walking_motion_retarget | PlaCo | 2000–2499 | 131.0 | 139.3 | 2.643 | 11.399 |
| walking_motion_retarget | PlaCo | 2500–2999 | 131.3 | 141.8 | 2.714 | 11.398 |
| walking_motion_retarget | PlaCo | 3000–3499 | 130.5 | 141.4 | 2.504 | 11.290 |
| walking_motion_retarget | PlaCo | 3500–3999 | 130.5 | 139.7 | 2.593 | 11.381 |
| walking_motion_retarget | PlaCo | 4000–4499 | 130.7 | 142.6 | 2.801 | 11.398 |
| walking_motion_retarget | PlaCo | 4500–4999 | 130.3 | 139.7 | 2.554 | 11.290 |

## Process startup and resident footprint

| Implementation | import ms | setup ms | baseline RSS MB | after import MB | after setup MB | import Δ MB | setup Δ MB |
|---|---:|---:|---:|---:|---:|---:|---:|
| Bonesaw | 0.91 | 0.52 | 39.36 | 40.35 | 41.39 | 0.99 | 1.04 |
| PlaCo | 52.04 | 6.56 | 39.33 | 88.43 | 101.71 | 49.11 | 13.27 |

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
