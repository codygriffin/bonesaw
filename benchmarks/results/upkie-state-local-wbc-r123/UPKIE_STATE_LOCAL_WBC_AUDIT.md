# Bonesaw Upkie state-local rolling WBC · r123

## Outcome

> **Admission: PASS.** This is the missing composed Upkie WBC test: two nonholonomic wheel contacts, floating rigid-body dynamics, contact forces, friction, and actuator effort are solved together in Rust and rebuilt independently in Pinocchio. Every corpus row is an immutable state query—policy = null, physics rollout = null.

## Corpus and semantic boundary

| item | value |
|---|---|
| states | 256 |
| model | /home/codygriffin/Documents/bonepilot/models/upkie/upkie.urdf |
| model SHA-256 | d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483 |
| coordinates | ['left_hip', 'left_knee', 'left_wheel', 'right_hip', 'right_knee', 'right_wheel'] |
| contact frames | ['left_wheel_center', 'right_wheel_center'] |
| rolling coefficients | [-0.05, 0.05] |
| policy | none |
| physics / integration | none |
| state carried between rows | none |
| decision vector | [floating qdd, actuator torque, two 3D contact forces] |

The posture, root velocity, and requested acceleration vary analytically across the corpus, but rows are not interpreted as a trajectory. Wheel rates are authored to satisfy v_center,x + c·qdot_wheel = 0 exactly. This isolates WBC admission from state estimation, balance policy, motor policy, and simulation.

## Example authority stack

| layer | authority | separate witness |
|---|---|---|
| Invariant | floating dynamics | Pinocchio M qdd + h − Sᵀτ − Jᵀf L∞ |
| Invariant | two RollingWheel contacts | longitudinal wheel coupling + lateral/normal acceleration L∞ |
| Resource | unilateral + square Coulomb cone | normal force and friction margin per wheel |
| Resource | URDF actuator effort | per-actuator effort margin and limiting actuator |
| Viability | root attitude + height | priority-0 task residual; hard contacts remain above it |
| Intent | root horizontal | priority-1 residual |
| Posture | joint/wheel acceleration witness | priority-2 residual and witness RMS |
| Recovery | bounded solve status | SolvedWithSlack / MaxIterations / infeasible stay typed |
| Budget | CPU exact f64 | per-query latency, work counters, allocations, deadline misses |

## Independent Pinocchio hard-equation oracle

| quantity | worst observed | gate |
|---|---|---|
| floating dynamics L∞ | 1.974e-11 | ≤ 2e-6 |
| rolling/lateral/normal contact L∞ | 4.613e-12 | ≤ 2e-6 |
| minimum normal force | 24.902577 N | ≥ -1e-8 |
| minimum square friction margin | 0.000000e+00 N | ≥ -2e-7 |
| minimum effort margin | 1.661657e+00 Nm | ≥ -2e-7 |
| Rust-reported dynamics L∞ | 1.974e-11 | diagnostic |
| Rust-reported contact L∞ | 4.618e-12 | diagnostic |

Pinocchio rebuilds mass, nonlinear effects, wheel-center Jacobians, and classical J-dot-v acceleration from the URDF. The gate uses returned Rust qdd, torque, and force—not Rust's residual fields—as its witness.

## Tracking error

| group | RMS | abs p50 | abs p95 | abs p99 | abs max |
|---|---|---|---|---|---|
| all_generalized_mps2_or_radps2 | 100.442860 | 1.369423 | 270.146718 | 271.921633 | 272.000000 |
| root_linear_mps2 | 0.723767 | 0.000006 | 2.167433 | 2.895372 | 2.932954 |
| joint_radps2 | 142.046733 | 34.259856 | 271.532332 | 271.980953 | 272.000000 |
| wheel_radps2 | 242.688023 | 262.570474 | 271.946376 | 271.998068 | 272.000000 |

Tracking error is not a feasibility failure. It is the continuous distance between the requested acceleration witness and the best command remaining after hard dynamics, rolling contact, friction, effort, and higher authority are honored.

### Task and nullspace residual stack

| slot | task | clipped | mean RMS | p95 | p99 | max |
|---|---|---|---|---|---|---|
| 0 | root_angular | 0 | 0.00001 | 0.00004 | 0.00016 | 0.00040 |
| 1 | root_horizontal | 166 | 0.57053 | 2.00954 | 2.07068 | 2.07391 |
| 2 | root_height | 0 | 0.00002 | 0.00008 | 0.00018 | 0.00033 |
| 3 | joint_posture | 113 | 137.35249 | 159.60947 | 159.99799 | 160.02399 |
| 7 | contact_force_regularization | 56 | 4.96748 | 6.65643 | 13.34631 | 14.60834 |
| 8 | actuator_torque_regularization | 56 | 0.59155 | 0.73183 | 0.73810 | 0.73838 |

The task stack stays expanded rather than collapsed into one score. This table shows the six active fixed slots; inactive override, CoM, momentum, swing, and point slots are omitted. Root and posture layers show exactly where the requested acceleration was surrendered.

### Acceleration-bound authority

| measurement | result |
|---|---|
| configured absolute qdd cap | 250.0 |
| maximum absolute qdd | 250.000000 |
| maximum utilization | 100.00% |
| minimum solver bound margin | 0.000e+00 |
| queries touching qdd cap | 133 / 256 |

A query can satisfy dynamics and contact exactly while landing on the configured acceleration cap. That is explicitly reported as exhausted numerical/kinematic authority, not mislabeled as successful pose tracking.

## CPU latency, jitter, memory, and allocations

| measurement | result |
|---|---|
| latency mean / std / MAD | 121.8 / 23.8 / 2.8 µs |
| latency p50 / p95 / p99 / p99.9 / max | 118.3 / 130.6 / 322.4 / 335.0 / 341.2 µs |
| absolute adjacent-query jitter p50 / p95 / p99 / max | 2.8 / 23.2 / 212.7 / 222.6 µs |
| p99 − p50 | 204.1 µs |
| deadline misses | {'0.5ms': 0, '1ms': 0, '2ms': 0, '5ms': 0, '10ms': 0} |
| whole-call wall / CPU | 249.928 / 249.847 ms |
| CPU / wall | 0.9997 |
| measured throughput | 8194.4 queries/s |
| current RSS before / after / delta | 86.30 / 86.37 / 0.06 MiB |
| process peak RSS before / after | 86.04 / 86.04 MiB |
| hot-loop allocation sentinel | 0 calls / 0 bytes |
| GC collections during measured call | 0 |

## Execution order windows

| rows | p50 / p99 / max µs | tracking RMS | dyn L∞ | contact L∞ | effort | slack |
|---|---|---|---|---|---|---|
| 0–31 | 119.7 / 275.7 / 341.2 | 112.48744 | 7.85e-12 | 1.77e-12 | 5.2% | 32 |
| 32–63 | 118.1 / 128.6 / 129.5 | 104.19855 | 6.65e-12 | 2.88e-12 | 7.4% | 29 |
| 64–95 | 117.3 / 269.4 / 331.4 | 89.68800 | 8.37e-12 | 3.65e-12 | 6.9% | 17 |
| 96–127 | 121.0 / 154.0 / 163.9 | 112.09895 | 9.13e-12 | 1.92e-12 | 6.1% | 32 |
| 128–159 | 116.1 / 126.1 / 127.6 | 84.32052 | 1.52e-11 | 4.61e-12 | 7.6% | 13 |
| 160–191 | 114.9 / 257.1 / 317.8 | 89.26590 | 1.90e-11 | 4.10e-12 | 7.9% | 9 |
| 192–223 | 115.9 / 124.8 / 126.1 | 94.79566 | 1.97e-11 | 1.74e-12 | 7.6% | 2 |
| 224–255 | 118.4 / 270.8 / 334.5 | 111.91382 | 7.88e-12 | 3.80e-12 | 7.2% | 32 |

These windows expose drift and outliers over call order; their x-axis is corpus row order, not simulated time.

### Solver work attribution

| counter | mean | p95 | p99 | max | latency corr. |
|---|---|---|---|---|---|
| task_pseudoinverse_calls | 5.01 | 6.00 | 6.00 | 6.00 | 0.0735 |
| clipped_steps | 2.06 | 4.00 | 5.00 | 5.00 | 0.0938 |
| task_jacobi_sweeps | 19.42 | 21.00 | 25.45 | 28.00 | 0.1019 |
| feasibility_projection_sweeps | 1.00 | 1.00 | 1.00 | 1.00 | 0.0000 |
| feasibility_halfspace_projections | 84.00 | 84.00 | 84.00 | 84.00 | 0.0000 |
| feasibility_polish_iterations | 0.00 | 0.00 | 0.00 | 0.00 | 0.0000 |

## Continuous authority sweeps

| friction | torque cap | statuses | tracking max RMS | friction margin | effort use | hard violation |
|---|---|---|---|---|---|---|
| 0.8 | 2000.0 | {'Solved': 90, 'SolvedWithSlack': 166} | 113.1567 | 0.000e+00 | 7.9% | 1.97e-11 |
| 0.4 | 2000.0 | {'Solved': 84, 'SolvedWithSlack': 172} | 113.1567 | -1.776e-15 | 7.9% | 1.97e-11 |
| 0.2 | 2000.0 | {'Solved': 25, 'SolvedWithSlack': 231} | 90.4806 | -8.882e-16 | 7.7% | 1.97e-11 |
| 0.05 | 2000.0 | {'Solved': 4, 'SolvedWithSlack': 116, 'MaxIterations': 136} | 41.5813 | -2.220e-16 | 8.1% | 8.14e-01 |
| 0.8 | 8.0 | {'Solved': 90, 'SolvedWithSlack': 166} | 113.1567 | 0.000e+00 | 15.9% | 1.97e-11 |
| 0.8 | 4.0 | {'Solved': 90, 'SolvedWithSlack': 166} | 113.1567 | 0.000e+00 | 31.7% | 1.97e-11 |
| 0.8 | 1.5 | {'Solved': 90, 'SolvedWithSlack': 166} | 113.1567 | 0.000e+00 | 84.7% | 1.97e-11 |
| 0.8 | 0.5 | {'SolvedWithSlack': 256} | 113.3653 | -3.553e-15 | 100.0% | 1.19e-10 |

Lower friction and actuator caps do not change the meaning of the query. They show how task residual rises—or a bounded solve becomes typed—while hard physical witnesses remain separately inspectable.

## Rolling slip stabilization and fault typing

| check | result |
|---|---|
| slip_range_mps | [-0.8, 0.8] |
| maximum_expected_stabilization_mps2 | 1.0 |
| pinocchio_stabilized_contact_linf | 2.4709123636057484e-12 |
| status_counts | {'Solved': 51, 'SolvedWithSlack': 77} |
| all_outputs_finite | True |
| zero_rolling_coefficient_rejected | True |
| allocation_calls | 0 |

## Determinism and isolation from execution layout

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| reverse_permutation_exact | True |
| two_chunk_exact | True |
| repeat_allocation_calls | 0 |
| permutation_allocation_calls | 0 |
| chunk_allocation_calls | 0 |

## Retained official Upkie reference comparison

| item | official Upkie C++ | Bonesaw Rust official profile |
|---|---|---|
| shared controller-law samples | 100000 | 100000 |
| wheel command mismatches | reference | 0 |
| maximum command error | reference | 0.000e+00 rad/s |
| p50 latency | 1.393 µs | 0.030 µs |
| p99 latency | 2.385 µs | 0.031 µs |
| peak RSS | 8.64 MiB | 8.68 MiB |
| hot-loop allocations | not instrumented upstream | 0 calls |

The retained comparison pins upstream WheelBalancer.cpp and proves exact command-law parity over 100,000 sequential samples. It is deliberately not presented as a WBC speed comparison: Upkie's measured boundary is dictionary I/O plus a scalar balance law; this report measures the full floating inverse-dynamics hierarchy.

## Deliberate remaining boundaries

This admission does not prove closed-loop stability, estimator robustness, motor tracking, thermal/power reliability, terrain transitions, or contact switching. Those require separate evidence. It does prove that the CPU implementation composes the Upkie morphology's rolling constraints with floating dynamics and bounded resources without hiding tracking loss inside a binary pose claim. CUDA batching remains deferred until this CPU semantic reference is stable.
