# Bonesaw CPU-exact dynamic batch · r82

## Outcome

**Admission: PASS.** The fixed Rust batch decision vector is now `[qdd, generalized joint effort, point-contact force]`. It enforces floating rigid-body dynamics, locked-point acceleration, actuator effort bounds, unilateral normal force, and Coulomb friction before optimizing emitted point tasks. Python supplies evaluation and an independent Pinocchio/NumPy hard-equation oracle; no policy or physics rollout is used.

## Example authority stack

This is the concrete architectural review stack. Rows are ordered by authority but their witnesses are never collapsed into one score.

| layer | example authority | separate witness |
|---|---|---|
| Invariant | contact locks 7951 / 7952 | hard 3-axis point acceleration equalities |
| Invariant | floating dynamics | M qdd + h = Sᵀ effort + Jᶜᵀ force |
| Viability | torso point 7901 + r81 stopping box | protected task residual and joint-limit authority remain separate |
| Intent | handle point 7902 | yields before Viability and retains its own residual |
| Contact resource | normal-force and Coulomb-cone margins | continuous force authority; finite only for declared point contacts |
| Actuator resource | per-actuator effort margin + limiting actuator | continuous effort headroom; no thermal inference |
| Recovery | typed MaxIterations / PrimalInfeasible | bounded search exhaustion is not relabeled as a proof |
| Solver budget | clipped steps + hard violation | numerical pressure stays distinct from physical headroom |
| Backend | CpuExactF64 | allocation-free CPU semantic reference; CUDA solve unavailable |

## Independent Pinocchio oracle

| model | states | dynamics L∞ | contact L∞ | friction margin | effort margin Nm | pass |
|---|---|---|---|---|---|---|
| toy_humanoid | 16 | 1.153e-09 | 4.675e-11 | 9.505e+01 | 1.295e+01 | True |
| upkie | 16 | 1.270e-11 | 1.791e-12 | 1.728e+01 | 1.048e+00 | True |

The oracle rebuilds mass, bias, and contact Jacobians from URDF in Pinocchio and recomputes the equations from returned commands. Rust's own residual fields are not used as the admission witness.

## Contact authority sweep

| friction μ | statuses | min cone margin | Intent residual |
|---|---|---|---|
| 1.0 | {'SolvedWithSlack': 8} | 2.505e+01 | 1.762e+01 |
| 0.5 | {'SolvedWithSlack': 8} | 1.027e+01 | 1.762e+01 |
| 0.2 | {'SolvedWithSlack': 8} | 1.395e+00 | 1.762e+01 |
| 0.05 | {'SolvedWithSlack': 7, 'MaxIterations': 1} | -1.554e-15 | 1.762e+01 |

## Actuator derating sweep

| effort scale | statuses | min effort margin Nm | Viability residual |
|---|---|---|---|
| 1.0 | {'SolvedWithSlack': 8} | 5.288e-01 | 6.216e+00 |
| 0.6 | {'SolvedWithSlack': 8} | 0.000e+00 | 6.216e+00 |
| 0.35 | {'SolvedWithSlack': 8} | -1.110e-16 | 8.947e+00 |
| 0.2 | {'MaxIterations': 8} | 3.400e-01 | 9.949e+00 |

These sweeps intentionally expose friction margin, actuator margin, and tracking residual separately. A physically bounded solve may sacrifice a task; `SolvedWithSlack` is useful output, not a false pose guarantee.

## Failure typing and isolation

| check | result |
|---|---|
| invalid_agent | 8 |
| invalid_status | InvalidInput |
| invalid_status_code | 7 |
| neighbor_outputs_bitwise_exact | True |
| zero_normal_authority_status_counts | {'MaxIterations': 17} |
| failure_typed | True |
| failed_commands_zeroed | True |
| pass | True |

## Determinism

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| permutation_exact | True |
| padding_exact | True |
| allocation_calls | 0 |
| pass | True |

## Release timing (untrimmed)

| agents | dynamic solve p50 us | dynamic solve p99 us | pipeline p50 us | alloc calls |
|---|---|---|---|---|
| 1 | 117.162 | 132.197 | 169.980 | 0 |
| 8 | 944.088 | 1009.715 | 1311.924 | 0 |
| 32 | 3767.556 | 4487.726 | 5210.436 | 0 |

Timers cover preallocated Rust stages and exclude NumPy marshalling. Every sampled call is retained.

## Deliberate scope boundary

R82 admits horizontal point contacts only. Finite support patches, center-of-pressure/CoM polygon inequalities, contact switching, power, thermal state, actuator realization, walking rollout, and a CUDA hierarchical solver remain unavailable. Gravity is currently the explicit fixed world vector `[0, 0, -9.81]`; other gravity inputs are typed `InvalidInput`, not silently accepted. The existing r81 joint stopping envelope is architecturally upstream but is not yet composed into this dynamic Python entry point.
