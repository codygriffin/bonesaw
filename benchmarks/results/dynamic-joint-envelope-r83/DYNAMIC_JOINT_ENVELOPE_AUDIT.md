# Bonesaw dynamic joint-envelope composition · r83

## Outcome

**Admission: PASS.** The R81 stopping envelope now executes inside the same preallocated Rust call as the R82 physical decision vector `[qdd, generalized joint effort, point-contact force]`. Caller acceleration authority and the derived position/velocity/braking box are intersected before floating dynamics, locked contacts, effort, unilateral force, friction, and lexicographic point tasks are solved. Python remains eval/oracle land; no policy or physics rollout is used.

## Example authority stack

These witnesses are ordered but never aggregated. In particular, a joint-envelope margin is not an actuator-effort or friction margin.

| layer | concrete authority | separate witness |
|---|---|---|
| Invariant | contact locks 7951 / 7952 | hard point acceleration equalities |
| Invariant | floating dynamics | qdd, effort, and force remain one physical decision |
| Viability | r83 stopping envelope | position/velocity/braking limits intersect caller qdd authority in Rust |
| Viability | torso point 7901 | protected residual after every hard physical row |
| Intent | handle point 7902 | retains residual as the stopping box consumes authority |
| Contact resource | normal-force / friction margins | not merged with joint or actuator margin |
| Actuator resource | effort margin / limiting actuator | not inferred from the acceleration box |
| Recovery | outside-limit maximum inward qdd | deterministic hard recovery command |
| Problem typing | InvalidInput / InvalidProblem / MaxIterations | malformed data, empty boxes, and work exhaustion stay distinct |
| Backend | CpuExactF64 | zero-allocation semantic path; CUDA remains unavailable |

## Independent double oracle

| model | states | NumPy bound delta | Pin dynamics L∞ | Pin contact L∞ | pass |
|---|---|---|---|---|---|
| toy_humanoid | 16 | 0.000e+00 | 4.440e-09 | 1.101e-10 | True |
| upkie | 16 | 0.000e+00 | 1.271e-11 | 1.792e-12 | True |

NumPy independently reconstructs every stopping interval from URDF limits and the observed f32 state. Pinocchio independently rebuilds M, h, and contact Jacobians and checks the returned physical command. Neither admission witness trusts Rust's own diagnostics.

## Near-limit conflict and recovery

| check | result |
|---|---|
| coordinate | 3 |
| joint_name | right_hip |
| near_limit_envelope_upper_qdd | 0.050115585327192846 |
| unconstrained_max_qdd | 75.95171655038797 |
| constrained_max_qdd | 0.05011558532719211 |
| minimum_envelope_margin | 0.0 |
| maximum_intent_residual | 21.772022772423167 |
| constrained_status_counts | {'SolvedWithSlack': 8} |
| recovery_status_counts | {'SolvedWithSlack': 8} |
| outside_limit_recovery_qdd_exact | True |
| pass | True |

The outward handle request is intentionally retained rather than silently weakened. As the stopping envelope closes, its Intent residual rises while dynamics, contact, effort, force, and the envelope remain hard.

## Continuous approach sweep

| check | result |
|---|---|
| coordinate | 3 |
| samples | 64 |
| farthest_upper_qdd | 20.0 |
| nearest_upper_qdd | 0.050115585327192846 |
| upper_bound_monotone_nonincreasing | True |
| solved_commands_inside_envelope | True |
| status_counts | {'SolvedWithSlack': 64} |
| minimum_envelope_margin | 0.0 |
| maximum_intent_residual | 18.239248657994327 |
| pass | True |

## Failure typing and isolation

| check | result |
|---|---|
| invalid_agent | 8 |
| invalid_agent_isolated_bitwise | True |
| disabled_poisoned_payload_ignored_exactly | True |
| empty_intersection_status_counts | {'InvalidProblem': 17} |
| empty_intersection_zero_commands | True |
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

| agents | envelope p50 us | solve p50 us | solve p99 us | pipeline p50 us | alloc calls |
|---|---|---|---|---|---|
| 1 | 0.351 | 117.602 | 128.074 | 170.667 | 0 |
| 8 | 1.232 | 938.255 | 967.164 | 1309.091 | 0 |
| 32 | 4.078 | 3774.207 | 4052.600 | 5222.945 | 0 |

Timers cover preallocated Rust stages only and exclude NumPy marshalling. All calls are retained.

## Scope boundary

R83 composes joint viability with point-contact dynamics, effort, and friction. Finite support patches, CoP/CoM polygon inequalities, power, thermal realization, contact switching, and a CUDA hierarchical solver remain unavailable. The fixed world-gravity contract remains `[0, 0, -9.81]`. Body response is not inferred without a later realization or closed-loop stage.
