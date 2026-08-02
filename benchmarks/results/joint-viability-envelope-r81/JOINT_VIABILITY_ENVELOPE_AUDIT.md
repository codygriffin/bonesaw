# Bonesaw joint viability envelope · r81

## Outcome

**Admission: PASS.** Rust now derives fixed-shape next-tick joint acceleration envelopes from observed position/velocity, authored position/velocity limits, explicit acceleration authority, and control period. The envelope feeds `CpuExactF64` directly in Rust; Python independently reconstructs the algebra and orchestrates evaluation.

This is kinematic safety evidence, not actuator-effort or contact-force feasibility. Those require the dynamic decision vector and are deliberately not inferred from this result.

## Independent URDF/NumPy oracle

| model | states | max bound error | ∞ masks exact | pass |
|---|---|---|---|---|
| toy_humanoid | 32 | 0.000e+00 | True | True |
| upkie | 32 | 0.000e+00 | True | True |

## Example authority stack

| layer | concrete signal | meaning |
|---|---|---|
| Invariant | contact locks 7951/7952 | remain hard equalities in the downstream strict solve |
| Viability | joint stopping envelope | position, velocity, next-tick position, and braking distance intersect as hard qdd bounds |
| Intent | point target 7902 | retains residual when the joint envelope consumes authority |
| Recovery | maximum inward qdd | an already-unsafe observation yields a deterministic recovery box, never an inverted interval |
| Resources | actuator acceleration only when explicitly authored | URDF effort is not reinterpreted as acceleration or torque feasibility |
| Dynamic feasibility | UNAVAILABLE in this batch slice | effort/friction/support require [qdd, tau, f] and remain the next admission |

## Recovery and isolation

- Outside-limit coordinate 0 receives exactly -20.0 rad/s² inward acceleration authority.
- Invalid-agent isolation exact: True; disabled payload ignored: True.

## Continuous approach sweep

| check | result |
|---|---|
| coordinate | 0 |
| sample_count | 64 |
| farthest_upper_qdd | 20.0 |
| nearest_upper_qdd | -20.0 |
| monotone_nonincreasing | True |
| requests_inward_before_limit | True |
| pass | True |

## Strict-solve composition

| check | result |
|---|---|
| all_commands_inside_envelope | True |
| status_all_solved_with_slack | True |
| minimum_solved_envelope_margin | 0.0 |
| clipped_steps | 96 |
| maximum_soft_residual | 111.25113887052382 |
| allocation_calls | 0 |
| pass | True |

## Determinism

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| permutation_exact | True |
| padding_exact | True |
| allocation_calls | 0 |
| pass | True |

## Release timing

| agents | p50 µs | p99 µs | p50 ns/agent | alloc calls |
|---|---|---|---|---|
| 1 | 0.361 | 0.371 | 361.0 | 0 |
| 32 | 3.968 | 4.970 | 124.0 | 0 |
| 256 | 30.939 | 37.087 | 120.9 | 0 |

Timers cover the preallocated Rust envelope executor only and exclude NumPy marshalling. All 200 calls per batch are retained.

## Next physical gate

The next batch slice must extend the decision vector to `[q̈, τ, contact force]`, enforce `M q̈ + h = Sᵀτ + Jᶜᵀf`, and only then add exact actuator effort, unilateral force, friction, and finite-support inequalities. Keeping that boundary explicit prevents a kinematically reachable request from being mislabeled physically realizable.
