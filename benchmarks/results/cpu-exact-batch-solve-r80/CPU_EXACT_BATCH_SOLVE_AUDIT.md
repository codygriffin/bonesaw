# Bonesaw strict CPU batch solve · r80

## Outcome

**Admission: PASS.** Fixed r79 task/contact rows now feed the established allocation-free `f64` strict hierarchy. This is the CPU semantic implementation requested before GPU batching; `CpuMirrorF32` still stops at row emission and no CUDA hierarchical solver is claimed.

The independent oracle is NumPy SVD over the same emitted physical rows, with per-row normalization, hard contact equalities, and a sequential null-space projector. It is independent of the Rust pseudoinverse implementation. Evals use no policy, physics rollout, or learned component.

## Independent numerical oracle

| model | max |qdd−NumPy| | max |level l2−NumPy| | hard contact L∞ | pass |
|---|---|---|---|---|
| toy_humanoid | 4.362e-10 | 1.804e-10 | 3.126e-13 | True |
| upkie | 9.581e-09 | 1.193e-10 | 3.979e-13 | True |

## Example authority stack

This concrete stack is also published in the live architecture review. Values never collapse into one health score.

| layer | example authority | interpretation |
|---|---|---|
| Invariant | contact-lock blocks 7951 / 7952 | hard 3-axis equalities; violation is reported separately |
| Viability | torso attractor 7901 · priority 1 | its normalized residual is frozen before lower layers |
| Intent | handle attractor 7902 · priority 2 | may retain residual when it conflicts with viability |
| Preference | no row in this example | undeclared is not displayed as a zero residual |
| Style | no row in this example | reserved terminal refinement |
| Feasibility | bound margin + hard violation + clipping | continuous compromise evidence, not a binary pose promise |
| Solver budget | MaxIterations | bounded search exhausted; explicitly not an infeasibility certificate |
| Resources | effort / power / thermal remain separate | this kinematic row bridge cannot promise physical realization |
| Backend | CpuExactF64 | strict CPU reference path; no CUDA solve claim |

## Bounded compromise and failure typing

* Narrow ±0.01 acceleration bounds: status is `SolvedWithSlack` for every agent; max |qdd| 0.010000; soft residual 1.031e+02; 48 clipped steps.
* Invalid bounds isolate agent 8 with `InvalidProblem`; neighboring commands remain bitwise exact: True.
* Contradictory nonzero hard rows are covered by Rust tests as `MaxIterations`, with continuous maximum violation and projection-work counters. Structural zero-row contradiction remains `PrimalInfeasible`. Commands are zeroed for non-solved statuses.

## Determinism and allocation

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| permutation_exact | True |
| padding_exact | True |
| allocation_calls | 0 |
| pass | True |

## Release timing (untrimmed)

| agents | solve p50 µs | solve p99 µs | pipeline p50 µs | alloc calls |
|---|---|---|---|---|
| 1 | 9.548 | 9.787 | 62.462 | 0 |
| 32 | 196.286 | 364.666 | 1632.873 | 0 |
| 256 | 1579.082 | 1968.185 | 13408.090 | 0 |

Timers exclude Python/NumPy marshalling and include only preallocated Rust execution. The full-pipeline timer is the sum of separately sampled FK, Jacobian, dynamics, point-query, emission, and exact-solve stages.

## Scope boundary

This checkpoint proves a stable CPU-exact consumer for point-attractor and contact-lock rows, strict priority, bound compromise, typed bounded-work failure, fixed diagnostics, determinism, and zero measured hot-path allocation. It does not yet add friction cones, torque/power/thermal inequalities to this batch ABI, physics realization, walking retargeting, or a CUDA solver. Those remain explicit subsequent admission gates.
