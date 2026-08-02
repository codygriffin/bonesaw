# Bonesaw rank-minimal support declaration · r85

## Outcome

**Admission: PASS.** Kernel ABI 2 separates force-point existence from kinematic row mode. Four sole points retain twelve independently bounded force variables and the exact finite CoP cone, while Locked + Normal + Disabled + Rolling modes emit six full-rank rigid-foot equations. Rust owns topology, emission, solve, and memory; Python owns independent oracles and reporting. No policy or physics rollout is used.

## Example authority stack

| layer | example authority | separate witness |
|---|---|---|
| Invariant | six rank-minimal sole rows | rank 6; all 12 material-point acceleration components implied |
| Invariant | floating dynamics | independent Pinocchio hard residual |
| Viability | finite patch 8401 at 20 mm | independent weighted-CoP margin + limiting stable ID |
| Viability | joint stopping envelope | separate qdd interval, recovery, and limiting joint |
| Intent | point attractor | physical-unit residual; cannot perturb support/dynamics |
| Preference/Style | normal-force distribution | yields from corner nominal when CoP margin binds |
| Contact resource | four force points / twelve force variables | unilateral and friction margins remain per point |
| Actuator resource | effort headroom | separate from support geometry and task residual |
| Solver budget | status, clipped steps, hard violation | bounded work remains distinct from physical headroom |
| Backend | CpuExactF64 | zero-allocation semantic reference; CUDA unavailable |

## Independent rank + dynamics + support oracle

| states | rows | rank | σmin | implied all-point L∞ | Pin dynamics L∞ | CoP min m | pass |
|---|---|---|---|---|---|---|---|
| 16 | 6 | [6] | 4.802e-02 | 2.787e-12 | 5.660e-10 | 0.020000 | True |

Pinocchio independently stacks the six selected rows and then checks all twelve sole-point acceleration components. NumPy independently rebuilds the hull and force-weighted CoP. Contact mode is descriptor-fingerprinted, so changing row semantics cannot reuse a stale kernel identity.

## Support versus Style force preference

| condition | CoP margin m | force-style L2 N |
|---|---|---|
| patch disabled | 0.008018 | 71.747 |
| 20 mm patch enabled | 0.020000 | 142.892 |

A corner-loaded nominal force distribution is legal without the patch but violates the requested reserve. Enabling patch 8401 moves CoP to exactly 20 mm and sacrifices only the lower-authority force-distribution objective.

## All-locked versus rank-minimal timing (untrimmed)

| agents | locked p50 us | minimal p50 us | p50 Δ | locked p99 us | minimal p99 us | alloc calls |
|---|---|---|---|---|---|---|
| 1 | 1815.488 | 1258.221 | -30.70% | 1865.594 | 1285.112 | 0 |
| 8 | 13848.863 | 9717.377 | -29.83% | 16893.750 | 9986.225 | 0 |
| 32 | 54133.707 | 38986.256 | -27.98% | 54847.753 | 39300.947 | 0 |

Profiles run sequentially on the same host and retain every sample; this is a directional implementation audit, not a pinned alternating hardware-counter promotion claim.

## Failure isolation and determinism

| check | result |
|---|---|
| invalid_margin_agent | 8 |
| invalid_margin_isolated | True |
| missing_patch_point_agent | 9 |
| missing_point_isolated | True |
| disabled_poisoned_margin_ignored_bitwise | True |
| pass | True |

| check | result |
|---|---|
| repeat_bitwise_exact | True |
| permutation_exact | True |
| padding_exact | True |
| allocation_calls | 0 |
| pass | True |

## Deliberate scope boundary

R85 exposes Disabled, LockedPoint, NormalPoint, and RollingPoint fixed modes. RollingWheel needs its coordinate and stabilization constants in a later descriptor revision. General automatic row-basis synthesis, contact switching, CoM-in-polygon tasks, integration, and CUDA remain unavailable. The all-locked and rank-minimal solvers may select different lower-layer joint/force optima even though both satisfy the same rigid-foot hard manifold; this report admits physical constraints and independent witnesses, not bitwise solution equivalence across a changed equality basis.
