# Bonesaw finite support patch · r84

## Outcome

**Admission: PASS.** Fixed support-patch topology now crosses the Rust/Python batch boundary. Rust constructs the convex hull from declared point-force slots and emits exact inward-eroded CoP inequalities. Python independently rebuilds Pinocchio dynamics/contact equations and the polygon/CoP geometry. This checkpoint uses neither a policy nor a physics rollout.

## Example authority stack

This is the concrete stack used for architectural review. Authority ordering and every witness remain separate; no scalar ‘feasibility score’ hides which resource is binding.

| layer | example authority | separate witness |
|---|---|---|
| Invariant | floating rigid-body dynamics | M qdd + h = Sᵀ effort + Jᶜᵀ force; hard residual |
| Invariant | five explicitly declared point contacts | per-point lock residual, unilateral load, friction margin |
| Viability | finite left-sole patch 8401 | hard CoP polygon; 20 mm requested margin + limiting stable ID |
| Viability | r83 joint stopping envelope | per-joint qdd interval and recovery count; independent of support |
| Intent | pelvis lateral attractor 8201 | residual rises when the CoP edge owns authority |
| Contact resource | normal and friction headroom | point-force blocks stay observable; no inferred wrench |
| Actuator resource | effort headroom + limiting actuator | continuous authority, separate from geometric support |
| Recovery | typed invalid/missing declarations | bad agent zeroed; neighbors bitwise unchanged |
| Solver budget | hard violation + clipped steps + task slack | numerical pressure is not physical feasibility |
| Backend | CpuExactF64 | preallocated Rust semantic reference; CUDA solve still unavailable |

## Independent Pinocchio + NumPy oracle

| states | dynamics L∞ | contact L∞ | minimum CoP margin m | reported Δ m | pass |
|---|---|---|---|---|---|
| 16 | 3.112e-10 | 7.312e-12 | 0.020000 | 1.388e-17 | True |

The oracle uses URDF geometry and Pinocchio products, returned point forces, and an independent monotone-chain hull. It does not trust Rust’s reported support or dynamics residuals for admission.

## Edge conflict

| condition | oracle margin m | Intent residual |
|---|---|---|
| support patch disabled | 0.012998 | 15.313750 |
| 20 mm support margin enabled | 0.020000 | 15.431411 |

The unconstrained solve places CoP inside the physical sole but below the requested inward margin. Enabling patch 8401 pins CoP to 20 mm and increases only the lower-authority Intent residual.

## Zero-load semantics and honest Upkie topology

| check | result |
|---|---|
| status | SolvedWithSlack |
| patch_total_normal_force_n | 0.0 |
| outside_patch_normal_force_n | 385.98695758588906 |
| reported_margin | Infinity |
| limiting_patch | None |
| pass | True |

| check | result |
|---|---|
| upkie_declared_contact_points | 2 |
| finite_area_claimed | False |
| two_point_patch_rejected | True |
| rejection | support patch 1 is invalid for the fixed contact declaration |
| pass | True |

Upkie’s two wheel contacts are a line, not a finite-area sole. R84 rejects a two-point patch and makes no finite support-margin claim for Upkie.

## Failure isolation

| check | result |
|---|---|
| invalid_margin_agent | 8 |
| invalid_margin_isolated | True |
| missing_patch_point_agent | 9 |
| missing_point_isolated | True |
| disabled_poisoned_margin_ignored_bitwise | True |
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

| agents | solve p50 us | solve p99 us | pipeline p50 us | alloc calls |
|---|---|---|---|---|
| 1 | 1812.859 | 1844.919 | 2056.499 | 0 |
| 8 | 13216.845 | 14605.411 | 15118.566 | 0 |
| 32 | 53905.942 | 54479.100 | 61405.240 | 0 |

All timing samples are retained. Timers cover preallocated Rust stages and exclude NumPy marshalling.

## Deliberate scope boundary

R84 admits fixed, horizontal finite-support declarations over point-force slots. It does not yet switch patches during gait, add a CoM-in-polygon task, model deformable contact, or stream this dynamic authority row into the orbit editor. The four sole points are all locked in this reference fixture; a later contact declaration revision should separate force-point existence from rank-minimal kinematic locking. CUDA remains gated on completion of the CPU concept and evaluations.
