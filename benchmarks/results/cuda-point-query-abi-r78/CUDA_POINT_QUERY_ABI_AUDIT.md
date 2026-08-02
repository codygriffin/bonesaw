# Bonesaw fixed point-query mirror · r78

> PROMOTE COMPILER-RESOLVED CPU POINT QUERIES ONLY. CUDA remains unavailable; no device correctness or performance claim is made.

This policy-free, simulator-free gate adds stable-ID point slots after the admitted FK/Jacobian/dynamics stages. Every slot has a frozen body frame and local offset hashed into the kernel descriptor and emits world position, floating point Jacobian, and kinematic bias acceleration Jdot-v.

## Admission decision

| Gate | Evidence | Decision |
|---|---|---|
| Point position/J ↔ Pinocchio | 2 / 2 models | PASS |
| Jv ↔ central-difference velocity | root SO(3)/translation + every joint | PASS |
| Jdot-v ↔ central-difference acceleration | zero world generalized acceleration | PASS |
| Stable IDs and offsets | descriptor-hashed fixed slot plan | PASS |
| Repeat/reindex/isolation | bitwise exact | PASS |
| Measured Rust allocations | 0 calls · 0 B | PASS |
| CUDA point-query D1/D2/D3 | executor unavailable | NOT RUN |

## Independent point oracle

| Model | States×slots | position max/gate | J max/gate | Jv max/gate | Jdot-v max/gate | D3 |
|---|---|---|---|---|---|---|
| toy_humanoid | 16×4 | 1.712e-07/0.001× | 1.514e-07/0.003× | 4.970e-08/0.001× | 2.648e-08/0.000× | PASS |
| upkie | 16×4 | 8.570e-08/0.002× | 1.136e-07/0.002× | 5.740e-08/0.001× | 2.495e-08/0.000× | PASS |

Pinocchio independently supplies BODY placements and LOCAL_WORLD_ALIGNED spatial Jacobians. The evaluator rotates each nonzero local offset, converts Pinocchio's local linear-first free-flyer tangent to Bonesaw's world angular-first tangent, and constructs the point Jacobian. A separate central difference left-multiplies root Exp(±dt·omega), translates the root, and advances every joint at constant generalized velocity. Its first derivative checks Jv; its second derivative checks Jdot-v without a policy or physics rollout.

## Exact layout and isolation

| Invariant | Evidence | Result |
|---|---|---|
| Layout | point[slot][3][agent], J[slot][3][g][agent], bias[slot][3][agent] | FROZEN |
| Repeat | True | EXACT |
| Permutation/padding/chunking | True / True / True | EXACT |
| Malformed velocity | agent 8 zeroed; neighbors unchanged | ISOLATED |

## Host timing · Upkie four point slots

| Agents | Point p50/p99 | Full pipeline p50/p99 | Full/agent p50 | Alloc |
|---|---|---|---|---|
| 1 | 1.182/1.224 µs | 52.539/60.353 µs | 52539.0 ns | 0 / 0 B |
| 32 | 25.098/30.250 µs | 1415.780/1470.447 µs | 44243.1 ns | 0 / 0 B |
| 256 | 213.178/251.488 µs | 11719.689/11951.683 µs | 45780.0 ns | 0 / 0 B |

Stage timers exclude NumPy↔SoA copies and retain every timing sample, including the p99 tail. This admits query products, not task residual/target emission or a solver.

## Deferred boundary

The next CPU slice may lower fixed point targets and contact locks into stable-ID task/constraint rows using these exact J and Jdot-v products. CudaMirrorF32 must later reproduce the point plan and pass device D1/D2/D3 on real hardware; no CPU fallback may masquerade as device execution.
