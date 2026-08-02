# Bonesaw fixed-layout Jacobian mirror · r76

> PROMOTE THE CPU JACOBIAN STAGE ONLY. CUDA remains unavailable; no device correctness or performance claim is made.

This policy-free, simulator-free gate extends kernel ABI v1 through world-expressed floating frame-origin and center-of-mass Jacobians. Tangent order is root angular, root linear, then joints. Python owns evaluation; both timed stages execute in preallocated Rust.

## Admission decision

| Gate | Result | Decision |
|---|---|---|
| CpuMirrorF32 ↔ Pinocchio f64 Jacobians | 2 / 2 | PASS |
| J·v ↔ finite-difference point/CoM velocity | 2 / 2 | PASS |
| D1 repeat bytes | True | PASS |
| Permutation / padding / chunking | True / True / True | PASS |
| Typed invalid-agent isolation | True | PASS |
| Measured Rust allocations | 0 calls · 0 B | PASS |
| CUDA Jacobian D1 / D2 / D3 | executor unavailable | NOT RUN |

## Independent Pinocchio oracle

| Model | States × bodies | Frame max abs | Frame gate | CoM max abs | CoM gate | D3 |
|---|---|---|---|---|---|---|
| toy_humanoid | 16 × 21 | 1.533e-07 | 0.003× | 1.788e-07 | 0.001× | PASS |
| upkie | 16 × 41 | 9.301e-08 | 0.002× | 5.166e-08 | 0.001× | PASS |

Each element must satisfy abs(error) ≤ 2e-05 + 2e-04·max(abs(values)). Root columns are independently reconstructed in world coordinates; joint columns come from Pinocchio LOCAL_WORLD_ALIGNED frame and CoM Jacobians.

## Finite-difference velocity property

| Model | Frame-origin max | Frame gate | CoM max | CoM gate | Result |
|---|---|---|---|---|---|
| toy_humanoid | 1.034e-04 | 0.256× | 9.042e-05 | 0.162× | PASS |
| upkie | 7.238e-05 | 0.134× | 6.039e-05 | 0.111× | PASS |

A central difference perturbs root orientation by left-multiplying Exp(±εω), root translation by ±εv, and every joint by ±εqdot. It checks the linear frame-origin rows and CoM Jacobian against observed velocity without a policy or physics rollout.

## Exact layout and isolation gates

| Invariant | Evidence | Result |
|---|---|---|
| Layout | frame[body][6][6+dof][agent] + com[3][6+dof][agent] | FROZEN |
| Repeat | poses + CoM + mass + frame/CoM Jacobians + status | EXACT |
| Permutation | reverse-interleave then inverse index | EXACT |
| Padded stride | 17 active: stride 32 vs 17 | EXACT |
| Chunking | 7 + 10 agents reassembled | EXACT |
| Malformed state | agent 8 status=2; zero Jacobians; neighbors exact | ISOLATED |

## Host timing · Upkie CPU mirror only

| Agents | FK p50/p99 | Jacobian p50/p99 | Combined p50/p99 | Combined / agent p50 | Allocations |
|---|---|---|---|---|---|
| 1 | 6.703/12.408 µs | 13.535/18.888 µs | 20.238/26.782 µs | 20238.0 ns | 0 / 0 B |
| 32 | 189.017/264.327 µs | 302.426/408.297 µs | 492.825/662.543 µs | 15400.8 ns | 0 / 0 B |
| 256 | 1597.843/2125.445 µs | 2588.099/3831.132 µs | 4185.700/5949.008 µs | 16350.4 ns | 0 / 0 B |

Timers surround Rust FK/CoM and Jacobian stages separately. NumPy↔SoA copies are excluded. The current Jacobian implementation deliberately favors auditable fixed traversal over tuning; this is an admission baseline, not a throughput result.

## Deferred device admission

CudaMirrorF32 must reproduce every frame and CoM Jacobian stage output under D1, satisfy CpuMirrorF32↔device D2/D3 tolerances, and repeat the exact agent-reindexing, padding, chunking, canary/isolation, warmup, memory-scaling, and jitter gates. CudaThroughputF32 remains separately fingerprinted and uncertified.
