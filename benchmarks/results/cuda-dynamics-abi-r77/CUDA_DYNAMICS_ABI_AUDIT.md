# Bonesaw fixed-layout dynamics mirror · r77

> PROMOTE CPU MODEL PRODUCTS ONLY. CUDA remains unavailable; no device correctness or performance claim is made.

This policy-free, simulator-free gate extends the fixed batch ABI through floating mass matrix M(q), bias force h(q,v,g), and centroidal momentum map Ag(q). Python owns evaluation; preallocated Rust owns every timed stage.

## Admission decision

| Gate | Evidence | Decision |
|---|---|---|
| CpuMirrorF32 ↔ independent Pinocchio | 2 / 2 models | PASS |
| Mass symmetry / positive definiteness | exact symmetry; positive minimum eigenvalue | PASS |
| D1 repeat bytes | True | PASS |
| Permutation / padding / chunking | True / True / True | PASS |
| Typed invalid-agent isolation | True | PASS |
| Measured Rust allocations | 0 calls · 0 B | PASS |
| CUDA dynamics D1 / D2 / D3 | executor unavailable | NOT RUN |

## Independent Pinocchio oracle

| Model | States | M max / gate | h max / gate | Ag max / gate | min eig(M) | D3 |
|---|---|---|---|---|---|---|
| toy_humanoid | 16 | 8.404e-06 / 0.021× | 3.750e-05 / 0.003× | 6.559e-06 / 0.164× | 1.308e-02 | PASS |
| upkie | 16 | 5.552e-07 / 0.003× | 5.695e-06 / 0.005× | 3.241e-07 / 0.008× | 1.707e-04 | PASS |

Pinocchio uses a body-local free-flyer tangent ordered linear then angular. The oracle explicitly maps it to Bonesaw's world-expressed angular-then-linear root tangent. For bias, it also supplies the configuration-dependent local linear acceleration induced when world root acceleration is zero. This prevents a false comparison caused by mismatched acceleration conventions.

Declared element gates are M: 4e-5 + 4e-4·scale, h: 1e-4 + 8e-4·scale, Ag: 4e-5 + 4e-4·scale. The corpus uses nonzero root/joint velocity and non-axis-aligned, per-agent gravity.

## Exact ABI and isolation gates

| Invariant | Evidence | Result |
|---|---|---|
| Layout | M[g][g][agent], h[g][agent], Ag[6][g][agent] | FROZEN |
| Repeat | M + h + Ag + status | EXACT |
| Permutation | reverse-interleave then inverse index | EXACT |
| Padded stride | 17 active: stride 32 vs 17 | EXACT |
| Chunking | 7 + 10 agents reassembled | EXACT |
| Malformed velocity | agent 8 status=2; zero products; neighbors exact | ISOLATED |

## Host timing · Upkie CPU mirror only

| Agents | FK p50/p99 | J p50/p99 | Dynamics p50/p99 | Combined p50/p99 | Combined/agent p50 | Alloc |
|---|---|---|---|---|---|---|
| 1 | 6.512/7.748 µs | 13.576/18.577 µs | 29.656/35.431 µs | 49.784/57.391 µs | 49784.0 ns | 0 / 0 B |
| 32 | 182.765/233.408 µs | 297.913/380.844 µs | 896.027/1116.784 µs | 1379.376/1727.384 µs | 43105.5 ns | 0 / 0 B |
| 256 | 1599.091/1663.447 µs | 2588.319/2803.730 µs | 7228.333/7472.063 µs | 11417.015/11873.409 µs | 44597.7 ns | 0 / 0 B |

Timers are stage-local and exclude NumPy↔SoA copies. The entire untrimmed timing trace feeds every percentile, including the p99 jitter tail. The current body-sum dynamics traversal is an auditable CPU admission baseline, not a throughput claim.

## Deferred device admission

CudaMirrorF32 must reproduce these exact layouts and D1 reindexing/isolation behavior, then satisfy CPU↔device D2/D3 gates for M, h, and Ag on a real CUDA driver/toolkit. CudaThroughputF32 remains separately fingerprinted and uncertified.
