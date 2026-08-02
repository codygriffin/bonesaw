# Bonesaw fixed-layout batch mirror · r75

> PROMOTE THE ABI AND CPU MIRROR ONLY. The CUDA device executor is unavailable; no GPU correctness, latency, throughput, or determinism claim is made.

This policy-free, simulator-free gate freezes the first state/FK/CoM batch contract before device work. Python owns experiment orchestration and Pinocchio comparison; the measured transition is fixed-layout, preallocated Rust.

## Admission decision

| Gate | Result | Decision |
|---|---|---|
| CpuMirrorF32 ↔ Pinocchio f64 (D3) | 2 / 2 | PASS |
| D1 repeat bytes | True | PASS |
| Permutation / padding / chunking | True / True / True | PASS |
| Typed invalid-agent isolation | True | PASS |
| Rust execute allocations | 0 calls · 0 B | PASS |
| CUDA device D1 / D2 / D3 | executor unavailable | NOT RUN |

## Independent f64 oracle

| Model | States × bodies | Translation max | Rotation max | CoM max | Orthogonality | D3 |
|---|---|---|---|---|---|---|
| toy_humanoid | 16 × 21 | 1.539e-07 m | 5.905e-08 rad | 1.557e-07 m | 2.384e-07 | PASS |
| upkie | 16 × 41 | 7.943e-08 m | 2.623e-08 rad | 7.145e-08 m | 2.980e-07 | PASS |

D3 thresholds are 5e-05 m translation, 5e-05 rad rotation, and 5e-05 m CoM. Rotation uses atan2(skew, trace), with orthogonality checked separately, so f32 drift cannot masquerade as a trace/acos angle.

## Exact layout and isolation gates

| Invariant | Evidence | Result |
|---|---|---|
| Agent SoA stride | 17 agents: aligned 32, compact 17 | EXACT |
| Repeat | body pose + CoM + mass + status bytes | EXACT |
| Permutation | reverse-interleave then inverse index | EXACT |
| Chunking | 7 + 10 agents reassembled | EXACT |
| Malformed state | agent 8 status=2; every neighbor compared | ISOLATED |

## Host timing · CPU mirror only

| Agents | Stride | p50 batch | p99 batch | p50 / agent | p99 / agent | Allocations |
|---|---|---|---|---|---|---|
| 1 | 32 | 6.732 µs | 8.577 µs | 6732.0 ns | 8577.4 ns | 0 / 0 B |
| 32 | 32 | 191.101 µs | 211.130 µs | 5971.9 ns | 6597.8 ns | 0 / 0 B |
| 256 | 256 | 1984.523 µs | 2648.127 µs | 7752.0 ns | 10344.2 ns | 0 / 0 B |

The timer surrounds only Rust execute_into. NumPy-to-SoA input copy and SoA-to-NumPy output copy are intentionally outside it; end-to-end transport belongs to a later websocket/device report.

## Frozen contract

Kernel ABI version 1 exposes StateInput + ForwardKinematics + CenterOfMass. State is state[coordinate][agent]; root and frame poses are pose[frame][12 components][agent], with row-major rotation followed by translation. MirrorF32 fixes explicit FMA, disables fast math and FTZ, and fingerprints program, build, ISA, layout, manifest, and math flags.

## Deferred device admission

CudaMirrorF32 must reproduce these stage outputs under D1, satisfy CpuMirrorF32↔CudaMirrorF32 D2 tolerances, rerun D3 against Pinocchio, and pass permutation, padded-stride, chunking, malformed-agent, memory-scaling, warmup, jitter, and fingerprint gates on a working NVIDIA driver/toolkit. CudaThroughputF32 remains a separate relaxed profile and cannot inherit mirror certification.
