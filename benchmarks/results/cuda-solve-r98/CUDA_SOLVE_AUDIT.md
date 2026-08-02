# Bonesaw CUDA fixed-level hierarchical solve · r98

## Outcome

**SOURCE + CPU MIRROR PASS; COMPILER/DEVICE UNAVAILABLE.** R98 adds the real no-fallback CUDA `HierarchicalSolve` source, fixed global scratch, compiler probe, manifest stage, capability, host allocation/launch/copy boundary, and conditional device conformance test. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries → Emission → Solve is ordered in one stream with one terminal synchronization.

This does not promote the full CUDA mirror. Actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent backend authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | fixed entry and scratch; no allocation, atomic, or block barrier |
| CpuMirrorF32 algorithm witness | PASS | 500-call replay, strict-f64 compatible command, budget typing |
| NVRTC FK/CoM | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Jacobians | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Dynamics | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Point queries | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Emission | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Solve | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device solve D1/D3 | NOT RUN | never inherited from source or CPU checks |
| full CudaMirrorF32 / Graph | False / False | later stages remain gated |

## Frozen solve boundary

| signal | value |
|---|---|
| stage manifest bits | 6615 |
| solve source SHA-256 | 14648a990708c1426fef642878863c8a9efd13970c39b3e403e6fc26a8732058 |
| mirror algorithm SHA-256 | 695c600907401e9c264d29b02fe8b8bc6a16b58b8c7dfed3454edc2443a2fd19 |
| arithmetic semantics | fixed_level_approximate |
| hard / task budget | 64 hard sweeps · 32 sweeps per active level · no early exit |
| batch layout | 24 generalized coordinates · capacity 17 · stride 32 |
| fixed CPU solve input/output/scratch | 29366 B |
| hidden CPU fallback | False |

One thread owns one complete agent and every reduction order. Immutable task priority/weight and contact mode arrays are uploaded once. Bounds are copied into fixed input buffers; commands, retained candidates, typed status, per-level residual/preservation fields, hard progress, margins, and work counters use fixed device outputs. Scratch is global agent-minor storage sized at construction.

## Example authority stack retained for architecture review

| layer | concrete example | independent witness |
|---|---|---|
| Invariant | NormalPoint z contact rows | final hard residual |
| Viability | point x acceleration +1 | level RMS + preservation drift |
| Intent | conflicting point x acceleration -1 | lower-level compromise only |
| Resources | generalized-acceleration bounds | minimum bound margin + clipping |
| Solver budget | 64 hard + 32/level fixed sweeps | initial/best/final residual + MaxIterations |
| Backend | CpuExactF64 vs CpuMirrorF32 | separate semantics and D3 |
| Admission | command vs retained candidate | budget exhaustion keeps command zero |

No layer is aggregated. Backend agreement cannot certify physical authority, a lower task cannot overwrite a higher achieved row, and a finite candidate is not an executable command.

## Continuous budget case

| signal | result |
|---|---|
| status | MaxIterations |
| hard residual initial → best → final | 1.25 → 1.0 → 1.0 |
| finite candidate retained | True |
| admitted command zero | True |
| infeasibility conclusion | NONE — MaxIterations is fixed-work exhaustion |

## CPU algorithm witness

| gate | result |
|---|---|
| 500-call complete-output byte replay | True |
| compatible command max absolute delta vs strict f64 | 0.0 |
| maximum admitted hard violation | 0.0 |
| maximum priority-preservation drift | 0.0 |
| typed budget / invalid-problem / invalid-input | True |
| neighbor isolation unit gate | True |
| padding zero | True |
| allocation-free execute unit gate | True |

Generated solve PTX, module loading, device execution, D1/D3, memory, timing, isolation, and direct-versus-Graph gates are **NOT RUN**. The CPU mirror is not substituted for device evidence.

## Admission boundary

R98 promotes only the implemented CUDA solve source/executor capability. This host reports solve NVRTC `library_unavailable` and runtime `no_device`. A CUDA host must retain source/model/descriptor/algorithm/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-input canaries, permutation, padding, chunking, neighbor isolation, memory scaling, timing, error injection, isolated executors, and direct-versus-Graph gates before device admission. Successful CPU solve or CUDA row emission does not satisfy that contract.
