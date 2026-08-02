# Bonesaw CUDA fixed point products · r95

## Outcome

**SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE.** R95 extends the fixed-buffer CUDA slice through compiler-resolved point position, floating point Jacobian, and kinematic bias acceleration `Jdot-v`. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries launches in one ordered stream and synchronizes once. Query frame indices and local offsets are immutable construction-time constants.

This remains a **model-product pipeline**, not a CUDA WBC. Task/constraint emission, hierarchical solve, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | four fixed entries; no atomics, device allocation, or block barrier |
| CPU mirror witness | PASS | 200-call complete point-product reference only |
| NVRTC FK/CoM | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Jacobians | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Dynamics | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Point queries | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device D1/D3 | NOT RUN | never inherited from source or CPU checks |
| full CudaMirrorF32 / Graph | False / False | later stages remain gated |

## Pipeline boundary

| signal | value |
|---|---|
| stage manifest bits | 6167 |
| point source SHA-256 | bbb38e10ed6c09e6a4a442b6ebf95264360fce2ab35db3cc84415dbb0faf337f |
| query sites | 4 |
| layout | 21 bodies · 24 generalized coordinates · capacity 17 · stride 32 |
| pipeline synchronization points | 1 |
| provisional device D3 abs+rel tolerance | 2e-4 for position, J, and Jdot-v |
| hidden CPU fallback | False |

## CPU mirror witness

| gate | result |
|---|---|
| 200-call point-product complete-byte repeat | True |
| active agents typed OK | True |
| padding inactive and zero | True |
| allocation-free hot-path unit gate | True |
| f64 position/J/Jdot-v oracle gate | True |

Generated PTX and device D1/D3, timing, memory, isolation, and Graph gates are **NOT RUN**. CPU execution is not substituted for a device pass.

## Example authority stack retained for architecture review

| layer | example authority | separate witness |
|---|---|---|
| Invariant | floating dynamics + locked support | hard residual; never inferred from point-query success |
| Viability | finite CoM/support and joint stopping | reserve, limiting stable ID, and recovery remain distinct |
| Intent | hip/torso/handle point tasks | world position, J, Jdot-v, and task residual |
| Resources | contact force, actuator effort, power, thermal | separate continuous headroom; no pose promise |
| Solver budget | iteration/time budget | bounded-work status is not infeasibility |
| Backend | source → CPU → compiler → runtime → device | no aggregate health boolean |

## Admission boundary

R95 promotes only the implemented point-product stage capability. This host reports point-query NVRTC `library_unavailable` and runtime `no_device`. A CUDA CI host must retain source/model/query/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-neighbor canaries, permutation, padding, chunking, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission.
