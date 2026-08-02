# Bonesaw CUDA FK + center of mass · r92

## Outcome

**SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE.** R92 adds the first generic model-product device boundary after R91 StateInput: a packed immutable tree ABI, NVRTC-loaded CUDA source, a fixed-buffer Driver API executor, ordered StateInput→FK/CoM launches, and one complete agent per CUDA thread. The hot Rust call owns no allocation and never falls back to the CPU.

This is deliberately a **StateInput + ForwardKinematics + CenterOfMass** implementation claim. Jacobians, dynamics, point products, row emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable on device.

## Independent authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | entry point, ownership, no atomic/allocation/barrier policy |
| CPU mirror witness | PASS | semantic and deterministic reference only |
| NVRTC compiler | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device D1/D2 | NOT RUN | never inherited from CPU or source checks |
| full CudaMirrorF32 / Graph | False / False | later stages remain gated |

These rows are intentionally not collapsed into one backend-health boolean. A source implementation can exist while its host compiler or device evidence is unavailable.

## Compiled boundary

| signal | value |
|---|---|
| stage manifest bits | 7 |
| CUDA source SHA-256 | 35df5864625436d33350cbd382daa5215a550c979e74cd417b2cab6ab4e72532 |
| packed model SHA-256 | 685662c9f66ae34a5de3c07325204842fb19fc5215e58b06d501f88da1f593f5 |
| NVRTC target | 7.5 |
| NVRTC toolkit | None |
| generated PTX bytes | None |
| layout | 20 joints · 21 bodies · 18 coordinates · capacity 17 · stride 32 |
| hidden CPU fallback | False |

The packed model uses joint AoS constants because all warp lanes read the same joint record, while state and output remain agent-minor SoA. Each thread walks the same topologically compiled tree, writes all body poses, and performs its own mass-weighted CoM reduction. There are no atomics or cross-agent reductions.

## CPU mirror witness

| gate | result |
|---|---|
| 200-call complete-output bitwise repeat | True |
| active agents typed OK | True |
| malformed agent typed invalid | True |
| malformed neighbor isolation exact | True |
| padding inactive and zero | True |
| allocation-free hot-path Rust unit gate | True |

NVRTC compilation and device D1/D2, launch, timing, isolation, and memory gates are **NOT RUN**. The report does not substitute CPU execution or call the FK/CoM stage device-certified.

## Admission boundary

R92 promotes `cuda_mirror_fk_com_f32` only as an implemented stage capability; it does not promote the full CUDA backend. This host reports NVRTC `library_unavailable` and runtime `no_device`. A CUDA CI host must retain the source/model/toolkit/driver/GPU fingerprint and pass D1 repeated bytes, D2 CPU-mirror tolerance, invalid-neighbor canaries, permutation/padding/chunking, memory scaling, direct-launch timing, error injection, and direct-versus-Graph equivalence before device admission.
