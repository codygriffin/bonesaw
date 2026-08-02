# Bonesaw CUDA kinematics pipeline · r93

## Outcome

**SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE.** R93 extends the generic device slice through floating-root frame-origin Jacobians and the center-of-mass Jacobian. The fixed-buffer executor now launches StateInput → FK/CoM → Jacobians in one stream, synchronizes once, and copies fixed outputs. One CUDA thread owns one complete agent; no stage performs a cross-agent reduction.

This remains a **kinematics-only** device implementation. Dynamics, point products, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | two fixed entries, fixed ownership, no atomics/allocation/barrier |
| CPU mirror witness | PASS | semantic and deterministic reference only |
| NVRTC FK/CoM | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Jacobians | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device D1/D3 | NOT RUN | never inherited from CPU/source checks |
| full CudaMirrorF32 / Graph | False / False | later stages remain gated |

The authority rows remain independent. Implemented code, a CPU reference, a compiler, and a conforming device are different facts.

## Pipeline boundary

| signal | value |
|---|---|
| stage manifest bits | 23 |
| FK/CoM source SHA-256 | 35df5864625436d33350cbd382daa5215a550c979e74cd417b2cab6ab4e72532 |
| Jacobian source SHA-256 | 04bd26e8e11cec2444adf3b6d9618b4fb41958183261f871b26a461bb5c0792b |
| packed model SHA-256 | 38d59f5067612e3479b0e0395333a5e0eb16d03be2acef1e05e1b14dbc5527bf |
| compiler target | 7.5 |
| layout | 20 joints · 21 bodies · 24 generalized coordinates · capacity 17 · stride 32 |
| pipeline synchronization points | 1 |
| hidden CPU fallback | False |

The model pack now includes a constant parent-joint index for every body. Each agent thread walks its ancestor chain in deterministic order, constructs root and joint columns in `[angular; linear]` convention, and accumulates body-mass CoM columns with explicit FMA.

## CPU mirror witness

| gate | result |
|---|---|
| 200-call FK+Jacobian complete-byte repeat | True |
| active agents typed OK | True |
| malformed agent typed invalid | True |
| malformed neighbor isolation exact | True |
| padding inactive and zero | True |
| allocation-free hot-path unit gate | True |
| Pinocchio + finite-difference f64 gate | True |

NVRTC compilation and device D1/D3, launch, timing, isolation, memory, and Graph gates are **NOT RUN**. CPU execution is not substituted for a device pass.

## Admission boundary

R93 promotes only the implemented Jacobian-stage capability. This host reports FK/CoM NVRTC `library_unavailable`, Jacobian NVRTC `library_unavailable`, and runtime `no_device`. A CUDA CI host must retain the model/source/PTX/toolkit/driver/GPU fingerprint and pass D1 repeat, D3 abs+rel tolerance, permutation, padding, chunking, cross-agent canaries, memory scaling, direct-launch timing, error injection, isolated-executor equivalence, and direct-versus-Graph equivalence before device admission.
