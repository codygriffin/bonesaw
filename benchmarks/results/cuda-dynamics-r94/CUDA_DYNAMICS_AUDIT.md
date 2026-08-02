# Bonesaw CUDA floating dynamics · r94

## Outcome

**SOURCE + CPU WITNESS PASS; COMPILER/DEVICE UNAVAILABLE.** R94 extends the fixed-buffer device slice through floating `M(q)`, `h(q,v,g)`, and `Ag(q)`. StateInput → FK/CoM → Jacobians → Dynamics launches in one ordered stream and synchronizes once. One CUDA thread owns one complete agent and all body reductions remain in deterministic body/coordinate order.

This is still a **model-product pipeline**, not the full WBC. Point products, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | three fixed entries; no atomics, device allocation, or block barrier |
| CPU mirror witness | PASS | 200-call complete-product reference only |
| NVRTC FK/CoM | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Jacobians | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Dynamics | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device D1/D3 | NOT RUN | never inherited from source or CPU checks |
| full CudaMirrorF32 / Graph | False / False | later stages remain gated |

## Pipeline boundary

| signal | value |
|---|---|
| stage manifest bits | 2071 |
| FK/CoM source SHA-256 | 35df5864625436d33350cbd382daa5215a550c979e74cd417b2cab6ab4e72532 |
| Jacobian source SHA-256 | 04bd26e8e11cec2444adf3b6d9618b4fb41958183261f871b26a461bb5c0792b |
| Dynamics source SHA-256 | 03ca7c24e31c9fe34e91e1b64f2bd41def6437288656a61a418156571bbe6c77 |
| packed model SHA-256 | e229d591beb0d4494b60ad721f66d83e0ac3fb0e0fce54bbc57a71980dfd0286 |
| layout | 20 joints · 21 bodies · 24 generalized coordinates · capacity 17 · stride 32 |
| pipeline synchronization points | 1 |
| provisional device D3 abs+rel tolerance | 5e-4 for M, h, and Ag |
| hidden CPU fallback | False |

The immutable model pack now includes each body's row-major inertia about its center of mass. The device recursion consumes world-expressed root/joint velocity and per-agent gravity, then assembles the symmetric mass matrix, bias covector, and centroidal map from the admitted frame Jacobians with explicit FMA.

## CPU mirror witness

| gate | result |
|---|---|
| 200-call FK+J+M/h/Ag complete-byte repeat | True |
| active agents typed OK | True |
| invalid velocity typed invalid | True |
| invalid neighbor isolation exact | True |
| padding inactive and zero | True |
| allocation-free hot-path unit gate | True |
| Pinocchio + physical-identity f64 gate | True |

Generated PTX and device D1/D3, timing, isolation, memory, and Graph gates are **NOT RUN**. CPU execution is not substituted for a device pass.

## Admission boundary

R94 promotes only the implemented floating-dynamics stage capability. This host reports NVRTC FK/CoM `library_unavailable`, Jacobians `library_unavailable`, Dynamics `library_unavailable`, and runtime `no_device`. A CUDA CI host must retain source/model/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, permutation, padding, chunking, canaries, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission.
