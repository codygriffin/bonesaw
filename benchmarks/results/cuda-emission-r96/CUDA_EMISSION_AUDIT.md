# Bonesaw CUDA fixed row emission · r96

## Outcome

**SOURCE + CPU ROW WITNESS PASS; COMPILER/DEVICE UNAVAILABLE.** R96 extends the fixed-buffer CUDA slice through point-attractor and contact-lock row emission. StateInput → FK/CoM → Jacobians → Dynamics → PointQueries → Emission launches in one ordered stream and synchronizes once. Query slots, task bandwidths, and contact modes are immutable construction-time constants; runtime supplies only fixed-shape activation masks and target jets.

This remains a **row-production pipeline**, not a CUDA WBC solve. Hierarchical solve, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unavailable.

## Independent backend authority levels

| authority level | result | meaning |
|---|---|---|
| source / ABI | PASS | five fixed entries; no atomics, device allocation, or block barrier |
| CPU row witness | PASS | 200-call emission replay plus independent formulas |
| NVRTC FK/CoM | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Jacobians | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Dynamics | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Point queries | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| NVRTC Emission | LIBRARY_UNAVAILABLE | NVRTC library is unavailable |
| CUDA runtime | NO_DEVICE | 0 visible device(s) |
| device D1/D3 | NOT RUN | never inherited from source or CPU checks |
| full CudaMirrorF32 / Graph | False / False | solve and later stages remain gated |

## Frozen emission boundary

| signal | value |
|---|---|
| stage manifest bits | 6359 |
| emission source SHA-256 | da915a69c439b222dd2422cb90ca02895262267fcc33d65883c06b018d566b18 |
| fixed topology | 2 point tasks · 4 contact locks · 4 point sites |
| batch layout | 24 generalized coordinates · capacity 17 · stride 32 |
| contact modes covered | LockedPoint · NormalPoint · RollingPoint · Disabled |
| pipeline synchronization points | 1 |
| provisional device D3 abs+rel tolerance | 5e-4 over every emitted floating row field |
| hidden CPU fallback | False |

Point tasks emit stable position error, velocity error, desired acceleration, Jacobian, and bias-corrected right-hand side. Contact locks copy only axes enabled by their compiled kinematic mode. Inactive, invalid, disabled-axis, and padded rows remain exactly zero.

## CPU row witness

| gate | result |
|---|---|
| 200-call complete emission-byte replay | True |
| independent formula oracle | True |
| maximum formula error | 0.0 |
| malformed mask and active NaN typed invalid | True |
| invalid-agent neighbor isolation exact | True |
| inactive and mode-filtered rows zero | True |
| padding inactive and zero | True |
| allocation-free hot-path unit gate | True |

Generated PTX and device D1/D3, timing, memory, isolation, and Graph gates are **NOT RUN**. CPU execution is not substituted for a device pass.

## Example authority stack retained for architecture review

| layer | example authority | separate witness |
|---|---|---|
| Invariant | floating dynamics + compiled contact rows | hard residual; row emission is not solve feasibility |
| Viability | finite support and joint stopping | reserve and limiting stable ID remain distinct |
| Intent | hip/torso/handle point rows | position, velocity, desired acceleration, J, and rhs |
| Resources | contact force, actuator effort, power, thermal | continuous headroom; no pose promise |
| Solver budget | iteration/time budget | not represented by emission success |
| Backend | source → CPU → compiler → runtime → device | no aggregate health boolean |

## Admission boundary

R96 promotes only the implemented row-emission stage capability. This host reports emission NVRTC `library_unavailable` and runtime `no_device`. A CUDA CI host must retain source/model/query/task/contact/PTX/toolkit/driver/GPU fingerprints and pass D1, D3, malformed-input canaries, permutation, padding, chunking, memory scaling, timing, error injection, isolated-executor, and direct-versus-Graph gates before device admission. Successful row production does not certify a compatible hierarchical solve or physical realizability.
