# G1 NormalFallback point-task scale causal split · R272

**Mechanism partial / walking profile rejected.** R272 makes the viability point-task weight retained during `NormalFallback` an explicit, default-preserving scale. The scale is applied only to the already typed fallback task; it adds no solver rows, policy step, or physics simulation.

## Result

| profile | scale | first fallback | first release | critical v min | knee limit tick | root RMS m | p99 ms |
|---|---|---|---|---|---|---|---|
| r270 baseline | 1.00 | 875 | 1108 | -4.097 | 874 | 15.514 | 4.752 |
| r272 dormant | 1.00 | 875 | 1108 | -4.097 | 874 | 15.514 | 4.701 |
| r272 scale0 | 0.00 | 875 | 899 | -4.097 | 874 | 15.532 | 5.062 |
| r272 scale025 | 0.25 | 875 | 954 | -4.097 | 874 | 17.030 | 4.678 |
| r272 scale05 | 0.50 | 875 | 1061 | -4.097 | 874 | 14.691 | 5.162 |

The scale-1 dormant replay is bitwise equal to the R270 baseline on all 72 retained non-timing arrays: True. More importantly, every reduced-scale candidate is bitwise equal to baseline through tick 875 on every non-timing array. All four profiles therefore reach the right-knee lower limit at tick 874, enter fallback at tick 875, and share the same -4.097 rad/s critical-window knee minimum. A post-fallback task cannot repair its own pre-fallback cause.

After that common boundary, the scale is observably causal but strictly worse for support duration: scale 0 releases at tick 899, scale 0.25 at 954, and scale 0.5 at 1061, all before the default's 1108. Full root RMS remains 14.691–17.030 m. Timing is reported per trace but cannot rescue the behavioral rejection. No scale is admitted.

## Contract

- The field defaults to 1.0, is finite/nonnegative, and only scales the existing soft point task after a measured `NormalFallback` phase.
- Hard contact rows, force cones, acceleration bounds, and fail-closed release semantics are unchanged.
- The replay consumes only the immutable reference and initial morphology witness; policy and physics step counts are zero.
- Reference SHA-256: `429baba94dae0c3b4e490dc7f0bc107416d3c0559fbfbad4df6e009fac3e48c6`; witness SHA-256: `f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295`.
