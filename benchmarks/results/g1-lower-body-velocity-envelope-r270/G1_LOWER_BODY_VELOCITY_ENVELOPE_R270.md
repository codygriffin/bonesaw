# G1 lower-body velocity-envelope causal split · R270

**Causal safety mechanism PASS / walking profile REJECTED.** This replay uses no policy and no physics simulator. It separately measures the soft viability task, the hard directional braking bound, their combination, and both R269 compositions. The opt-in profile derives a lower-body coordinate allowlist from the pinned URDF joint names and activates at 50% velocity utilization. The default controller is unchanged.

## Result

| profile | first fallback | first release | prefix root RMS cm | full root RMS m | max attitude deg | p99 ms |
|---|---|---|---|---|---|---|
| r268 low gain baseline | 863 | 869 | 1.845 | 18.879 | 40.48 | 4.911 |
| r269 bounded continuation | 863 | 888 | 1.845 | 17.026 | 120.38 | 4.844 |
| r270 dormant r269 stack | 863 | 888 | 1.845 | 17.026 | 120.38 | 4.743 |
| r270 lower body soft only | 864 | 869 | 1.858 | 17.571 | 19.78 | 4.726 |
| r270 lower body hard only | 888 | 889 | 2.650 | 19.485 | 89.90 | 5.146 |
| r270 lower body soft plus hard | 875 | 1108 | 2.099 | 15.514 | 3.53 | 4.752 |
| r270 hard only composed r269 stack | 888 | 900 | 2.650 | 17.923 | 26.00 | 4.693 |
| r270 soft plus hard composed r269 stack | 875 | 897 | 2.099 | 16.334 | 82.64 | 5.457 |

The hard-bound-only control establishes causality: it moves first fallback 863→888 and clips the pinned right-knee coordinate 9 to -4.097 rad/s in the critical window. Soft-only moves fallback only to tick 864 and reaches -7.425 rad/s. The early window has zero activations in every opt-in control.

The long release delay is an interaction, not a hard-bound-only result. Hard-only releases at tick 889; soft-only releases at 869; soft+hard reaches tick 1108. It retains 336.3 N one tick before release but has already reached the right-knee lower position limit at tick 874. This localizes the next problem to continuous position-limit/support-transition recovery.

The interaction is not compositional yet. Adding R269 continuation and the predeclared target-1 handoff preserves the tick-875 combined fallback but releases at tick 897, versus 1108 standalone, and records 5.457 ms p99. Hard-only plus R269 releases at tick 900. R269 and the soft+hard R270 profile therefore remain alternative diagnostics, not one admitted controller profile.

This is not a walking pass: combined full-run root RMS remains 15.514 m and every opt-in variant releases support before the end of the trace. The combined candidate records 4.752 ms p99 and 0 20 ms misses; timing is reported per retained distribution, not borrowed between variants.

## Contract

- The directional hard braking bound and soft viability task are independently switchable, opt-in, and lower-body-only; a zero soft weight is a valid hard-only causal control.
- A conflicting hard braking interval is carried to the solver and fails closed as `InvalidProblem`; it is never silently weakened or skipped.
- With the new fields dormant, all 72 non-timing trace arrays are bitwise equal to R269 (NaNs compared positionally).
- Composition with R269 is explicitly rejected; no benchmark row may borrow the standalone release time and the composed timing result.
- The profile consumes only the immutable reference and initial morphology witness; no future policy, oracle force, solved acceleration, or simulator state is used.
- Status-5 release telemetry remains fail-closed with zero rejected dynamics/contact residual witnesses.
- Reference SHA-256: `429baba94dae0c3b4e490dc7f0bc107416d3c0559fbfbad4df6e009fac3e48c6`; witness SHA-256: `f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295`.
