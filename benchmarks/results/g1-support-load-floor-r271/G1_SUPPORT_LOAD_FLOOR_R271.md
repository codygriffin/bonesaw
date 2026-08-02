# G1 per-patch aggregate support-load floor experiment · R271

Mechanism implemented; global walking profile rejected. R271 adds a default-off hard aggregate normal-load row to each finite support patch. The row constrains the sum of a foot's four normal-force slots, so load can redistribute across the sole.

## Result

| profile | floor | first bind | fallback | release | min row margin N | violations | root RMS m | p99 ms |
|---|---|---|---|---|---|---|---|---|
| r270 lower body hard baseline | 0.000 | 187 | 875 | 1108 | 0.000000 | 0 | 15.514 | 4.752 |
| r271 dormant zero floor | 0.000 | 187 | 875 | 1108 | 0.000000 | 0 | 15.514 | 4.721 |
| r271 floor005 | 0.005 | 187 | 505 | 505 | 0.000000 | 0 | 24.382 | 4.499 |
| r271 floor10 | 0.100 | 168 | 424 | 424 | -0.000000 | 0 | 25.632 | 4.703 |
| r271 floor25 | 0.250 | 144 | 308 | 308 | -0.000000 | 0 | 28.960 | 3.996 |

The 0% R271 trace is bitwise equal to all 72 retained non-timing R270 arrays: True. Every nonzero trace satisfies every audited per-patch row within 1e-6 N; the smallest 0.5% floor first binds at tick 187 and moves contingency/release from the R270 baseline's 875/1108 to 505/505. The 10% and 25% cases fail at 424 and 308.

That separates mechanism from policy: the generic row works and is dormant at zero, but applying the same minimum continuously to every active walking patch forbids deliberate unloading of the outgoing foot. The first 0.5% bind occurs hundreds of ticks before R270's right-knee limit event at tick 874, so this profile is not a recovery for that event and receives no authority.

Timing is reported per isolated trace but is not a selection criterion. The evaluator performs no policy or physics steps; controller ticks and non-hold integration steps are recorded per profile. All candidate states remain finite, and releases are retained as fail-closed evidence rather than resets hidden from the score.

## Contract

- Hard row: sum of the four compact normal-force slots for one active sole is at least fraction × supported weight ÷ active patches.
- Compact slots are decoded in active-target order, not permanent left/right slots; release and hold statuses are excluded from successful-support claims.
- API fraction is finite in [0, 1] and defaults to zero. The dormant A/B decides whether defaults changed.
- Pinned G1 mass is 32.105857280 kg; supported weight is 314.958459917 N at 9.81 m/s².
- Source contains only the immutable reference, initial morphology witness, and pinned URDF. Policy steps: 0; physics steps: 0.
- Reference SHA-256: `429baba94dae0c3b4e490dc7f0bc107416d3c0559fbfbad4df6e009fac3e48c6`; witness SHA-256: `f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295`; model SHA-256: `9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43`.
