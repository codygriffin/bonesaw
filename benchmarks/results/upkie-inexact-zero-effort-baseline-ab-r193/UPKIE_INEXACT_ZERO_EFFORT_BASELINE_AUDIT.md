# Bonesaw zero-effort terminal baseline A/B · r193

> Mechanism **PASS** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Semantic correction

- Typed withhold executes zero actuator effort. R191 scored it as zero generalized acceleration; r193 instead runs a separate no-contact fixed-zero-effort dynamics query and supplies that admitted acceleration to the unchanged terminal chooser.
- The query is state-local Rust model evaluation. It does not call MuJoCo, integrate a plant, infer future dropout duration, refresh Primary health, or make zero torque executable through a different authority type.
- Stored first-run work: **40,308 control ticks**. Independently re-audited **259** r193 queries.

## Plant consequence

| case | dropout | exact | r191 zero-qdd | r193 zero-effort | r193 fall Δ s |
|---|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 1.790s | FALL 1.790s | -0.680 |
| left_1n | drop10 | FALL 2.470s | FALL 2.735s | FALL 1.715s | -0.755 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.170s | FALL 2.330s | -0.190 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.365s | FALL 2.365s | -0.155 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 5.970s | FALL 3.055s | -1.575 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 4.230s | FALL 4.230s | -0.400 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.635s | FALL 1.635s | +0.025 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.595s | FALL 1.595s | -0.015 |

## First changed terminal decisions

| case | dropout | time s | action old→new | baseline tilt old→new | support-free tilt | baseline joint-v | support-free joint-v | |zero-effort qdd|∞ |
|---|---|---|---|---|---|---|---|---|
| left_1n | drop10 | 1.500 | 0→2 | 0.100→0.393 | 0.254 | 3.445 | 3.313 | 21.630 |
| right_1n_mirror | drop5 | 1.500 | 0→2 | 0.025→0.135 | 0.059 | 5.105 | 2.035 | 17.999 |
| handle_forward_4n | drop5 | 1.250 | 2→0 | 0.941→0.781 | 0.862 | 2.472 | 0.000 | 13.353 |

Action indices are 0 withhold, 1 retained, and 2 support-free. The table reports the first causal choice that changes in each row; it does not average later trajectory divergence.

## Runtime

- Zero-effort dynamics / terminal-selector maxima: **115.519 / 0.972 µs**.
- Full loop: **50** 5 ms overruns; loop/controller maxima **6.938 / 6.616 ms**.
- Fixed-effort query, terminal chooser, and measured controller hot paths report zero Rust allocation; Python GC remains zero.
