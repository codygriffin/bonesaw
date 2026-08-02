# Bonesaw zero-effort terminal baseline A/B · r192

> Mechanism **PASS** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Semantic correction

- Typed withhold executes zero actuator effort. R191 scored it as zero generalized acceleration; r192 instead runs a separate no-contact fixed-zero-effort dynamics query and supplies that admitted acceleration to the unchanged terminal chooser.
- The query is state-local Rust model evaluation. It does not call MuJoCo, integrate a plant, infer future dropout duration, refresh Primary health, or make zero torque executable through a different authority type.
- Stored first-run work: **40,308 control ticks**. Independently re-audited **259** r192 queries.

## Plant consequence

| case | dropout | exact | r191 zero-qdd | r192 zero-effort | r192 fall Δ s |
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

## Runtime

- Zero-effort dynamics / terminal-selector maxima: **130.405 / 1.072 µs**.
- Full loop: **58** 5 ms overruns; loop/controller maxima **7.137 / 6.791 ms**.
- Fixed-effort query, terminal chooser, and measured controller hot paths report zero Rust allocation; Python GC remains zero.
