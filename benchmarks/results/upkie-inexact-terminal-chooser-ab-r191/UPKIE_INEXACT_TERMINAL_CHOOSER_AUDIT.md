# Bonesaw three-way terminal chooser A/B · r191

> Mechanism **PASS** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Plant consequence

- Stored first-run work: **62,817 control ticks** across **77 profiles**, each with exact replay.
| case | dropout | exact | terminal chooser | fall Δ s |
|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 1.790s | -0.680 |
| left_1n | drop10 | FALL 2.470s | FALL 2.735s | +0.265 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.170s | -0.350 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.365s | -0.155 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 5.970s | +1.340 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 4.230s | -0.400 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.635s | +0.025 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.595s | -0.015 |

## Independent physical impact audit

- Independently re-audited **481** online terminal queries at a declared root-impact plane of **0.225 m**; full candidate diagnostics and selections disagree on **0** ticks.
- Ballistic time, vertical impact velocity, and vertical specific energy are candidate-invariant by construction. Candidate commands affect only terminal tilt/rate, joint headroom/speed, effort, and admission diagnostics.
- The J/kg field is only `0.5 vz²`; no horizontal or rotational energy is invented without mass/inertia evidence, and the audit is not a collision-impulse or injury certificate.

## Runtime

- Online selector / independent re-audit maxima: **8.106 / 0.972 µs**.
- Full loop: **115** 5 ms overruns; loop/controller maxima **7.056 / 6.730 ms**.
- Rust hot-path allocation and Python GC inside measured controller execution: **zero**.
