# Bonesaw inexact-hold authority sweep · r186

> Mechanism **PASS** · continuous consequence **REJECTED** · synchronous profile **REJECTED**. One fixed first-loss effort fraction is applied to both isolated 5 ms and first-of-two 10 ms missing samples.

## Authority summary

| authority | earlier | worst Δ s | new green falls | verdict |
|---|---|---|---|---|
| 0.00 | 5 | -1.245 | 0 | REJECTED |
| 0.25 | 3 | -1.415 | 0 | REJECTED |
| 0.50 | 4 | -0.575 | 1 | REJECTED |
| 0.75 | 5 | -1.780 | 0 | REJECTED |
| 1.00 | 3 | -1.655 | 0 | REJECTED |

## Runtime resource gate

- 5 ms loop overruns: **131** (required: 0).
- Worst loop: **7.426 ms**.
- Worst controller call: **6.955 ms** (required: ≤5.000 ms).
- Timed Rust allocation and Python GC collections: **zero**.

## Case detail

| case | dropout | authority | exact | candidate | fall Δ s |
|---|---|---|---|---|---|
| nominal | drop5 | 0.00 | RECOVERED | RECOVERED | — |
| nominal | drop10 | 0.00 | RECOVERED | RECOVERED | — |
| nominal | drop5 | 0.25 | RECOVERED | RECOVERED | — |
| nominal | drop10 | 0.25 | RECOVERED | RECOVERED | — |
| nominal | drop5 | 0.50 | RECOVERED | RECOVERED | — |
| nominal | drop10 | 0.50 | RECOVERED | RECOVERED | — |
| nominal | drop5 | 0.75 | RECOVERED | RECOVERED | — |
| nominal | drop10 | 0.75 | RECOVERED | RECOVERED | — |
| nominal | drop5 | 1.00 | RECOVERED | RECOVERED | — |
| nominal | drop10 | 1.00 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | 0.00 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | 0.00 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | 0.25 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | 0.25 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | 0.50 | RECOVERED | FALL 2.495s | — |
| forward_4n_reference | drop10 | 0.50 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | 0.75 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | 0.75 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | 1.00 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | 1.00 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | 0.00 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | 0.00 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | 0.25 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | 0.25 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | 0.50 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | 0.50 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | 0.75 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | 0.75 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | 1.00 | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | 1.00 | RECOVERED | RECOVERED | — |
| left_1n | drop5 | 0.00 | FALL 2.470s | FALL 1.785s | -0.685 |
| left_1n | drop10 | 0.00 | FALL 2.470s | FALL 2.725s | +0.255 |
| left_1n | drop5 | 0.25 | FALL 2.470s | FALL 1.540s | -0.930 |
| left_1n | drop10 | 0.25 | FALL 2.470s | FALL 1.715s | -0.755 |
| left_1n | drop5 | 0.50 | FALL 2.470s | FALL 2.325s | -0.145 |
| left_1n | drop10 | 0.50 | FALL 2.470s | FALL 2.475s | +0.005 |
| left_1n | drop5 | 0.75 | FALL 2.470s | FALL 2.570s | +0.100 |
| left_1n | drop10 | 0.75 | FALL 2.470s | FALL 2.220s | -0.250 |
| left_1n | drop5 | 1.00 | FALL 2.470s | FALL 2.430s | -0.040 |
| left_1n | drop10 | 1.00 | FALL 2.470s | FALL 4.175s | +1.705 |
| right_1n_mirror | drop5 | 0.00 | FALL 2.520s | FALL 2.170s | -0.350 |
| right_1n_mirror | drop10 | 0.00 | FALL 2.520s | FALL 2.365s | -0.155 |
| right_1n_mirror | drop5 | 0.25 | FALL 2.520s | FALL 3.065s | +0.545 |
| right_1n_mirror | drop10 | 0.25 | FALL 2.520s | FALL 3.415s | +0.895 |
| right_1n_mirror | drop5 | 0.50 | FALL 2.520s | FALL 3.340s | +0.820 |
| right_1n_mirror | drop10 | 0.50 | FALL 2.520s | FALL 2.510s | -0.010 |
| right_1n_mirror | drop5 | 0.75 | FALL 2.520s | FALL 1.560s | -0.960 |
| right_1n_mirror | drop10 | 0.75 | FALL 2.520s | FALL 2.000s | -0.520 |
| right_1n_mirror | drop5 | 1.00 | FALL 2.520s | FALL 3.035s | +0.515 |
| right_1n_mirror | drop10 | 1.00 | FALL 2.520s | FALL 2.505s | -0.015 |
| handle_forward_4n | drop5 | 0.00 | FALL 4.630s | FALL 3.385s | -1.245 |
| handle_forward_4n | drop10 | 0.00 | FALL 4.630s | UNSETTLED | — |
| handle_forward_4n | drop5 | 0.25 | FALL 4.630s | RECOVERED | — |
| handle_forward_4n | drop10 | 0.25 | FALL 4.630s | FALL 3.215s | -1.415 |
| handle_forward_4n | drop5 | 0.50 | FALL 4.630s | FALL 4.055s | -0.575 |
| handle_forward_4n | drop10 | 0.50 | FALL 4.630s | FALL 5.445s | +0.815 |
| handle_forward_4n | drop5 | 0.75 | FALL 4.630s | FALL 2.850s | -1.780 |
| handle_forward_4n | drop10 | 0.75 | FALL 4.630s | FALL 2.915s | -1.715 |
| handle_forward_4n | drop5 | 1.00 | FALL 4.630s | FALL 5.200s | +0.570 |
| handle_forward_4n | drop10 | 1.00 | FALL 4.630s | FALL 2.975s | -1.655 |
| forward_4n_friction_0p03 | drop5 | 0.00 | FALL 1.610s | FALL 1.700s | +0.090 |
| forward_4n_friction_0p03 | drop10 | 0.00 | FALL 1.610s | FALL 1.595s | -0.015 |
| forward_4n_friction_0p03 | drop5 | 0.25 | FALL 1.610s | FALL 1.635s | +0.025 |
| forward_4n_friction_0p03 | drop10 | 0.25 | FALL 1.610s | FALL 1.695s | +0.085 |
| forward_4n_friction_0p03 | drop5 | 0.50 | FALL 1.610s | FALL 1.720s | +0.110 |
| forward_4n_friction_0p03 | drop10 | 0.50 | FALL 1.610s | FALL 1.605s | -0.005 |
| forward_4n_friction_0p03 | drop5 | 0.75 | FALL 1.610s | FALL 1.715s | +0.105 |
| forward_4n_friction_0p03 | drop10 | 0.75 | FALL 1.610s | FALL 1.650s | +0.040 |
| forward_4n_friction_0p03 | drop5 | 1.00 | FALL 1.610s | FALL 1.635s | +0.025 |
| forward_4n_friction_0p03 | drop10 | 1.00 | FALL 1.610s | FALL 1.630s | +0.020 |

## Contract

- Authority is Q15 Rust state, not a Python post-process. Quarter fractions are represented exactly and scale the cached effort without compounding.
- The first missing tick receives the same fraction whether the next sample returns or is also unavailable. No future burst-duration oracle is used.
- Zero authority disables retained authority and follows withheld semantics; full authority reproduces r185's one-tick hold.
- Strictly admitted authority values: **none**.
