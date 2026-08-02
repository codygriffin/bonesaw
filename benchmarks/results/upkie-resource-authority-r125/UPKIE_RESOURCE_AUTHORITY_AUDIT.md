# Bonesaw Upkie resource-authority replay · r125

## Outcome

> **Admission: PASS.** 10,000 immutable Upkie states were queried in nominal, acceleration-derated, effort-derated, and combined resource regimes. No policy, contact estimator, state integration, simulator, or plant model participates.

## Resource sensitivity

| case | statuses | min qdd / effort scale | tracking RMS | min qdd / effort margin | qdd / effort bound queries | dyn / contact L∞ | p50 / p99 / max µs |
|---|---|---|---|---|---|---|---|
| nominal | {'Solved': 1215, 'SolvedWithSlack': 8785} | 1.00 / 1.00 | 90.608 | 0.00e+00 / 1.61e+00 | 5607 / 0 | 3.18e-11 / 5.34e-12 | 111.6 / 143.7 / 191.6 |
| acceleration_derated | {'Solved': 1104, 'SolvedWithSlack': 8896} | 0.22 / 1.00 | 56.996 | 0.00e+00 / 1.62e+00 | 6629 / 0 | 3.18e-11 / 5.34e-12 | 110.2 / 139.5 / 179.4 |
| effort_derated | {'Solved': 979, 'SolvedWithSlack': 9021} | 1.00 / 0.04 | 85.370 | 0.00e+00 / 0.00e+00 | 5112 / 1868 | 6.11e-11 / 1.19e-11 | 113.6 / 353.7 / 467.7 |
| combined | {'Solved': 902, 'SolvedWithSlack': 9098} | 0.22 / 0.04 | 54.921 | 0.00e+00 / 0.00e+00 | 6201 / 1368 | 6.11e-11 / 1.19e-11 | 112.0 / 344.5 / 452.2 |

Each scale is external state-local authority in `[0,1]`, not inferred temperature, reliability, policy, or body response. Rust applies it directly to the configured bounds before solving; task residual remains the continuous record of motion that could not be tracked.

## Boundary checks

| check | result |
|---|---|
| nominal all-ones equals absent scales | True |
| combined repeat bitwise exact | True |
| combined reverse-order exact | True |
| combined four-chunk exact | True |
| inputs immutable | True |
| all hot-loop allocation calls / bytes | 0 / 0 |
| RSS before / after / delta | 96.97 / 152.22 / 55.25 MiB |
| whole replay wall / CPU | 9.548 / 9.546 s |
| GC collections | 8 |

## Atomic invalid-scale probes

| fault | typed ValueError | outputs unchanged |
|---|---|---|
| negative_acceleration_scale | True | True |
| above_one_acceleration_scale | True | True |
| nonfinite_acceleration_scale | True | True |
| negative_effort_scale | True | True |
| above_one_effort_scale | True | True |
| nonfinite_effort_scale | True | True |

The report distinguishes hierarchy compromise from declared resource exhaustion. It still does not prove that a real actuator realizes the command; calibrated electrical/thermal models, observation transport, and plant tracking remain separate authority rows.
