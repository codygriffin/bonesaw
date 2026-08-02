# Bonesaw G1 actuator-realization envelope · r62

## Result

The admitted r54 WBC effort trace is replayed through six declared Rust command-response envelopes without a robot policy, state integration, rigid-body physics, or contact simulation. The only persistent state is realized actuator effort. Infinite-bandwidth/infinite-slew control reproduces every requested effort bit-exactly. Finite profiles expose bandwidth lag, slew limiting, availability clipping, power mismatch, persistence, and contact-edge recovery as separate signals.

> All finite profiles are synthetic sensitivity cases, not Unitree G1 actuator calibration. This report quantifies how much command-path authority would be required; it does not claim that the real robot has any listed bandwidth, slew rate, or derating fraction.

## Capability matrix

| case | bandwidth / rate / availability | RMS error Nm | p99 / max normalized error | availability / slew ticks | longest >5% | edge recovery max | p99 Rust step |
|---|---|---|---|---|---|---|---|
| ideal_control | infinite Hz / infinite Nm/s / 100% | 0.0000 | 0.000% / 0.000% | 0 / 0 | 0 ms | 0 ms | 2.222 µs |
| fast_synthetic | 20.0 Hz / 5000.0 Nm/s / 100% | 0.5351 | 3.083% / 27.351% | 0 / 0 | 15 ms | 10 ms | 2.223 µs |
| medium_synthetic | 10.0 Hz / 1000.0 Nm/s / 100% | 0.7323 | 3.954% / 36.740% | 0 / 0 | 40 ms | 25 ms | 2.144 µs |
| slow_synthetic | 5.0 Hz / 250.0 Nm/s / 100% | 0.9595 | 4.861% / 45.172% | 0 / 108 | 50 ms | 55 ms | 0.441 µs |
| half_available | 10.0 Hz / 1000.0 Nm/s / 50% | 0.7323 | 3.954% / 37.115% | 2 / 0 | 40 ms | 25 ms | 0.531 µs |
| quarter_available_slow | 5.0 Hz / 250.0 Nm/s / 25% | 1.0657 | 8.585% / 46.715% | 1623 / 108 | 2145 ms | 55 ms | 0.521 µs |

## Interpretation

- Stateless r54 admission remains the authority for instantaneous dynamics/contact/friction/CoP/effort feasibility; realization is a separately scored downstream boundary.
- Normalized effort error uses each actuator's authored URDF effort limit, so joints with different units of authority are comparable without forming one aggregate health score.
- Availability clipping and slew limiting are causal flags from the Rust step, while bandwidth lag is retained continuously in the error trace and dwell curves.
- Mechanical power is observational `effort × oracle velocity`; electrical/thermal/reliability claims still require calibrated motor and driver profiles plus telemetry.
- A future constrained forward-dynamics surrogate may map realized effort error to acceleration tracking, but it must remain separate from this physics-free command-path certificate and from integrated simulation.

## Mechanism gates

- PASS `source_oracle_remains_immutable`
- PASS `ideal_control_is_bit_exact`
- PASS `all_scenarios_repeat_bit_exactly`
- PASS `all_rust_steps_are_allocation_free`
- PASS `all_realized_efforts_respect_declared_availability`

## Artifacts

`actuator-realization-raw.npz` retains every realized effort, error, causal clipping flag, timing, and allocation trace for all six cases. `actuator-realization-metrics.json` retains per-actuator leaders, power/energy observations, dwell, edge recovery, evaluation boundary, and source checksums. The CSV is the compact capability curve.
