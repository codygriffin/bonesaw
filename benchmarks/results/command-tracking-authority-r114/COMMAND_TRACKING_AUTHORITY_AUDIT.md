# Bonesaw observed-versus-commanded authority · r114

## Outcome

**PASS.** R114 turns the existing command-divergence diagnostic into a two-stage authority envelope. The observed joint state is mapped through the exact actuation transform, then compared with the commanded splice position and velocity. Crossing a contingency threshold withholds feed-forward effort and selects the independently validated brake. Crossing a reject threshold withholds both plans. This evaluates a controller contract only; it introduces no policy, physics engine, state integration, or claim that the plant follows the command.

| case | position / velocity error | tracking action | selection | flags | position contingency / reject headroom | velocity contingency / reject headroom |
|---|---|---|---|---|---|---|
| nominal | 0.005 / 0.100 | nominal | primary | 0x0 | 0.005 / 0.025 | 0.100 / 0.400 |
| position_contingency | 0.020 / 0.100 | contingency | contingency | 0x40000 | -0.010 / 0.010 | 0.100 / 0.400 |
| position_rejected | 0.040 / 0.100 | rejected | rejected | 0x80000 | -0.030 / -0.010 | 0.100 / 0.400 |
| velocity_contingency | 0.005 / 0.300 | contingency | contingency | 0x40000 | 0.005 / 0.025 | -0.100 / 0.200 |
| velocity_rejected | 0.005 / 0.600 | rejected | rejected | 0x80000 | 0.005 / 0.025 | -0.400 / -0.100 |

The configured position thresholds are `0.010/0.030` actuator units and velocity thresholds are `0.200/0.500` actuator units/s. Headroom remains signed and continuous on both sides of each decision. Limiting actuator indices are retained independently for position and velocity.

## Execution evidence

Across 10000 retained transactions, timing was `7.143/12.063/73.249` µs p50/p99/max. Semantic replay was exact: **True**. Timed allocation calls/bytes were `0/0`. An inverted contingency/reject envelope is rejected at construction: **True**.

## Authority boundary

This is direct evidence that observation and command have diverged, not an explanation of why. Torque saturation, bandwidth, delay, contact loss, calibration error, thermal derating, and mechanical failure remain separately typed causes. A warning selects braking but does not certify that the physical body will realize that brake; a hard breach rejects because command-space geometry is no longer an adequate witness for the observed mechanism state.
