# Bonesaw analytic dynamic admission faults · r100

## Outcome

**PASS.** R100 adds analytic position extrema to every quintic and validates the actuator polynomial after exact transmission mapping in generalized joint coordinates. Endpoint-safe but interior-unsafe motion is rejected. Admission emits composable fixed flags, so solver, expiry, actuator derivative, and joint-position causes do not collapse into one health score.

This corpus remains policy- and physics-free. It evaluates command safety and fallback selection; it does not claim the robot realized the command.

## Retained cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`; status `2` is contingency.

| case | selection | status | flags | primary / contingency valid | selected position headroom | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|---|
| nominal | 0 | 0 | 0x00 | True / True | 1.259999369415348 | 134.669 / 179.666 / 184.328 | 0 / 0 |
| primary_position_limit | 1 | 2 | 0x11 | False / True | 0.0 | 173.818 / 198.064 / 199.076 | 0 / 0 |
| expired_plan | 1 | 2 | 0x04 | True / True | 1.2599987388306957 | 127.561 / 147.120 / 150.264 | 0 / 0 |

The near-limit case starts Upkie's left hip at `1.2595 rad` with `+0.05 rad/s` and asks for `+100 rad/s²`. Its strict dynamics/contact solve remains finite, but the primary quintic crosses the `1.26 rad` joint limit at an analytic interior extremum. Flag `0x10` activates, primary is withheld, the braking contingency retains nonnegative position headroom, and admitted feed-forward effort is exactly zero.

The expiry case deliberately waits beyond the previous 20 ms plan. Flag `0x4` activates and independently valid braking is selected. All three cases replay semantic output bytes exactly across 100 resets, allocate zero bytes inside the Rust transaction, satisfy hard dynamics/contact residuals below `1e-7`, and remain inside the 20 ms budget.

## Boundary and remaining work

This closes analytic joint-position admission for fixed-horizon segments, including coupled square transmissions. It does not yet provide continuous collision sweeps, effort/impedance samples, observation history/scene ingest, root-pose arrays in this eval adapter, calibrated plant realization, or a dynamically re-solved contingency. A command state already outside the viable set may still require rejection; this report does not disguise that as safe braking.
