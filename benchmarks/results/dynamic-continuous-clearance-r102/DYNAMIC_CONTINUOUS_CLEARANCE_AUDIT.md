# Bonesaw conservative continuous-clearance admission · r102

## Outcome

**PASS.** R102 keeps the deterministic 1 ms collision grid as direct evidence, then optionally requires a conservative continuous certificate. For each compiled sphere, Rust precomputes joint-coordinate center-speed coefficients. Analytic maximum joint velocity over the quintic bounds every pair's relative center speed, and admission subtracts half an interval of possible travel from the sampled minimum.

This is a conservative Lipschitz certificate over the represented sphere proxies. It is policy-free and physics-free, and it does not claim exact mesh continuous collision detection.

## Retained adversarial cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

| case | selection | flags | sampled primary min m | continuous lower m | relative speed m/s | braking lower m | raw violation pair / ns | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|---|---|---|
| safe_certified | 0 | 0x000 | 0.19999999999999996 | 0.19999999999999996 | 0.0 | 0.19999999999999996 | -1 / -1 | 19.627 / 25.536 / 50.055 | 0 / 0 |
| near_grid_only | 0 | 0x000 | 0.020231879529286623 | 0.019008473505750956 | 2.446812047071335 | 0.04444999999999996 | -1 / -1 | 23.610 / 32.403 / 35.558 | 0 / 0 |
| near_rate_certified | 1 | 0x200 | 0.020231879529286623 | 0.019008473505750956 | 2.446812047071335 | 0.04444999999999996 | -1 / -1 | 23.504 / 29.907 / 30.968 | 0 / 0 |

The near case is clear at every grid point: its sampled primary minimum is **20.232 mm** against a 20 mm requirement, and no raw violation pair or time exists. Its continuous lower bound is only **19.008 mm**, so flag `0x200` activates only when continuous certification is requested. The exact same command is selected under explicit grid-only policy and withheld under conservative-rate policy; the independently certified braking segment is selected with zero admitted feed-forward effort.

All cases replay semantic bytes exactly across 100 resets, allocate zero bytes inside the Rust transaction, satisfy hard dynamics/contact residuals below `1e-7`, and remain far inside the 20 ms horizon.

## Authority interpretation

The example authority stack now has two collision rows: **sampled geometry** records the measured proxy minimum and exact first failing sample, while **continuous clearance** records the lower-bound certificate and speed budget. A red continuous row with a green sampled row means “no collision was observed, but clearance was not proved”; it is not reported as an observed collision.

## Boundary and remaining work

The certificate is intentionally conservative and fixed-root over each 20 ms command segment. Unsupported authored geometry is still rejected by default; world SDF/obstacles, synthesized floating-root trajectories, tighter pair-specific interval bounds, and exact mesh CCD remain explicit future work.
