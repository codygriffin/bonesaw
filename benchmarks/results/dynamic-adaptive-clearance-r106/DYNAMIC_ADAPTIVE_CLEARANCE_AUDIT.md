# Bonesaw adaptive pair/interval clearance · r106

## Outcome

**PASS.** R106 retains the exact 1 ms command grid, then recursively samples only pair/interval leaves whose local Lipschitz reserve is still below the required clearance. Maximum depth is an explicit construction-time bound. Joint-velocity extrema are analytic on each subinterval; midpoint FK and primitive distance use caller-owned Rust scratch.

The four cases below are the same observed state, WBC result, command polynomial, and 20 mm requirement. Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

| case | selection | sampled min mm | continuous lower mm | midpoint pair queries | leaf intervals | unresolved | depth reached | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|---|---|---|
| depth_0 | 1 | 20.231879529286623 | 19.008473505750956 | 0 | 0 | 0 | 0 | 20.619 / 34.092 / 53.741 | 0 / 0 |
| depth_1 | 1 | 20.231879529286623 | 19.62017651751879 | 1 | 21 | 1 | 1 | 32.185 / 40.011 / 51.467 | 0 / 0 |
| depth_2 | 1 | 20.231879529286623 | 19.926028023402704 | 2 | 22 | 1 | 2 | 33.257 / 40.012 / 41.599 | 0 / 0 |
| depth_3 | 0 | 20.231879529286623 | 20.078953776344665 | 3 | 23 | 0 | 3 | 24.476 / 30.923 / 35.267 | 0 / 0 |

The base 21-point trace remains bit-identical at **20.232 mm** with no sampled violation. Depths one and two improve the bound but keep one leaf explicitly unresolved. Depth three performs exactly three midpoint pair queries and proves **20.079 mm**, so Primary is admitted without weakening the 20 mm requirement.

All depths replay semantic bytes exactly across 100 command-state resets, allocate zero calls/bytes inside the timed Rust transaction, and satisfy hard dynamics/contact residuals below `1e-7`. A separate Rust regression uses safe base endpoints with a penetrating midpoint and proves refinement records a real sampled collision pair/time rather than allowing a continuous certificate to conceal it.

## Authority boundary

Refinement work, unresolved leaves, sampled violations, and final selection are independent witnesses. A deeper certificate may reduce false contingency selection, but it is not a policy, plant rollout, collision response, or guarantee about future command-state evolution. The R105 live path already demonstrates why that distinction matters.

## Remaining work

The bound still uses conservative primitive-point reach coefficients rather than closest-feature velocity, and box-box clearance remains a separating-axis lower bound. Pair-specific stopping criteria, shared primitive avoidance Jacobians, world collision, mesh CCD, and calibrated plant response remain outside this claim.
