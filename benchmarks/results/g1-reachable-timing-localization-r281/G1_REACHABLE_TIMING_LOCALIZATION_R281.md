# G1 reachable-tube timing localization · R281

> Exact candidate **REJECTED AND REMOVED** · timing cause narrowed · authority **NO**.

R281 profiles the behavior-positive R280 two-axis support profile without policy or physics. It separates ordinary p99 work from two deterministic contact-release tails, then tests a scalar-order-identical two-column Jacobi specialization against the installed established build.

## Exact A/B

| build | p50 µs | p95 µs | p99 µs | max µs | >5 ms | Jacobi sweeps | pseudoinverses |
|---|---:|---:|---:|---:|---:|---:|---:|
| established | 1773.3 | 4098.9 | 5199.9 | 189059.7 | 40 | 56422 | 9368 |
| two-column specialization | 1803.1 | 4156.1 | 5364.0 | 192880.0 | 46 | 56422 | 9368 |

All 89 non-timing arrays are bit-for-bit identical. The specialization removes only vacuous pair-loop/feature branches and changes no mathematical work; it is slower on every displayed percentile and is removed. This rules out two-column dispatch overhead as the missing 0.2–0.7 ms.

## Work localization

| priority | Invariant | Safety | Viability | Preference | Style |
|---|---:|---:|---:|---:|---:|
| pseudoinverse calls | 1620 | 1192 | 2942 | 2310 | 1304 |
| Jacobi sweeps | 6500 | 5923 | 5999 | 19531 | 18469 |
| clipped steps | 524 | 947 | 2402 | 1934 | 839 |

Ordinary step latency correlates more with task Jacobi/pseudoinverse work (0.225/0.217) than bounded feasibility sweeps/halfspace projections (0.150/0.153). Preference and Style account for 67.3% of task Jacobi sweeps, while Viability produces the most clipped steps. The next exact optimization must remove repeated projected-task inversions or dense arithmetic after active-limit hits—not alter support behavior or factorization order.

## Deterministic release tails

Ticks 1694 and 2296 are status 5 after a pre-contingency status-2 failure. Their exported solver counters are all zero because the contact-free fallback overwrites diagnostics from the failed contact solve and retry. The 171–189 ms events are controller work, not a sleep or network pause. A cumulative per-attempt diagnostic is required before optimizing this tail; final-attempt counters alone are misleading.

## Decision

No controller or authority change is promoted. Preserve R280's full X/Y task. Next: add allocation-free cumulative solve-attempt/work telemetry, localize the repeated Preference/Style inverse rebuilds, and accept an optimization only if the 89-array trace remains exact and five pinned p99 repeats pass 5 ms.
