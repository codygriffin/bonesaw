# G1 low-authority anytime budget · R285

> Mechanism **RETAINED DEFAULT-OFF** · hard feasibility **PASS** · degraded profiles **REJECTED** · authority **CLOSED**.

R285 evaluates the suggested ‘fewer QP iterations, continue operating’ boundary at the only soft layers allowed to degrade: Preference and terminal Style. Invariant, Viability, Intent, equality, bounds, and named hard rows are never capped. Exhaustion returns the current feasible solution, preserves every completed higher optimum, skips lower authority, and emits both a final-attempt level and a cumulative retry-safe mask.

## Policy-free / physics-free replay

| profile | exhausted ticks | p50 µs | p99 µs | >5 ms | root RMS m | CoM RMS m | stance-foot RMS m | normal contingencies |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| unbounded | 0 | 1779.9 | 5226.5 | 39 | 13.288 | 13.176 | 12.353 | 1353 |
| style1 | 13 | 18.3 | 3947.1 | 5 | 28.039 | 28.005 | 27.273 | 1829 |
| style2 | 2 | 16.3 | 4431.5 | 10 | 24.159 | 24.063 | 23.051 | 1669 |
| pref20+style2 | 2 | 16.4 | 3883.4 | 8 | 24.159 | 24.063 | 23.051 | 1669 |

Style-2 exhausts on only ticks `[155, 158]`. It lowers p99 by 15.21% and 5 ms misses by 74.36%, but the first omitted terminal refinement is also the first semantic divergence. Root RMS then increases by 81.81% and normal-contact contingencies rise to 1669. This is a closed-loop sensitivity result, not a hard-feasibility failure: dynamics/contact residuals stay below 1e-8 and failed/infeasible ticks remain zero.

## Decision

Retain the independently configurable and fully typed mechanism as a default-off degraded-mode primitive. Reject every measured finite profile for walking promotion. A static per-level call count is too blunt: two early Style truncations alter the later contact path. The next continuous design must carry explicit low-authority progress or use a supervisor with a tracking-error budget; it cannot infer safety from a local hard-feasible solve alone.

The unbounded default reproduces the established 89-array digest exactly and reports zero false exhaustion. No policy, physics, actuator command, contact authority, or hardware authority is introduced.
