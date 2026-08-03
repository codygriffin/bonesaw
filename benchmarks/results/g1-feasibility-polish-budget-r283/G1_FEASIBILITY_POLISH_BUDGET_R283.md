# G1 bounded feasibility-polish profile · R283

> Exact profile **RETAINED** · deterministic tail **PASS** · p99 **REJECTED** · default/authority **UNCHANGED**.

R283 applies the user-suggested bounded-compute tradeoff to the policy- and physics-free R280/R281 WBC replay. It reduces the speculative active-set accelerator from 64 to seven iterations while retaining the existing eight-sweep exact Dykstra boundary. Failed acceleration remains fail-closed and is never integrated.

## Five CPU-4-pinned repeats

| repeat | 89 arrays | p50 µs | p95 µs | p99 µs | max µs | >5 ms | tails 1694/2296 ms | RSS Δ MiB | GC |
|---|---|---|---|---|---|---|---|---|---|
| 0 | exact | 1774.2 | 4018.0 | 5203.8 | 7926.2 | 40 | 7.37/7.93 | 3.957 | 0 |
| 1 | exact | 1778.6 | 4163.4 | 5252.3 | 7794.5 | 40 | 7.35/7.79 | 3.980 | 0 |
| 2 | exact | 1786.1 | 4284.7 | 5516.9 | 7759.4 | 48 | 7.24/7.76 | 3.977 | 0 |
| 3 | exact | 1776.6 | 4086.0 | 5244.1 | 7792.8 | 39 | 7.29/7.79 | 3.891 | 0 |
| 4 | exact | 1785.5 | 4203.2 | 5280.8 | 7938.1 | 40 | 7.35/7.79 | 3.969 | 0 |

All five runs reproduce the established 89-array digest `3489c58b1259bcb97feb87dde038de8df30da9c2d1de3cb77a4f169b9f228521`. The two deterministic tails fall from 195.2/184.3 ms to at most 7.93 ms (96.01% mean reduction). Dense polish pseudoinverses fall from 126 per tail to 12 (90.48% reduction).

## Decision

The profile is retained as an explicit real-time configuration, not promoted to the universal default: lower budgets changed behavior in the qualification sweep, so seven is corpus-qualified rather than a global theorem. Ordinary work still misses the 5,000 µs gate in every repeat (mean p99 5299.6 µs, σ 111.4 µs; 207 total misses). Preference/Style task inversions remain the next exact optimization target.

No support, contact, actuator, thermal, or walking authority is admitted by this result.
