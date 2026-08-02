# Bonesaw floating feasibility projection budget · r262

> This report compares already-generated floating-WBC traces. It runs zero policy steps, physics steps, and plant actions; authority is not admitted.

## Result

The unbounded profile misses the 50 Hz (20 ms) deadline on 281/600 ticks, with p99 276773.8 µs and a 7296-sweep contingency. A finite 8-sweep ceiling bounds p99 to 4027.8 µs and max to 16698.0 µs, with zero 20 ms misses and no failed/infeasible or contact-release tick.

The cap is therefore a useful real-time fail-closed policy, not a functional walking admission: every bounded trace still exercises normal-contact contingency and retains red tracking/residual gates. The default remains unbounded until a multi-law floating transfer profile proves that bounded recovery preserves behavior.

## Sweep

| profile | projection cap | p50 µs | p99 µs | max µs | 20 ms misses | 5 ms p99 | contingency ticks | dynamics residual | foot RMS m | functional |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|
| unbounded | unbounded | 4883.8 | 276773.8 | 376164.7 | 281 | FAIL | 249 | 304 | 0.282 | FAIL |
| cap8 | 8 | 1665.9 | 4027.8 | 16698.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap16 | 16 | 1621.4 | 3924.6 | 16605.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap32 | 32 | 1652.0 | 4273.8 | 16441.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap64 | 64 | 2003.2 | 3961.3 | 16481.4 | 0 | PASS | 299 | 433 | 0.154 | FAIL |

## Interpretation

- A finite projection cap makes the 50 Hz timing contract explicit and keeps the existing contingency path state-retaining; it does not turn an unfinished hard-feasibility solve into an executable nominal command.
- The identical bounded behavior across 8–64 sweeps indicates that this stress trace reaches the same normal-contact fallback before the cap matters to tracking. The lower cap is recommended only for the measured timing envelope, not as a universal controller constant.
- The next functional gate is a fresh floating transfer holdout with contact-law/geometry improvements and a typed degraded-state metric; no policy or GPU batching is implied by this report.
