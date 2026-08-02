# Bonesaw floating feasibility projection budget · r262

> This report compares already-generated floating-WBC traces. It runs zero policy steps, physics steps, and plant actions; authority is not admitted.

## Result

The unbounded profile misses the 50 Hz (20 ms) deadline on 281/600 ticks, with p99 276773.8 µs and a 7296-sweep contingency. A finite 8-sweep ceiling bounds generic-build p99 to 4027.8 µs, but its 16698.0 µs maximum still misses 5 ms.

The complete measured execution profile additionally limits feasibility polish to two iterations and uses the spec's CPU-ISA build (`-C target-cpu=native`) pinned to logical CPU 4. Across five independent 600-tick processes it records 0/3000 five-millisecond misses, 0/3000 twenty-millisecond misses, 3468.6 µs worst per-process p99, and 3550.6 µs observed maximum. All 72/72 non-timing arrays replay exactly and Python performs zero collections.

This closes the measured ordinary-process 200 Hz execution overrun for the fail-closed profile, not the controller behavior gate: the two-iteration trace spends 354/600 ticks in normal-contact contingency and retains red tracking/residual gates. The default remains unbounded until a multi-law floating transfer profile proves that bounded recovery preserves behavior.

## Sweep

| profile | projection cap | p50 µs | p99 µs | max µs | 20 ms misses | 5 ms p99 | contingency ticks | dynamics residual | foot RMS m | functional |
|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---|
| unbounded | unbounded | 4883.8 | 276773.8 | 376164.7 | 281 | FAIL | 249 | 304 | 0.282 | FAIL |
| cap8 | 8 | 1665.9 | 4027.8 | 16698.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap16 | 16 | 1621.4 | 3924.6 | 16605.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap32 | 32 | 1652.0 | 4273.8 | 16441.0 | 0 | PASS | 299 | 433 | 0.154 | FAIL |
| cap64 | 64 | 2003.2 | 3961.3 | 16481.4 | 0 | PASS | 299 | 433 | 0.154 | FAIL |

## Host-native 200 Hz repeats

| trial | p50 µs | p99 µs | max µs | 5 ms misses | 20 ms misses | contingency ticks | GC collections |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 1404.5 | 2730.7 | 3030.5 | 0 | 0 | 354 | 0 |
| 1 | 1397.2 | 2764.4 | 3001.7 | 0 | 0 | 354 | 0 |
| 2 | 1527.1 | 3444.4 | 3516.8 | 0 | 0 | 354 | 0 |
| 3 | 1402.1 | 3468.6 | 3550.6 | 0 | 0 | 354 | 0 |
| 4 | 1397.8 | 2753.9 | 3003.9 | 0 | 0 | 354 | 0 |

## Process resources

| trial | wall ms | process CPU ms | thread CPU ms | CPU/wall | jitter p99 µs | RSS Δ MiB | peak RSS MiB | involuntary switches |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 1075.4 | 1074.4 | 1074.3 | 0.999 | 699.8 | 1.008 | 55.6 | 38 |
| 1 | 1073.9 | 1073.3 | 1073.3 | 0.999 | 692.5 | 1.008 | 55.6 | 18 |
| 2 | 1138.3 | 1138.0 | 1137.9 | 1.000 | 784.1 | 1.008 | 55.5 | 13 |
| 3 | 1111.3 | 1111.1 | 1111.0 | 1.000 | 860.2 | 1.008 | 55.6 | 8 |
| 4 | 1076.3 | 1075.1 | 1075.0 | 0.999 | 710.7 | 1.008 | 55.7 | 49 |

Python `tracemalloc` peaks at 1560 bytes and observes 0 collections. It does not observe Rust allocations; the native controller allocation sentinel remains authoritative.

## Functional cost of the timing profile

The exact replay has 199 solved, 47 solved-with-slack, and 354 normal-contact-contingency ticks per process. Root/foot RMS are 0.148/0.168 m; maximum dynamics/contact-acceleration residuals are 317/9.55. These values are failures, not hidden by the timing gate.

## Interpretation

- Finite projection and polish budgets make the timing contract explicit and keep the existing contingency path state-retaining; they do not turn an unfinished hard-feasibility solve into an executable nominal command.
- The generic 8–64 sweep rows close the apparent freeze and 50 Hz deadline, but only the separately fingerprinted host-native 8-sweep/two-polish profile demonstrates zero five-millisecond overruns. The profile is not a universal controller constant.
- Exact repeat covers 72 non-timing arrays. Python GC is zero; Rust hot-loop allocation remains governed by the existing native controller allocation sentinel because `tracemalloc` cannot observe the Rust allocator.
- The next functional gate is a fresh floating transfer holdout with contact-law/geometry improvements and a typed degraded-state metric; no policy or GPU batching is implied by this report.
