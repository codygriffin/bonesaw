# G1 feasibility-kernel A/B — r28

## Conclusion

The accepted r28 feasibility fast paths reduce deterministic CPU work while
preserving the complete non-timing trace byte-for-byte. They do **not** make the
160-tick G1 toe-step meet its 5 ms p99 deadline.

- Cached immutable squared row norms remove repeated norm dot-products from
  every cyclic halfspace projection.
- Exactly zero Dykstra multiplier deltas no longer execute a zero-valued row
  transpose update.
- Constraint order, multiplier arithmetic, stopping tests, tolerances, and the
  cold feasibility seed are unchanged.
- All 37 arrays shared by the cold and optimized raw traces, excluding only
  `step_ns`, are byte-identical (dtype, shape, NaN payloads, and signed-zero
  representation included).

## Environment and method

| Field | Value |
|---|---|
| CPU | AMD Ryzen 7 3700X, 8 cores / 16 threads |
| ISA | x86_64 |
| Affinity | logical CPU 2 via `taskset -c 2` |
| Build | Cargo release + PyO3 batch trace |
| Motion | 160 ticks, 5 ms nominal period, deterministic 1 cm G1 toe-step |
| Builds | cold kernel switches off; optimized switches on |
| Repeats | five sequential traces/build; five `perf stat` traces/build |
| Hardware counters | user task-clock, cycles, instructions, branches, branch misses, cache misses |

Builds were measured in separate sequential blocks rather than interleaved,
so wall-time comparisons remain exposed to frequency, thermal, scheduler, and
desktop-load drift. Retired instruction/branch counts are the primary evidence
for kernel work reduction; per-tick tail latency remains a reported outcome,
not the causal proof.

## Hardware-counter medians (five runs)

| Counter | Cold | Optimized | Delta |
|---|---:|---:|---:|
| task-clock ns | 1,101,364,090 | 976,462,008 | −11.341% |
| cycles | 4,365,906,793 | 3,810,830,640 | −12.714% |
| instructions | 15,690,449,483 | 14,634,689,670 | −6.729% |
| branches | 2,270,383,473 | 2,120,189,729 | −6.615% |
| branch misses | 8,574,674 | 6,737,000 | −21.431% |
| cache misses | 13,044,329 | 13,068,419 | +0.185% |

The instruction count is particularly stable: cold trials span
15,689,956,599–15,690,784,855 instructions; optimized trials span
14,634,583,069–14,634,824,048.

## Pinned per-tick timing (five runs)

| Build | p50 median (range), µs | p95 median (range), µs | p99 median (range), µs | max median (range), µs |
|---|---:|---:|---:|---:|
| Cold | 2,473.3 (2,437.9–2,499.6) | 25,472.7 (25,143.7–25,802.8) | 26,921.5 (26,429.2–28,520.5) | 28,663.9 (26,900.4–36,153.3) |
| Optimized | 2,485.6 (2,452.4–4,320.1) | 22,758.4 (19,651.5–33,215.6) | 29,908.5 (20,954.1–35,015.1) | 31,333.4 (21,523.8–36,383.3) |

The optimized five-run median whole-call thread CPU time is 861.237 ms versus
909.761 ms cold (−5.33%), but its p99 median is worse because two optimized
trials encountered broad host-side slowdowns. Five additional unpinned
optimized repeats span 20.991–30.503 ms p99 with a 24.162 ms median. The
canonical regenerated `latest` artifact records 21.661 ms p99. These spreads
are why no single best timing sample is used as the r28 acceptance claim.

## Solver work and validity

The optimized and cold traces both retain:

- 53 `Solved`, 64 `SolvedWithSlack`, 40 `Precontact`, and 3 planned
  `TouchdownNormal` ticks;
- zero fallback, release, infeasible, or numerical-failure ticks;
- one feasibility sweep on normal ticks;
- up to 755 sweeps / 176,670 halfspace projections in the locked-contact tail;
- dynamics/contact residuals below the unchanged 1e-8 hard contract;
- the same root, joint, tracked-frame, task, support-phase, status, and solver
  work bytes outside timing.

## Rejected algorithmic experiments

Moving the active-set polish handoff from normalized violation 1e-3 to 1e-2,
1e-1, and 1.0 reduced single-run p99 to 13.332, 10.152, and 7.119 ms,
respectively. The status/support sequences and hard residuals remained valid,
but joint trajectories changed materially (for example, up to 3.658 rad/s
velocity delta at 1e-2). Those variants are retained as experimental artifacts
and are not part of r28. A future early handoff needs an exact Euclidean
projection active set or another proof that removes seed dependence.

