# G1 NormalFallback bounded relock probe · R273

**Mechanism passed / walking profile rejected.** R273 removes the absorbing-state defect from `NormalFallback`: a default-off cadence may spend one bounded full-lock solve, promote only a solved result, and otherwise discard it before retrying the established normal-only rows.

## Result

| profile | interval | fallback | attempts | admit | reject | max relock ticks | release | root RMS m | p99 ms |
|---|---|---|---|---|---|---|---|---|---|
| r272 baseline | off | 875 | 0 | 0 | 0 | 0 | 1108 | 15.514 | 4.701 |
| r273 dormant | off | 875 | 0 | 0 | 0 | 0 | 1108 | 15.514 | 4.696 |
| r273 interval1 | 1 | 875 | 33 | 3 | 30 | 6 | 916 | 15.607 | 5.155 |
| r273 interval4 | 4 | 875 | 8 | 3 | 5 | 6 | 926 | 16.725 | 5.234 |
| r273 interval8 | 8 | 875 | 13 | 4 | 9 | 6 | 976 | 16.597 | 5.708 |
| r273 interval16 | 16 | 875 | 5 | 1 | 4 | 1 | 957 | 19.257 | 4.992 |
| r273 interval32 | 32 | 875 | 4 | 3 | 1 | 10 | 1011 | 16.085 | 5.941 |
| r273 interval64 | 64 | 875 | 3 | 0 | 3 | 0 | 1108 | 14.821 | 4.575 |

The disabled R273 replay is bitwise equal to R272 on all 72 non-timing arrays: True. Every enabled trace is also bitwise equal through the first fallback tick 875, so the probe cannot rewrite its cause. The right knee still reaches its lower limit at tick 874.

Intervals 1, 4, 8, 16, and 32 produced explicit full-lock admissions lasting at most 10 ticks, proving the state can recover continuously. They then re-entered fallback and released at ticks 916, 926, 976, 957, and 1011—all earlier than baseline tick 1108. Interval 64 rejected every probe and preserved the release tick, adding work without authority. No cadence is promoted.

## Execution contract

- Interval zero does not branch into probe construction or an extra solve.
- An admitted probe uses the ordinary full locked contact modes and is the only path from `NormalFallback` back to `Locked`.
- A rejected full-lock output is never integrated. The controller restores the normal-only modes and task before one bounded retry; rejection-with-release has its own typed status.
- Probe admission, rejection, rejection-with-release, relock duration, per-status solver work, and latency remain visible in the raw trace and corpus report.
- The replay consumes an immutable reference and initial-state witness; it executes zero policy and zero physics steps.
