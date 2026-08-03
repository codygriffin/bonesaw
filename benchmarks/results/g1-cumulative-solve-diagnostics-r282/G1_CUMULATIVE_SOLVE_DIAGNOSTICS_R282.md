# G1 cumulative solve-attempt diagnostics · R282

> Telemetry **PASS** · semantic preservation **PASS** · authority **NO**.

R282 adds a fixed-capacity Rust witness for every floating-WBC solve attempted during one controller tick. It executes zero policy and zero physics steps. The final-attempt arrays remain unchanged; cumulative arrays preserve failed primary, retry, localization, and handoff work before a safe fallback clears the reusable output.

## Attempt-stage work

| stage | attempts mean | attempts max | task pinv mean | task pinv max | task Jacobi mean | task Jacobi max | polish pinv mean | polish pinv max |
|---|---|---|---|---|---|---|---|---|
| primary | 0.550 | 1 | 4.036 | 26 | 24.309 | 178 | 0.214 | 63 |
| relock_retry | 0.000 | 0 | 0.000 | 0 | 0.000 | 0 | 0.000 | 0 |
| localization_probe | 0.000 | 0 | 0.000 | 0 | 0.000 | 0 | 0.000 | 0 |
| global_normal_retry | 0.002 | 1 | 0.003 | 6 | 0.018 | 42 | 0.054 | 63 |
| localized_handoff_retry | 0.000 | 1 | 0.005 | 11 | 0.024 | 55 | 0.000 | 1 |

The trace contains 5 ticks with retries and a maximum of 2 solve attempts per tick. Cumulative task work is 4.043 pseudoinverses and 24.351 Jacobi sweeps per tick (p99 16.0/99.7).

## Release-tail witness

| tick | latency ms | final status | pre status | attempts | stage mask | final task pinv | cum task pinv | final task Jacobi | cum task Jacobi | halfspaces | polish iter | polish pinv | polish Jacobi | primary polish pinv | retry polish pinv |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1234 | 1.454 | 5 | 2 | 2 | 0x09 | 0 | 0 | 0 | 0 | 3456 | 2 | 0 | 0 | 0 | 0 |
| 1694 | 195.173 | 5 | 2 | 2 | 0x09 | 0 | 0 | 0 | 0 | 3456 | 128 | 126 | 1152 | 63 | 63 |
| 2296 | 184.299 | 5 | 2 | 2 | 0x09 | 0 | 0 | 0 | 0 | 3456 | 128 | 126 | 1210 | 63 | 63 |

All three release ticks spend 16 projection sweeps and 3,456 halfspace projections. Tick 1234 finishes in roughly 1.4 ms with two polish iterations and no polish inverse. The deterministic 1694/2296 tails instead exhaust 128 polish iterations, including 126 dense polish pseudoinverses and 1,152/1,210 Jacobi sweeps. Stage attribution shows which solve owns those inversions. Stage masks are stable IDs: bit 0 primary, bit 1 relock retry, bit 2 localization probe, bit 3 global normal retry, and bit 4 localized handoff retry.

## CPU, memory, and decision

The replay p99 is 5283.0 µs with 0 Python GC collections and RSS delta 3.891 MiB. All 89 established non-timing arrays are bit-for-bit identical to the R281 profile (SHA-256 `3489c58b1259bcb97feb87dde038de8df30da9c2d1de3cb77a4f169b9f228521`). The new witness is caller-owned and allocation-free in the timed Rust path; it changes no controller authority or integrated state.

This closes the diagnostic blind spot. It does not yet optimize the repeated Preference/Style projected inversions or admit support/contact authority; the next slice must use these counters to prove an exact work reduction against five pinned repeats.
