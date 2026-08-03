# G1 host-native CPU timing qualification · R297

> Host-native production p99 **PASS** · fixed-58 candidate **REJECTED** · 112-array exactness **PASS** · authority **CLOSED**.

R297 separates two questions that earlier timing reports mixed: whether the local deployment target meets the 5 ms WBC deadline when compiled for its actual CPU, and whether R296's 58-element Jacobi coupling-dot specialization should change the source default. The answer is yes to the qualified host-native build profile and no to the specialization.

## Policy/physics-free exact replay

Seven CPU-4-pinned AB/BA pairs execute the frozen 2,317-tick support-transfer corpus with `RUSTFLAGS=-C target-cpu=native`. Every pair is exact across all 112 non-timing arrays with digest `70d80266121b1d50304c2b18535e9a928e27ac1c6d4b446238210675dc29764b`. No policy or physics step executes.

| pair | production p50 / p99 µs | fixed-58 p50 / p99 µs | production / candidate max µs |
|---|---:|---:|---:|
| 1 | 1562.2 / 4644.5 | 1571.6 / 4683.5 | 6774.8 / 6822.2 |
| 2 | 1575.8 / 4620.9 | 1565.4 / 4617.1 | 6728.3 / 6730.6 |
| 3 | 1571.1 / 4633.4 | 1567.0 / 4670.6 | 6783.8 / 6735.4 |
| 4 | 1586.8 / 4660.0 | 1569.3 / 4647.2 | 6756.4 / 6773.0 |
| 5 | 1573.4 / 4690.9 | 1571.5 / 4645.1 | 6730.9 / 6716.4 |
| 6 | 1570.7 / 4711.9 | 1579.8 / 5060.5 | 6809.2 / 7261.1 |
| 7 | 1574.6 / 4628.7 | 1553.9 / 4603.0 | 6633.3 / 6644.1 |

Production passes all seven 5,000 µs p99 checks; its median p99 is 4644.5 µs and worst repeat is 4711.9 µs. The candidate passes six of seven, but reaches 5060.5 µs once and has a slightly worse 4647.2 µs median. One retained process-memory pair records 62,732 KiB production and 62,656 KiB candidate RSS; this is complete-process evidence, not a per-solve allocation claim.

Two earlier generic-extension pairs ran beside extension builds and reached roughly 17 ms p99 in both profiles. They remain explicit non-decision evidence. R293's separately qualified portable build remains the portable reference at 5.175 ms median p99; R297 closes only this machine's declared host-native build profile.

## Why the local microbenchmark did not generalize

| scope | production instructions | fixed-58 instructions | delta | branch delta |
|---|---:|---:|---:|---:|
| narrow 2,000-tick native sentinel | 29.429 B | 28.056 B | -4.664% | -15.721% |
| full 2,317-tick Python admission process | 39.374 B | 39.475 B | +0.255% | +0.579% |

The fixed-size path fully unrolls 58 multiply/add terms and grows the pseudoinverse function by 1,127 bytes. That helps the narrow fixed-shape native sentinel, but the integrated support-transfer process uses a broader mix of projected shapes. Every one of three complete-process counter pairs retires more instructions (+0.2533%, +0.2565%, +0.2554%), while cache misses rise 0.97% on average and the wall tail fails once. Mean cycles appear lower, but one control process crosses a host-load rise; cycles are therefore not decision evidence.

## Decision

Keep the R296 specialization rejected and absent from production source; R293 remains the source default. Adopt `-C target-cpu=native` as the default for the explicitly local managed server, with `BONESAW_LIVE_RUSTFLAGS` as the override. This closes the local host-native 5 ms corpus gate without claiming a portable generic deadline, walking/contact realization, calibrated actuator/thermal authority, or hardware authority.
