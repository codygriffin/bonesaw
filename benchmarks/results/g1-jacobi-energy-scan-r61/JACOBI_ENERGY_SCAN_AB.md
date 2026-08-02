# Bonesaw fused Jacobi energy-scan A/B · r61

## Decision · REJECT

Fusing the initial Frobenius reduction into the per-column energy scan preserves both results bit-for-bit, but adds 58,916 retired instructions per native tick (+0.268%). Production keeps the original two scans; the fusion remains disabled.

> Removing a source-level matrix pass is not automatically less machine work. The original independent Frobenius reduction is compiler-friendly; the fused loop carries two dependent accumulators and loses enough reduction efficiency to outweigh one fewer scalar square.

## Native five-run A/B

| variant | instructions/tick M | cycles/tick M | task-clock/tick ms | p50 / p99 / max µs |
|---|---|---|---|---|
| production control | 21.991006 | 5.855 | 1.422 | 1413.100 / 1539.089 / 2105.269 |
| fused experiment | 22.049923 | 5.861 | 1.419 | 1405.977 / 1724.429 / 1984.300 |

## Preservation and scope

- A direct Rust witness confirms bit-exact Frobenius squared norm and every initial column energy between separate and fused scans.
- Native control and experiment semantic reports are exactly equal, including status, residuals, margins, tracking, extrema, repeatability, and allocations.
- The unchanged production default passes r54 43/43 and keeps all 56 non-timing arrays byte-exact.
- The experimental fusion is available only through `bonesaw-core/jacobi-energy-scan-experiment` and is disabled by default.
- Stable instruction work decides this A/B; sequential-block latency and cycle medians remain observational host signals.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| benchmarks/results/g1-jacobi-energy-scan-r61/control/native-wbc-counter-metrics.json | 81d65ddf56ce3d30cff9a33849fabfc710877951e3ee5203dd06d8b5cd3977ce |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/native-wbc-counter-metrics.json | 2494ecef033896ee2d14b2c087cd343c7f077b8c077522820b51157072412b58 |
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz | f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295 |
| benchmarks/results/g1-jacobi-energy-scan-r61/r54-default/oracle-wbc-admission-raw.npz | e311ba3ad1a682fd67d666a097d0aa2876c294ff4d541cd8c38fb0f6e49a3152 |
