# Bonesaw CPU-tail stability audit · r57

## Result

Four release traces execute the same 2,317 immutable G1 states. All non-timing arrays are byte-exact across 56 fields, including physical outputs, status, constraint residuals, clipping, and every solver-work counter. Despite identical work, nominal 5 ms overruns range from 49 to 558 ticks and the longest episode ranges from 20 to 320 ms.

> A deterministic iteration cap is not justified by this evidence. Dense-kernel work does correlate with latency inside each run, but host CPU placement/frequency/contention materially changes the tail while the work trace remains identical.

## Repeated timing

| run | affinity | p50 / p95 / p99 / max ms | >5 ms ticks / episodes | longest >5 ms | CPU / wall | latency corr pinv / Jacobi / clips |
|---|---|---|---|---|---|---|
| baseline | not recorded | 3.511 / 4.599 / 5.876 / 7.832 | 76 / 52 | 40 ms | 0.999799 | 0.486 / 0.676 / 0.513 |
| cpu2 | logical CPU 2 | 3.370 / 4.696 / 5.953 / 11.295 | 105 / 33 | 320 ms | 0.999717 | 0.427 / 0.607 / 0.457 |
| cpu4 | logical CPU 4 | 3.368 / 4.212 / 5.746 / 7.622 | 49 / 38 | 20 ms | 0.999694 | 0.495 / 0.689 / 0.524 |
| cpu6 | logical CPU 6 | 3.596 / 5.905 / 7.742 / 12.021 | 558 / 94 | 290 ms | 0.999835 | 0.335 / 0.485 / 0.351 |

## Cross-run stability

| run pair | per-tick latency correlation | >5 ms tick-set Jaccard |
|---|---|---|
| baseline ↔ cpu2 | 0.890 | 0.351 |
| baseline ↔ cpu4 | 0.957 | 0.623 |
| baseline ↔ cpu6 | 0.730 | 0.108 |
| cpu2 ↔ cpu4 | 0.919 | 0.439 |
| cpu2 ↔ cpu6 | 0.767 | 0.184 |
| cpu4 ↔ cpu6 | 0.750 | 0.088 |

The cross-run per-tick spread is 0.307 ms median, 2.747 ms p99, and 4.399 ms maximum. The best observed p99 is still 5.746 ms, so real kernel optimization remains useful; the unstable tail simply means wall-time iteration truncation is not yet the correct mechanism.

## Decision

- Preserve the exact bounded-work solver and 20 ms oracle admission contract.
- Do not add a clock-dependent early exit to the pure core.
- Add per-tick thread CPU cycles/instructions and fixed-governor isolated-host runs before changing Jacobi convergence or clipping budgets.
- Pursue semantics-preserving dense-kernel work first; accept an iteration budget only with a typed feasible-best status and separate tracking/dwell gates.
- Keep the nominal 5 ms line red in the browser until an isolated repeat distribution—not one lucky trace—passes.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-metrics.json | 0706030dc9d2f116d4b291efff7c3a041576fa4fba030f99150d7518c9229e18 |
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz | f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295 |
| benchmarks/results/g1-cpu-tail-r57/cpu2/oracle-wbc-admission-metrics.json | ae8d99241e4bd29e36baee32a0d55bd58f00535c9f7a4ff60b315e7cd206b5dd |
| benchmarks/results/g1-cpu-tail-r57/cpu2/oracle-wbc-admission-raw.npz | 24377645f955f75307c8563c4d01fac386a058faac291339af54c3a378e4a4f5 |
| benchmarks/results/g1-cpu-tail-r57/cpu4/oracle-wbc-admission-metrics.json | 89fe96b6cbb7219c0d1424385c64a41630635a43094e93526b8844e2597d71b5 |
| benchmarks/results/g1-cpu-tail-r57/cpu4/oracle-wbc-admission-raw.npz | dfe8cdd1d753fa828d39ab6bf3c419dbe256af23c9e34be165137845d4e3ab7f |
| benchmarks/results/g1-cpu-tail-r57/cpu6/oracle-wbc-admission-metrics.json | e6af5f7784a19cc0660b14af4b8682005c387a83c2884462e926c68af9ef962a |
| benchmarks/results/g1-cpu-tail-r57/cpu6/oracle-wbc-admission-raw.npz | 5e9a3ecf39603293eebca33255217c9c7952f46fee538f4aca1edfe9125008a0 |
