# Bonesaw CPU counter stability audit · r58

## Result

Five release processes pinned to logical CPU 4 retire a median 239.807401 billion instructions. The instruction relative span is only 0.001081%, while task-clock spans 15.181–19.451 s (27.63% of the median) and cycles span 62.530–79.415 billion (26.73%). All 56 non-timing arrays remain byte-exact against r54.

> Retired instructions confirm deterministic work at whole-process scope. These counters include Python startup, reference loading, kinematic witnesses, repeatability passes, report generation, and NPZ serialization; they are not yet legal per-WBC-tick attribution.

## Five pinned trials

| trial | task clock s | cycles B | instructions B | IPC | branch miss % | cache misses / M insn | WBC p50 / p99 / max ms | >5 ms ticks / episodes / longest |
|---|---|---|---|---|---|---|---|---|
| run1 | 19.451 | 79.415 | 239.809363 | 3.020 | 0.192 | 217.130 | 5.021 / 8.551 / 11.784 | 1185 / 212 / 365 ms |
| run2 | 15.580 | 63.377 | 239.808826 | 3.784 | 0.190 | 167.245 | 3.436 / 5.803 / 7.853 | 65 / 45 / 25 ms |
| run3 | 15.453 | 63.164 | 239.806770 | 3.797 | 0.190 | 168.294 | 3.410 / 5.763 / 7.698 | 52 / 41 / 20 ms |
| run4 | 15.181 | 62.530 | 239.807401 | 3.835 | 0.190 | 156.325 | 3.374 / 5.648 / 7.583 | 51 / 39 / 20 ms |
| run5 | 15.220 | 62.850 | 239.807364 | 3.816 | 0.189 | 160.096 | 3.386 / 5.699 / 7.912 | 51 / 40 / 20 ms |

## Interpretation

- Preserve the deterministic bounded-work solver and separate 5 ms observation from the admitted 20 ms contract.
- Do not use wall-clock time to stop Jacobi or clipping work; identical instruction traces coexist with materially different task-clock and cycle totals.
- The first trial is a useful cold/contended outlier, not a discard: instruction work stays fixed while task-clock and cycles expand.
- Build a native WBC-only counter sentinel before attributing cycles or instructions to one solver tick or priority layer.
- Optimize only against byte-exact semantic traces and retain hard-row, tracking, clipping-duration, allocation, and 20 ms gates.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| benchmarks/results/g1-multistep-oracle-r54/oracle-wbc-admission-raw.npz | f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295 |
| benchmarks/results/g1-cpu-counters-r58/perf1.csv | 986d7ae24d748a1b23e1367dcd75d3464ab8322c9a7a63770bdfad01e7dd3c9a |
| benchmarks/results/g1-cpu-counters-r58/run1/oracle-wbc-admission-metrics.json | c35b25b65c714ac6f7c4e06fa37cd285ff0b4a55c572100a8c980e78740a8384 |
| benchmarks/results/g1-cpu-counters-r58/run1/oracle-wbc-admission-raw.npz | e5330a096dd375270dda52c20ebe824ec3f5aeca399b49a7d584601e47301f4b |
| benchmarks/results/g1-cpu-counters-r58/perf2.csv | 16214b06e823cf6431fa846d7123824f8717eecf6cc421cf73839c4e7def3e63 |
| benchmarks/results/g1-cpu-counters-r58/run2/oracle-wbc-admission-metrics.json | 9568506e45705fcb6e8798b74440eceb17e3506a92975ae50aca8549f69f3794 |
| benchmarks/results/g1-cpu-counters-r58/run2/oracle-wbc-admission-raw.npz | fc35c16c4ac41c76c0469a9b10412bb0818002032083b0d36516815799fa5931 |
| benchmarks/results/g1-cpu-counters-r58/perf3.csv | 0ebadcf860fe4c19bb43aa129deed2784b64b4899daed1daa807b8fc3f0891c5 |
| benchmarks/results/g1-cpu-counters-r58/run3/oracle-wbc-admission-metrics.json | 15ea23cf0bc6bfa428a8cdc81a1315c1d7f336ba95c614ece0d6dc210c4f2e45 |
| benchmarks/results/g1-cpu-counters-r58/run3/oracle-wbc-admission-raw.npz | 8ce2559fc38b3091bf95a286316bc498ca2b17f56a5135679c102b7ccceba2e8 |
| benchmarks/results/g1-cpu-counters-r58/perf4.csv | d79f0bb238ff049052bc6d4148c2d5c0531817a424147a56b653f2bcd3cd0144 |
| benchmarks/results/g1-cpu-counters-r58/run4/oracle-wbc-admission-metrics.json | 217df4a9f16f531783a350510ee83aa3f1415150fb749a458043c92f84a945c0 |
| benchmarks/results/g1-cpu-counters-r58/run4/oracle-wbc-admission-raw.npz | 06530bf953a8339ca2dd7aeda510b4a9e5dbf7deb38cb4d60932a7c213cc2ca5 |
| benchmarks/results/g1-cpu-counters-r58/perf5.csv | c5e4fa72fffc4b2f116ee7996a32bdccbfbe8e7412f39ab7f05a50076758d22d |
| benchmarks/results/g1-cpu-counters-r58/run5/oracle-wbc-admission-metrics.json | 1e5f086786f54409cbda9dfb9631ee151430a803c02e898fac2042e0e5a7a3d8 |
| benchmarks/results/g1-cpu-counters-r58/run5/oracle-wbc-admission-raw.npz | d9eaabb4fa11842c536900d0544bfd361c4e7bd45f46b47e1f0d07a31a24a673 |
