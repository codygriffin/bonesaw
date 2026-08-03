# G1 support-transfer trajectory tube · R279

**Mechanisms pass; every walking profile remains rejected.** R279 keeps the finite-horizon position tube as one default-off hard option and adds a second, independently switchable exact discrete DCM backward-reachable-set observer. The observer folds axis-aligned support boxes backward through the authored preview, then computes a moving-boundary acceleration interval. A separate hard flag installs those four rows; intent projection remains a third, independent layer.

## Frozen simulator-free replay

| profile | active | hard | hard solved | unresolved | first conflict | release | root RMS m | foot RMS m | p99 µs | >5 ms |
|---|---|---|---|---|---|---|---|---|---|---|
| dormant control | 0 | no | 0 | 0 | 875 | 1108 | 15.514 | 15.467 | 4768.9 | 19 |
| local position h5 | 936 | yes | 234 | 702 | 874 | 874 | 17.427 | 17.127 | 4736.9 | 19 |
| reachable observer h5 | 936 | no | 0 | 936 | 875 | 1108 | 15.514 | 15.467 | 4750.8 | 17 |
| reachable hard h5 | 936 | yes | 0 | 936 | 295 | 295 | 26.629 | 26.555 | 4218.8 | 10 |
| reachable hard h10 | 956 | yes | 0 | 956 | 290 | 290 | 29.619 | 28.991 | 4972.7 | 22 |
| reachable hard h25 | 1016 | yes | 0 | 1016 | 275 | 275 | 26.562 | 25.729 | 4662.3 | 18 |
| reachable combined h5 | 936 | yes | 0 | 936 | 295 | 295 | 26.629 | 26.555 | 4642.9 | 18 |

The dormant build is bit-exact on all 89 shared non-timing arrays against R278. R279 adds no new trace arrays (`[]`); the six tube arrays introduced by R278 remain neutral when the feature is disabled. All runs execute zero policy steps and zero physics steps.

The local finite-horizon position tube is materially less aggressive than R278's DCM barrier: its h5 row first fails at 874 versus R278's 324, but it still does not improve the control prefix (1108 release) and remains rejected.

The reachable-set observer is behavior-neutral when hard enforcement and intent projection are both disabled. Its hard h5 counterpart retains a first-solve witness at tick 295 with minimum witness margin -2.836e+00 m/s², correctly failing closed before it can admit a contradictory acceleration interval. This is useful constraint evidence, not a walking claim.

The combined reachable h5 row projects intent on 861 ticks and releases at 295; intent shaping and hard viability therefore remain separate authority layers.

## Constraint evidence

The Rust reachability primitives use fixed-size boxes and a fixed four face output. Empty moving-boundary intervals return a typed `None` witness; they are never clamped into contradictory hard rows. Stable face IDs and first-solve witnesses survive contingency retries, while unresolved attempts remain visible in the trace.

## CPU, jitter, allocations, and memory

| profile | wall ms/tick | CPU ms/tick | CPU/wall | RSS Δ MiB | peak MiB | Python GC | trace peak B |
|---|---|---|---|---|---|---|---|
| dormant control | 1.380 | 1.379 | 0.9996 | 3.484 | 57.078 | 0 | 1848 |
| local position h5 | 1.353 | 1.352 | 0.9995 | 3.551 | 57.082 | 0 | 1848 |
| reachable observer h5 | 1.374 | 1.373 | 0.9999 | 3.551 | 57.180 | 0 | 1848 |
| reachable hard h5 | 0.525 | 0.524 | 0.9999 | 3.617 | 57.176 | 0 | 1848 |
| reachable hard h10 | 0.904 | 0.904 | 0.9999 | 3.551 | 57.207 | 0 | 1848 |
| reachable hard h25 | 0.796 | 0.796 | 0.9997 | 3.613 | 57.184 | 0 | 1848 |
| reachable combined h5 | 0.557 | 0.557 | 0.9998 | 3.617 | 57.219 | 0 | 1848 |

The per-tick Rust path uses fixed-capacity caller-owned storage and does not allocate. The reachability fold is bounded by the preview horizon; Python retains orchestration, arrays, statistics, and reporting. Timing is a consequence metric because rejected profiles enter different retry/release paths, not a kernel-speed claim.

## Decision and remaining chunk

Retain both formulations as default-off diagnostic mechanisms. Admit no walking profile and no actuator/contact authority. The next WBC chunk is a support-transition retiming/replanning loop that can react to a negative reachable-set witness (hold or move the liftoff edge) before hard rows become infeasible, then demonstrate support through tick 1108 with bounded tracking and p99 below 5 ms.
