# G1 support-transfer trajectory tube · R278

**Mechanism passes; every walking profile remains rejected.** R278 adds a default-off, allocation-free hard horizontal CoM-acceleration polytope to the Rust floating WBC. Python supplies four causal DCM control-barrier faces from the intersection of current and previewed support patches. The boundary includes measured CoM velocity, measured CoM height, a finite schedule horizon, and optional joint-position headroom scaling. It is independent of the separately switchable soft root/CoM intent projector.

## Frozen simulator-free replay

| profile | active | hard solved | unresolved | first conflict | release | root RMS m | foot RMS m | p99 µs | >5 ms |
|---|---|---|---|---|---|---|---|---|---|
| dormant control | 0 | 0 | 0 | 875 | 1108 | 15.514 | 15.467 | 4711.7 | 14 |
| hard h5 | 936 | 233 | 703 | 324 | 324 | 15.958 | 15.285 | 5112.5 | 26 |
| hard h10 | 956 | 237 | 719 | 325 | 326 | 19.684 | 19.389 | 5020.6 | 25 |
| hard h25 | 1016 | 49 | 967 | 314 | 325 | 26.971 | 26.467 | 4662.6 | 15 |
| combined h5 | 936 | 14 | 922 | 309 | 310 | 27.147 | 26.431 | 5113.7 | 26 |

The dormant build is bit-exact on all 83 shared non-timing arrays against R277. Its six new caller-owned arrays are `['limiting_center_of_mass_tube_halfspace', 'minimum_center_of_mass_tube_margin', 'support_trajectory_tube_active', 'support_trajectory_tube_clipped', 'support_trajectory_tube_headroom_scale', 'support_trajectory_tube_target']` and remain inactive/neutral. All runs execute zero policy steps and zero physics steps.

The five-tick hard-only row is the least damaging candidate. It admits 233 active first solves with a minimum admitted face margin of -8.882e-16 m/s² and zero violations. Nevertheless its first hard conflict moves from control tick 875 to 324, release moves from 1108 to 324, root RMS changes 15.514→15.958 m, and p99 is 5112.5 µs. The row is fail-closed, but not useful.

The combined h5 row applies the proposed preview target as well as the hard boundary on 936 ticks. Its root/foot RMS is 27.147/26.431 m, confirming that intent projection and hard viability must remain separate authority layers.

## Constraint evidence

Each face has a stable ID and reports both the admitted post-contingency margin and the first-hard-solve witness before a retry can overwrite it. Failed or skipped solves remain `unresolved`; they are never counted as boundary violations or silently integrated. The selected hard-only row's limiting admitted face counts (+x/-x/+y/-y) are `{'0': 0, '1': 0, '2': 171, '3': 62}`. Its proposed but unapplied preview target differs from authored CoM intent by 4.799 cm RMS.

The selected row has 1 negative first-solve witness on an unsolved tick. That is the fail-closed evidence: the violating candidate is retained for diagnosis, classified unresolved, and never becomes an admitted acceleration or state update.

## CPU, jitter, allocations, and memory

| profile | wall ms/tick | CPU ms/tick | CPU/wall | RSS Δ MiB | peak MiB | Python GC | trace peak B |
|---|---|---|---|---|---|---|---|
| dormant control | 1.361 | 1.361 | 0.9997 | 3.496 | 56.871 | 0 | 1848 |
| hard h5 | 1.436 | 1.436 | 0.9999 | 3.570 | 57.031 | 0 | 1848 |
| hard h10 | 1.223 | 1.222 | 0.9998 | 3.484 | 57.020 | 0 | 1848 |
| hard h25 | 0.676 | 0.676 | 0.9998 | 3.484 | 56.977 | 0 | 1848 |
| combined h5 | 0.753 | 0.751 | 0.9982 | 3.551 | 56.980 | 0 | 1848 |

The per-tick Rust path uses fixed-capacity caller-owned halfspaces and does not allocate. Python retains orchestration, arrays, statistics, and reporting. Timing is a consequence metric because rejected profiles enter different retry/release paths; it is not normalized as a kernel speedup.

## Decision and remaining chunk

Retain the typed hard boundary, stable face telemetry, preview generator, and intent/viability split as default-off experimental mechanisms. Admit no walking profile and no actuator/contact authority. The largest remaining WBC chunk is a time-varying support-transfer viability construction that does not turn a locally valid DCM barrier into an early hard conflict—likely a short horizon of reachable sets or a small sequential convex tube with warm-started, explicitly budgeted progress across control ticks. It must retain support through control tick 1108, improve tracking, and stay below 5 ms p99 before the COMPLAINTS acceptance gate can close.
