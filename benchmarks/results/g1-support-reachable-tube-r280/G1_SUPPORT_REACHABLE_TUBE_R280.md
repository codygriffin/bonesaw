# G1 reachable support tube timing qualification · R280

> Mechanism **PASS** · soft behavior **PASS** · soft timing **REJECTED** · hard authority **REJECTED**.

R280 qualifies the default-off R279 exact discrete DCM backward-reachable support tube on the frozen 2,317-tick G1 morphology replay. It executes zero policy and zero physics steps. Observer, request shaping, soft tracking, and hard acceleration rows remain independent authority layers.

## Behavior and switch independence

| profile | active | clipped | conflict | release | root RMS m | CoM RMS m | foot RMS m | p99 µs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| dormant | 0 | 0 | 875 | 1108 | 15.514 | 15.483 | 15.467 | 4768.9 |
| observer | 1832 | 0 | 875 | 1108 | exact | exact | exact | 4645.6 |
| request only | 1832 | 1205 | 875 | 1108 | exact | exact | exact | 4648.2 |
| soft two-axis | 1832 | 1205 | 875 | 1234 | 13.288 | 13.176 | 12.858 | repeated below |
| hard | 1832 | 0 | 325 | 326 | 16.227 | 16.119 | 15.652 | 4655.8 |

The two-axis soft profile delays release 1108→1234 and improves root/CoM/foot RMS by 14.35%/14.90%/16.87%. Observer and request-only integrated state/status are bit-exact to dormant; request-only changes bounded command telemetry on 1,205 ticks without gaining execution authority.

## Pinned CPU, jitter, memory, and GC

| repeat | p99 µs | max µs | >5 ms | wall ms/tick | CPU ms/tick | RSS Δ MiB | Python GC |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 5226.3 | 187989.3 | 42 | 1.765 | 1.764 | 3.547 | 0 |
| 1 | 5500.6 | 192120.5 | 55 | 1.821 | 1.821 | 3.547 | 0 |
| 2 | 5729.8 | 197385.1 | 57 | 1.826 | 1.826 | 3.621 | 0 |
| 3 | 5538.9 | 197116.9 | 57 | 1.816 | 1.815 | 3.547 | 0 |
| 4 | 5258.5 | 189680.6 | 41 | 1.768 | 1.767 | 3.613 | 0 |

All five CPU-4-pinned repeats are semantically bit-exact and all miss the 5,000 µs p99 gate. Mean p99 is 5450.8 µs (population σ 187.3 µs), with 252 deadline misses across 11,585 steps. Every repeat has deterministic late status-5 events at ticks 1694 and 2296 (188–197 ms). They do not set p99, but remain a separate controller-path tail defect. The Rust fold/WBC uses fixed-capacity storage; Python owns orchestration and retained trace arrays, so RSS delta is not a per-step Rust allocation claim.

## Hard evidence and rejected shortcut

Hard enforcement admits 662/1832 active ticks and leaves 1170 explicitly unresolved. Admitted rows have zero face leakage; the negative first-solve witness remains visible and is never integrated. Conflict/release 325/326 rejects hard authority.

A world-Y-only task lowered one p99 sample to 4956.5 µs but regressed release to 1140 and root/CoM/foot RMS to 17.370/17.306/17.099 m, all worse than dormant. It was removed. Timing is not purchased by deleting behavior-critical task rows.

## Decision

No actuator, contact, or walking authority is admitted. The immediate work is exact-trace CPU optimization of the 0.23–0.73 ms p99 excess and localization of the deterministic release path. Then hard support, contact, joint, actuator, thermal, and plant envelopes can be composed.
