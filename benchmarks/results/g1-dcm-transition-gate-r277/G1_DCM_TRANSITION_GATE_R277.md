# G1 schedule-bounded DCM transition shaping · R277

**Mechanism passed / DCM profile rejected.** R277 adds a default-off, allocation-free Rust schedule gate around the existing DCM/virtual-ZMP task. A positive horizon activates before an authored multi-to-single support loss, stays active through single support, and emits a caller-owned byte witness. Zero preserves the historical continuous mode.

| profile | active ticks | fallback | release | root RMS m | foot RMS m | p99 ms | max ms | >5 ms | >20 ms |
|---|---|---|---|---|---|---|---|---|---|
| R276 control | 0 | 875 | 1108 | 15.514 | 15.467 | 4.800 | 6.916 | 19 | 0 |
| R277 dormant | 0 | 875 | 1108 | 15.514 | 15.467 | 4.708 | 7.076 | 16 | 0 |
| R277 gated 5e-5 / 10 | 956 | 926 | 959 | 19.280 | 19.059 | 6.726 | 181.498 | 67 | 2 |
| continuous 0.025 | 2317 | 467 | 466 | 24.882 | 24.569 | 3.904 | 131.586 | 11 | 1 |

## Causal state at the original conflict

| profile | root y m | CoM y m | root vy m/s | right knee rad |
|---|---|---|---|---|
| R276 control | 0.15284 | 0.14338 | -1.66794 | -0.08727 |
| R277 dormant | 0.15284 | 0.14338 | -1.66794 | -0.08727 |
| R277 gated 5e-5 / 10 | 0.10510 | 0.10696 | -0.42716 | 0.43082 |
| continuous 0.025 | -5.03971 | -4.95757 | 0.06741 | -0.08098 |

The gated profile does move the upstream state in the intended direction: at tick 874, lateral root speed falls from -1.668 to -0.427 m/s, CoM y moves from 0.14338 to 0.10696 m, and the right knee is no longer pinned at its -0.087267 rad lower limit. The original tick-875 full-lock conflict moves to tick 926. This is a causal mechanism result, not a walking pass.

The consequence remains unacceptable. Release advances from tick 1108 to 959, root/foot RMS rise from 15.514/15.467 m to 19.280/19.059 m, and p99 reaches 6.726 ms with two 20 ms misses. During the gated interval, virtual ZMP is clipped on 75.8% of active ticks, support margin p05/min is -0.993/-1.213 m, and DCM acceleration p95/max is 7.738/10.920 m/s². The soft task can change the state, but it does not construct a support-feasible trajectory.

## Frozen small-weight screen

| weight | pre ticks | fallback | release | root RMS m | foot RMS m |
|---|---|---|---|---|---|
| 0.000010 | 5 | 450 | 449 | 24.489 | 24.269 |
| 0.000025 | 5 | 432 | 431 | 24.828 | 24.172 |
| 0.000050 | 5 | 725 | 734 | 19.154 | 18.727 |
| 0.000100 | 5 | 793 | 860 | 18.341 | 17.722 |
| 0.000010 | 10 | 463 | 462 | 19.997 | 19.897 |
| 0.000025 | 10 | 432 | 431 | 25.293 | 24.763 |
| 0.000050 | 10 | 926 | 959 | 19.280 | 19.059 |
| 0.000100 | 10 | 463 | 462 | 22.348 | 22.367 |

All eight schedule-bounded profiles fail before the control release; the 5e-5 / 10-tick row is the latest at tick 959 and was rerun alone for the retained timing/memory record. The historical continuous 0.025 mode is a negative control: it releases at tick 466 and reaches 24.882/24.569 m root/foot RMS.

## Dataflow, CPU, jitter, and memory

| profile | wall ms/tick | CPU ms/tick | CPU/wall | RSS delta MiB | peak RSS MiB | Python GC | tracemalloc peak B |
|---|---|---|---|---|---|---|---|
| R276 control | 1.383 | 1.382 | 0.9995 | 3.305 | 56.176 | 0 | 1664 |
| R277 dormant | 1.364 | 1.364 | 0.9997 | 3.316 | 56.188 | 0 | 1672 |
| R277 gated 5e-5 / 10 | 1.695 | 1.694 | 0.9998 | 3.309 | 56.242 | 0 | 1672 |
| continuous 0.025 | 0.810 | 0.810 | 0.9999 | 3.301 | 56.281 | 0 | 1672 |

Dormant R277 is bit-exact on all 82 shared non-timing arrays against R276; the only new array is `dcm_pre_liftoff_active`, and it is zero throughout. Every retained run performs zero policy steps and zero physics steps. The Python hot call records zero garbage collections and a 1,672-byte tracemalloc peak; caller-owned arrays carry all per-tick telemetry. Timing is workload-dependent because rejected profiles enter different contact/release states, so it is reported as consequence rather than normalized solver speedup.

## Decision

Retain the schedule gate as a default-off experimental/diagnostic mechanism, but admit no profile and no authority. R277 rules out both continuous and schedule-bounded scalar weighting of the current DCM objective. The next CPU implementation needs an explicit support-feasible root/CoM trajectory tube (with velocity and joint-headroom state), not another gain sweep or failure-time selector.
