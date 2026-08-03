# G1 post-transfer Style budget · R290

> Mechanism **RETAINED DEFAULT-OFF** · nominal prefix **EXACT** · hard residuals **PASS** · walking promotion **REJECTED**.

R290 arms the existing terminal Style projected-solve ceiling only after the causal support-transfer projector clips an intent command. The clipping tick remains nominal; the bounded solve starts on the next tick. Invariant, viability, intent, equality, bounds, and contact rows are unchanged.

## Frozen policy/physics-free replay

| profile | p50 µs | p99 µs | max µs | >5 ms | root RMS m | CoM RMS m | stance-foot RMS m |
|---|---:|---:|---:|---:|---:|---:|---:|
| unbounded | 1774.1 | 5191.3 | 7667.2 | 38 | 13.288 | 13.176 | 12.353 |
| post-transfer Style-2 | 1761.7 | 5165.3 | 7875.3 | 35 | 14.623 | 14.527 | 13.648 |

The first support-tube clip is tick `819`. The inclusive prefix through that tick is bit-exact. Style-2 exhausts once at tick `1152`, which is the first capped solve and first semantic divergence; hard dynamics/contact residuals remain `2.603e-09` / `5.972e-11`.

## Pinned CPU-4 repeats, jitter, and process memory

| profile | repeat | p99 µs | max µs | >5 ms | wall ms | max RSS MiB | semantics |
|---|---:|---:|---:|---:|---:|---:|---|
| unbounded | 0 | 5191.3 | 7667.2 | 38 | 3970.0 | 59.5 | exact |
| unbounded | 1 | 5288.1 | 7780.8 | 40 | 3980.0 | 59.5 | exact |
| unbounded | 2 | 5259.9 | 7773.1 | 40 | 4020.0 | 59.4 | exact |
| unbounded | 3 | 5229.2 | 7706.1 | 37 | 3990.0 | 59.4 | exact |
| unbounded | 4 | 5204.2 | 7704.3 | 39 | 3980.0 | 59.4 | exact |
| post_transfer_style2 | 0 | 5165.3 | 7875.3 | 35 | 3870.0 | 59.4 | exact |
| post_transfer_style2 | 1 | 5183.6 | 7833.4 | 33 | 3880.0 | 59.6 | exact |
| post_transfer_style2 | 2 | 5174.3 | 7821.4 | 32 | 3910.0 | 59.5 | exact |
| post_transfer_style2 | 3 | 5182.7 | 7888.6 | 32 | 3880.0 | 59.4 | exact |
| post_transfer_style2 | 4 | 5177.5 | 8052.0 | 35 | 3910.0 | 59.5 | exact |

Across five repeats the candidate mean p99 is 5176.7 µs (σ 6.6 µs), still above the 5,000 µs target. The candidate raises root/CoM/foot RMS by 10.0%/10.3%/10.5%; therefore a lower-authority solve is not promoted merely because it removes some dense work. RSS is process-level evidence from `/usr/bin/time`, not a per-step Rust allocation claim.

## Decision

Retain the gate as a typed, default-off research primitive. Reject Style-2 for walking: it misses the timing gate and regresses closed-loop tracking after the first exhaustion. The nominal unbounded profile remains the reference. A future continuous authority design needs an explicit tracking-error or progress budget rather than a static Style call count.
