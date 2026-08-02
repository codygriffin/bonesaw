# Bonesaw integrated floating CPU advance · r99

## Outcome

**PASS.** R99 composes the admitted strict floating WBC query with explicit actuator mapping, exact commanded-state splice, primary and braking-contingency quintics, analytic segment validation, and one contiguous 20×1 ms actuator block. Rust owns the complete transaction; Python owns corpus construction, arrays, statistics, and this report.

This is intentionally policy- and physics-free. Every tick consumes an authored observed state. The command consequence is measured, but it is never fed back as if it were a simulated robot.

## Transaction and authority boundary

| stage | retained behavior |
|---|---|
| Observed authority query | Upkie floating WBC with two RollingPoint wheel contacts |
| Admission | only Solved/SolvedWithSlack and analytically valid segment may select primary |
| Actuation | generalized joint acceleration and effort mapped through immutable CompiledActuation |
| Splice | new segment begins at exact previous commanded position/velocity/acceleration |
| Contingency | independently synthesized braking segment; feed-forward effort becomes exactly zero |
| Dense output | 20 samples at deterministic 1 ms spacing per 20 ms transaction |
| Plant boundary | no state integration, contact response, estimator, or policy |

The example authority stack therefore gains a distinct **command synthesis and admission** row. WBC feasibility, actuator-space effort, segment validity, compute budget, and observed/commanded divergence remain separate signals.

## Corpus and fixed layout

| signal | value |
|---|---|
| reference model | models/upkie/upkie.urdf |
| ticks / authored duration | 500 / 10.000 s |
| joint / generalized / actuator coordinates | 6 / 12 / 6 |
| samples per tick | 20 |
| Python-owned corpus/output bytes | 1781000 |
| primary / contingency / rejected | 500 / 0 / 0 |
| D1 complete semantic output bytes exact | True |

The reference is a five-cycle, 2 s cosine-derived acceleration command over the first two joint coordinates. Both observations and root twist remain zero by construction, so the run asks only: what command block does this state-local WBC transaction emit?

## Exact splice and segment validity

| gate | result |
|---|---|
| position splice maximum absolute error | 0.0 |
| velocity splice maximum absolute error | 0.0 |
| acceleration splice maximum absolute error | 0.0 |
| all primary segments analytically valid | True |
| all contingency segments analytically valid | True |
| expired prior plans | 0 |
| maximum analytic |velocity| | 0.09084178790106837 |
| maximum analytic |acceleration| | 0.27120690948977744 |
| maximum analytic |jerk| | 88.82718212204296 |
| minimum analytic joint-position headroom | 1.1023498690982552 |
| ticks with nonzero admission flags | 0 |

The splice comparisons are against the prior block's exact twentieth sample and are byte-equivalent as f64 values. Analytic extrema come from polynomial roots, not a sampling-only approximation.

## WBC tracking and physical invariants

| signal | result |
|---|---|
| joint-acceleration objective RMS | 0.0018204825878276848 |
| joint-acceleration objective maximum absolute error | 0.0031926019705009745 |
| maximum floating-dynamics residual | 5.145550652230213e-12 |
| maximum contact-acceleration residual | 2.048692321560373e-14 |
| maximum hard residual | 5.145550652230213e-12 |
| maximum admitted actuator effort magnitude | 0.1468719881622101 |

Tracking residual is not hidden when dynamics/contact consume authority. It is an objective consequence, while the two hard residuals remain the admission evidence.

## Command consequence without a plant

| signal | result |
|---|---|
| maximum command position magnitude | 0.15765013090174493 |
| maximum command velocity magnitude | 0.09084140699810127 |
| maximum sampled acceleration magnitude | 0.2711696476447925 |
| maximum observed/commanded position divergence | 0.1570201847543566 |
| final observed/commanded position divergence | 0.1570201847543566 |

The divergence is supposed to be visible: observations remain fixed while commands evolve. No tracking-success, stability, disturbance-recovery, power, or reliability claim is inferred from this trace.

## Time, jitter, CPU, memory, and allocation

| metric | result |
|---|---|
| mean / p50 / p95 / p99 / max step µs | 165.900 / 151.993 / 206.708 / 215.846 / 258.999 |
| p99 − p50 jitter µs | 63.85374999999999 |
| 20 ms deadline overruns | 0 |
| maximum over-execution relative to 20 ms µs | -19741.001 |
| maximum Rust allocation calls / bytes per step | 0 / 0 |
| calling-thread CPU / wall ms | 84.018 / 84.112 |
| calling-thread CPU / wall ratio | 0.9988832973705344 |
| maximum RSS before / after / growth KiB | 35640 / 37176 / 1536 |

These are untrimmed same-process release-build measurements of the Rust transaction inside the GIL-detached boundary, not target-hardware WCET. PyO3 detach/reattach bookkeeping is intentionally outside the core allocation counter and is represented in the enclosing calling-thread CPU/wall measurement. Array bytes are explicit Python corpus/output storage; RSS is a process high-water mark and therefore not attributed solely to Rust.

## Remaining admission work

The transaction still needs observed root pose/twist arrays beyond this zero-root eval adapter, effort or impedance feed-forward sampling as an explicit command schema, collision validation on the dynamic segment, state-history ingest, frame snapshots, typed program/input epochs in every exported block, and long recorded-replay/fault corpora. Closed-loop tracking still requires a separately named external plant or hardware run.
