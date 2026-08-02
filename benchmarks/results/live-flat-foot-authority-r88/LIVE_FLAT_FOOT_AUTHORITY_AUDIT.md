# Bonesaw live finite-foot authority · r88

## Outcome

**Admission: PASS.** A separate flat-foot browser adapter uses the toy humanoid's authored mass, inertia, limits, and collision geometry. Rust construction emits eight force points, two exact 20 mm CoP patches, and two R87-derived six-row rigid-foot bases. Every state-local raw-WBC query composes the R83 position/velocity/braking envelope before solving. Python drives the same WebSocket commands as the browser and scores state frames; guided pose updates deliberately avoid a policy, external physics, or synthetic body-response claim.

## Live authority transition

| signal | result |
|---|---|
| profile | bonesaw-tools/flat-foot-editor-r88 |
| support patches / force points | [8401, 8402] / [4, 4] |
| commanded chest lean | 0.160 m |
| support margin: before / minimum / final | 46.840 / 20.000 / 53.949 mm |
| ticks below 30 mm optional reserve | 13 |
| limiting patch counts | {'8401': 14, '8402': 46} |
| minimum solved joint-stopping reserve | 184.709078 rad/s² |
| maximum effort utilization | 21.036% |
| maximum hard residual | 1.308e-09 |
| active task count range | [4, 4] |
| status counts | {'Degraded': 17, 'Ok': 43} |

Support and joint stopping remain separate Viability rows. Effort, hard residual, task residuals, solver work, and model availability remain independent; there is no aggregate health verdict.

## End-to-end adapter timing

| samples | p50 µs | p95 µs | p99 µs | max µs | >5 ms ticks |
|---|---|---|---|---|---|
| 60 | 7370.475 | 9116.861 | 10646.911 | 11787.180 | 59 |

Timing is retained even when it misses the 5 ms editor contract; physical admission does not erase compute-budget pressure.

## Deliberate boundary

The toy geometry is a CPU concept fixture, not hardware calibration. The guided editor queries instantaneous feasibility at each commanded pose and does not integrate Bonesaw acceleration as a physics surrogate. An unguided 200 Hz self-integration experiment drifted into a hard-row numerical failure under sustained lean; it is not promoted and remains a required closed-loop/physics eval. General contact switching, calibrated actuator realization, thermal state, a production non-wheeled model, and CUDA remain unavailable.
