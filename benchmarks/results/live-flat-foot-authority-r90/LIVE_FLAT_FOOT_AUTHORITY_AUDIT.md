# Bonesaw live finite-foot authority · r90

## Outcome

**Admission: PASS.** A separate flat-foot browser adapter uses the toy humanoid's authored mass, inertia, limits, and collision geometry. Rust construction emits eight force points, two exact 20 mm CoP patches, and two R87-derived six-row rigid-foot bases. Every state-local raw-WBC query composes the R83 position/velocity/braking envelope before solving. Python drives the same WebSocket commands as the browser and scores state frames; guided pose updates deliberately avoid a policy, external physics, or synthetic body-response claim.

## Live authority transition

| signal | result |
|---|---|
| profile | bonesaw-tools/flat-foot-editor-r90 |
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

## End-to-end query and batch timing

| scope | count | p50 µs | p95 µs | p99 µs | max µs | budget overruns |
|---|---|---|---|---|---|---|
| maximum single query per streamed frame | 60 | 1923.276 | 2818.571 | 3076.067 | 3223.385 | 0 / 5 ms |
| sum of four 5 ms subqueries | 60 | 7348.656 | 8793.136 | 9147.807 | 9372.934 | 0 / 20 ms |

`solve_us` now has one unambiguous contract: it is the maximum end-to-end single-query duration among the four 5 ms subqueries represented by a 50 Hz streamed frame. `query_batch_us` is their sum and is compared with 20 ms. Transport, JSON serialization, and browser rendering are outside both measurements. Physical admission does not erase either compute-budget witness.

## Solver-work attribution

| work counter | p50 | p99 | max | correlation with solve time |
|---|---|---|---|---|
| task_pseudoinverse_calls | 3.0 | 10.4 | 11.0 | 0.145 |
| task_jacobi_sweeps | 11.0 | 75.2 | 90.0 | 0.174 |
| clipped_steps | 0.0 | 7.8 | 9.0 | 0.129 |
| feasibility_projection_sweeps | 1.0 | 1.0 | 1.0 | constant |
| feasibility_polish_iterations | 0.0 | 0.0 | 0.0 | constant |

These counters distinguish task-level dense pseudoinverse/Jacobi work from hard-feasibility projection and active-set polishing. They are diagnostic witnesses, not iteration budgets to tune until timing turns green.

## Deliberate boundary

The toy geometry is a CPU concept fixture, not hardware calibration. The guided editor queries instantaneous feasibility at each commanded pose and does not integrate Bonesaw acceleration as a physics surrogate. An unguided 200 Hz self-integration experiment drifted into a hard-row numerical failure under sustained lean; it is not promoted and remains a required closed-loop/physics eval. General contact switching, calibrated actuator realization, thermal state, a production non-wheeled model, and CUDA remain unavailable.
