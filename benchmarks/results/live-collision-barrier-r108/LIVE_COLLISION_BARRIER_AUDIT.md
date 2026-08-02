# Bonesaw live floating collision viability · r108

## Outcome

**PASS.** The r108 WebSocket contract exposes the floating closest-feature collision barrier as its own Viability signal across 60 guided 50 Hz frames. It does not replace the sampled/adaptive command-clearance rows and does not turn the guided pose into plant response.

| signal | retained value |
|---|---|
| closest clearance / hard-margin reserve | 50.000 / 30.000 mm |
| active pair count | 13–13 |
| limiting pairs | {"12": 60} |
| limiting bodies | {"left_thigh\u2194right_thigh": 60} |
| quality | {"analytic_primitive": 60} |
| minimum barrier residual | 4.737e+00 m/s² |
| raw WBC p50 / p99 / max | 2.011 / 3.266 / 3.359 ms |

All retained frames report zero unsupported shapes. The minimum achieved-minus-required normal acceleration is `4.737e+00` m/s². Maximum dynamics/contact residuals are `1.670e-09` / `4.899e-11`, and the separate command certificate retains at least `19.601` mm.

## Typed authority boundary

The live row carries closest pair/body, limiting active pair/body, active count, explicit primitive quality, signed clearance and hard-margin reserve, relative normal velocity, required/achieved normal acceleration, post-solve residual, and unsupported-shape count. Support, joint stopping, effort, solver time, trajectory clearance, command selection, thermal state, and realized body response remain separate rows.

## Remaining work

The barrier is a local rigid-feature linearization and excludes closest-feature switching and normal curvature. World collision, mesh CCD, authored exclusion matrices, closest-feature continuous rate certificates, calibrated actuator realization, and measured plant response remain separate milestones.
