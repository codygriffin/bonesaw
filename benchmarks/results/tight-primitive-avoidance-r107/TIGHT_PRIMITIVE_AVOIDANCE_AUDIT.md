# Bonesaw shared tight-primitive avoidance · r107

## Outcome

**PASS.** R107 derives the differential avoidance row from the same tight primitive pair and signed-distance routine used by command admission. The witness carries a deterministic normal, closest surface points, locally selected rigid-body feature points, stable pair ID, distance quality, relative normal velocity, and analytic Jacobian. The previous sphere-cover evaluator remains a separately named fallback API.

The fixed box–sphere fixture sweeps 321 states without a policy or physics rollout. Tight geometry has 1 pair; the cover needs 3. Maximum cover overreach is **131.046 mm**. It creates 132 false influence samples and 122 false hard-collision samples that the shared primitive witness does not report.

| representation | pairs | p50 / p99 / max µs | alloc calls / bytes | exact replay |
|---|---|---|---|---|
| tight | 1 | 0.311 / 0.391 / 6.181 | 0 / 0 | True |
| sphere_cover_fallback | 3 | 0.501 / 0.571 / 0.751 | 0 / 0 | True |

The analytic distance matches the fixture oracle within `2.776e-17` m. Its Jacobian matches a finite-difference distance derivative within `5.839e-10`; normal and surface-witness errors are `0.000e+00` and `5.551e-17` m. Both representations replay exactly and allocate zero calls/bytes inside the timed Rust query.

## Authority boundary

This removes representation disagreement between avoidance and command admission for supported spheres, capsules, cylinders-as-capsules, and boxes. It does not make the browser pose a realized plant, and it does not aggregate avoidance pressure with support, effort, thermal, solver, or command-selection authority. Box–box still reports the conservative separating-axis lower bound and labels that quality explicitly.

## Remaining work

The fixed-root kinematic controller now consumes shared primitive rows; the floating dynamic WBC still needs the corresponding acceleration-level barrier and typed active-pair telemetry. World geometry, authored exclusion matrices, mesh CCD, closest-feature continuous-rate bounds, contact response, and calibrated plant behavior remain separate milestones.
