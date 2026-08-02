# Bonesaw CPU world-SDF collision authority · r109

## Outcome

**PASS.** R109 adds an immutable dense signed-distance grid to the CPU core and samples deterministic body-attached sphere probes against it inside the floating WBC. The independent NumPy plane oracle evaluates 321 caller-authored states without a policy, integration, contact response, or external physics; 193 states activate the world barrier.

Distance, relative-velocity, gradient-norm, and required-acceleration oracle errors are `2.220e-16` m, `5.551e-16` m/s, `1.332e-15`, and `4.974e-14` m/s². The minimum post-solve barrier residual is `-3.553e-15` m/s².

| query | p50 / p99 / max µs | alloc calls / bytes | exact replay |
|---|---|---|---|
| floating world-SDF barrier | 22.933 / 34.376 / 43.302 | 0 / 0 | True |

The barrier changes joint acceleration by as much as **82.281 m/s²** relative to the world-disabled query. This is state-local authority, not simulated obstacle response.

## Unknown-space contract

Every grid node must be finite at construction. The Reject policy withheld the out-of-volume query: **True**. The OccupiedBoundary policy instead produced source code `1`, minimum clearance `-0.814` m, and 4 active probes. The deliberately deep outside fault returned bounded-solve status `4` and was not admitted: **True**. A non-finite grid was rejected: **True**. Unknown space never silently becomes free.

## Barrier semantics

Trilinear interpolation returns both the scalar SDF and its analytic, unnormalized gradient, so the emitted Jacobian is the derivative of the reported scalar rather than a cosmetically normalized normal. For `h = sdf(center) - sphere_radius - hard_margin`, Rust emits `J qdd + bias + 2 ζ ω hdot + ω² h >= 0`. `bias` includes the current gradient projection of rigid-point `Jdot·v`; SDF Hessian curvature and voxel-feature switching remain excluded and explicitly named.

## Authority boundary

World collision is separately typed from self-collision, support, joint stopping, effort, solver budget, sampled/adaptive commanded-segment admission, unknown-space policy, and realized plant response. Sphere covers are conservative robot probes, not mesh CCD. No aggregate health score is produced.

## Remaining work

The world barrier must next be streamed as its own live authority row and paired with commanded-segment world validation. Dynamic SDF updates, external obstacle slots, mesh validation, SDF Hessian/rate bounds, and calibrated plant response remain separate milestones.
