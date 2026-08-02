# Bonesaw floating collision viability barrier · r108

## Outcome

**PASS.** R108 lifts the R107 closest-feature witness into the floating CPU WBC as a relative-degree-two hard inequality. The query retains closest-pair and limiting-active-pair provenance, distance quality, relative normal velocity, rigid-feature bias acceleration, required/achieved normal acceleration, and post-solve barrier residual.

The fixed box–sphere oracle evaluates 321 independent observed states without a policy, integration, contact response, or external physics. 70 states activate the barrier. Distance, relative velocity, and required-acceleration oracle errors are `2.776e-17` m, `0.000e+00` m/s, and `5.329e-15` m/s². The minimum post-solve barrier residual is `-2.220e-16` m/s².

| query | p50 / p99 / max µs | alloc calls / bytes | exact replay |
|---|---|---|---|
| floating tight barrier | 12.815 / 18.956 / 49.533 | 0 / 0 | True |

The enabled barrier changes the inward joint acceleration by as much as **62.171 m/s²** relative to the collision-disabled WBC query. This is an authority constraint, not a simulated body response.

## Barrier semantics

For `h = distance - hard_margin`, Rust emits `J qdd + bias + 2 ζ ω hdot + ω² h >= 0`. `bias` is the projected rigid-feature `Jdot·v` term. Closest-feature switching and normal-direction curvature are excluded, so this is explicitly a local acceleration barrier rather than exact continuous collision detection. The separately retained R106 sampled/adaptive command gate still owns between-tick trajectory admission.

## Authority boundary

Collision viability remains a separate row from dynamics/contact invariants, finite-support reserve, joint stopping, actuator effort, thermal state, solver budget, trajectory clearance, and command selection. Unsupported authored geometry is counted in the typed evidence instead of being treated as clear. No aggregate health score is produced.

## Remaining work

The r108 WebSocket contract and browser authority stack now consume this typed evidence as an independent Viability row. General authored exclusions, world collision, mesh CCD, closest-feature continuous rate bounds, contact response, and calibrated plant behavior remain separate milestones.
