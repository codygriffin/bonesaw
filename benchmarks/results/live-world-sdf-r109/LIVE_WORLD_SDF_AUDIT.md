# Bonesaw live world-SDF viability · r109

## Outcome

**PASS.** The r109 WebSocket contract retains an independent world-SDF Viability row across 60 guided 50 Hz frames during an 80.0 mm vertical torso disturbance. This is a state-local authority query, not a policy, simulator, or plant rollout.

| signal | retained value |
|---|---|
| closest clearance / hard-margin reserve | 66.409 / 46.409 mm |
| active probe count | 7–7 |
| closest / limiting bodies | {"left_hand": 60} / {"left_hand": 60} |
| field / unknown-space policy | {"trilinear_grid": 60} / {"reject": 60} |
| minimum barrier residual | 7.329e+00 m/s² |
| raw WBC p50 / p99 / max | 1.982 / 3.111 / 3.175 ms |

All frames used the trilinear grid path with unit-gradient error at most `8.882e-16` and explicit `reject` unknown-space policy. The minimum achieved-minus-required acceleration was `7.329e+00` m/s². Maximum dynamics/contact residuals were `1.670e-09` / `4.899e-11`, and the separate self-collision command certificate retained at least `19.601` mm.

## Typed boundary

The row carries closest and limiting stable probe/body IDs, active count, conservative proxy quality, SDF source, gradient norm, signed clearance and hard-margin reserve, normal velocity, required/achieved acceleration, post-solve residual, unsupported geometry, and unknown-space policy. It remains separate from self-collision, support, joint stopping, effort, solver work, trajectory admission, thermal state, and body response.

## Bounded failure evidence

The first demonstration placement began 3.591 mm inside the represented wall. Feasibility exhausted its bounded work after 1,767,200 halfspace projections with 5.475e-3 maximum violation, returned `MaxIterations`, zeroed the executable candidate, and withheld the frame. The displayed placement begins outside the hard margin so the live row can be inspected continuously. `MaxIterations` is budget exhaustion, not an infeasibility proof.

## Remaining work

The local barrier excludes SDF Hessian curvature and voxel-feature switching. Swept command validation against world geometry, mutable/dynamic maps, mesh-specific probes, calibrated actuation, and measured plant response remain separate milestones.
