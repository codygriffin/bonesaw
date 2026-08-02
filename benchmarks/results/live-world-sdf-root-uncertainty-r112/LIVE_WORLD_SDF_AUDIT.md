# Bonesaw live floating-root-error + swept world-SDF authority · r112

## Outcome

**PASS.** The r112 WebSocket contract retains the local world-SDF Viability row and exposes floating-root prediction, deterministic forecast-error erosion, sampled command-world geometry, and continuous certification as separate rows across 60 guided 50 Hz frames during an 80.0 mm vertical torso disturbance. This is a state-local authority query plus policy-/physics-free command admission, not a policy, simulator, probability claim, or plant rollout.

| signal | retained value |
|---|---|
| closest clearance / hard-margin reserve | 66.409 / 46.409 mm |
| active probe count | 7–7 |
| closest / limiting bodies | {"left_hand": 60} / {"left_hand": 60} |
| field / unknown-space policy | {"trilinear_grid": 60} / {"reject": 60} |
| minimum barrier residual | 7.329e+00 m/s² |
| raw WBC p50 / p99 / max | 1.877 / 2.913 / 3.066 ms |
| command admission p50 / p99 / max | 1.446 / 2.446 / 2.517 ms |
| primary sampled / continuous world minimum | 64.661 / 64.602 mm |
| brake sampled / continuous world minimum | 64.661 / 64.602 mm |
| root translation / attitude error radius | 0.900 mm / 0.039° |
| primary / brake prediction erosion | 1.748 / 1.748 mm |

All frames used the trilinear grid path with unit-gradient error at most `8.882e-16` and explicit `reject` unknown-space policy. The minimum achieved-minus-required acceleration was `7.329e+00` m/s². Maximum dynamics/contact residuals were `1.670e-09` / `4.899e-11`, and the separate self-collision command certificate retained at least `19.601` mm. Selector counts were `{"contingency": 4, "primary": 55, "rejected": 1}`: the world primary and brake stayed clear throughout, while separately typed self-collision authority still caused the retained contingency/reject decisions.

## Command-world witnesses

Every primary and brake quintic was sampled at 21 deterministic 1 ms knots against the same immutable SDF, then conservatively bounded between knots. The primary limiting body counts were `{"left_hand": 31, "left_upper_arm": 29}`. No sampled violation or unknown-space witness occurred: **True**. Primary work used at most depth 0 with 0 unresolved leaves; safe broad certificates legitimately required zero midpoint probes in this trace.

## Typed boundary

The local row carries closest and limiting stable probe/body IDs, active count, conservative proxy quality, SDF source, gradient norm, signed clearance and hard-margin reserve, normal velocity, required/achieved acceleration, post-solve residual, unsupported geometry, and unknown-space policy. The command rows separately carry primary/brake minima, field source, first violation or unknown time, continuous limiting probe/body, rate bound, leaf/midpoint/unresolved/depth work, flags, and final selection. None is aggregated with self-collision, support, joint stopping, effort, thermal state, solver budget, or body response.

## Bounded failure evidence

The first demonstration placement began 3.591 mm inside the represented wall. Feasibility exhausted its bounded work after 1,767,200 halfspace projections with 5.475e-3 maximum violation, returned `MaxIterations`, zeroed the executable candidate, and withheld the frame. The displayed placement begins outside the hard margin so the live row can be inspected continuously. `MaxIterations` is budget exhaustion, not an infeasibility proof.

## Remaining work

The local barrier excludes SDF Hessian curvature and voxel-feature switching. Swept command validation uses a measured global field Lipschitz bound, explicit floating-root paths, compiled angular reach, and declared deterministic root-error growth; it is conservative sphere-probe evidence, not covariance, arbitrary mesh CCD, or a calibrated estimator claim. Mutable/dynamic map epochs, command-history or forward-dynamics predictors, mesh-specific probes, calibrated actuation, and measured plant response remain separate milestones.
