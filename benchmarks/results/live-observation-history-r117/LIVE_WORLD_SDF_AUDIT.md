# Bonesaw live canonical observation + tracking + robust world-SDF authority · r117

## Outcome

**PASS.** The r117 WebSocket contract routes every floating-WBC query through the fixed-capacity canonical observation history before timestamp admission, observed-versus-commanded tracking, local world-SDF Viability, scene epoch/freshness, floating-root prediction, deterministic forecast-error erosion, sampled command-world geometry, and continuous certification. Across 60 guided 50 Hz frames during an 80.0 mm vertical torso disturbance, reconstruction provenance was `{"exact": 60}` and every reconstructed state was hard-eligible: **True**. Every observation was causal, fresh, and synchronized: **True**. This is canonical state-local authority plus policy-/physics-free command admission, not a map-frame jump, policy, simulator, probability claim, fault diagnosis, or plant rollout.

| signal | retained value |
|---|---|
| closest clearance / hard-margin reserve | 66.409 / 46.409 mm |
| active probe count | 7–7 |
| closest / limiting bodies | {"left_hand": 60} / {"left_hand": 60} |
| field / unknown-space policy | {"trilinear_grid": 60} / {"reject": 60} |
| minimum barrier residual | 7.329e+00 m/s² |
| raw WBC p50 / p99 / max | 1.872 / 2.942 / 3.065 ms |
| command admission p50 / p99 / max | 1.430 / 1.979 / 1.985 ms |
| primary sampled / continuous world minimum | 64.661 / 64.602 mm |
| brake sampled / continuous world minimum | 64.661 / 64.602 mm |
| root translation / attitude error radius | 0.900 mm / 0.039° |
| primary / brake prediction erosion | 1.748 / 1.748 mm |
| world scene epoch / validity | [115] / {"valid": 60} |
| world scene age p50 / p99 | 625.000 / 1203.200 ms |
| scene covers full command horizon | True |
| observation source / sequence range | {"0xb015": 60} / [8, 244] |
| observation age p50 / minimum headroom | 0.000 / 10.000 ms |
| sync uncertainty p50 / minimum headroom | 0.000 / 2.000 ms |
| all observations causal / fresh / synchronized | True |
| canonical reconstruction provenance | {"exact": 60} |
| history occupancy / capacity | [5, 64] / 64 |
| history ingest accepted / ignored / rejected | 60 / 0 / 0 |
| all reconstructions hard eligible | True |
| tracking action counts | {"contingency": 6, "nominal": 54} |
| max |observed−commanded| position / velocity | 0.2732 / 1.4944 |
| minimum tracking contingency headroom q / v | -0.0232 / 0.5056 |

All frames used the trilinear grid path with unit-gradient error at most `8.882e-16` and explicit `reject` unknown-space policy. The minimum achieved-minus-required acceleration was `7.329e+00` m/s². Maximum dynamics/contact residuals were `1.670e-09` / `4.899e-11`, and the separate self-collision command certificate retained at least `37.312` mm. Selector counts were `{"contingency": 6, "primary": 54}`: world and self geometry stayed clear, while the six typed tracking-contingency frames withheld feed-forward authority and selected the independently valid brake.

## Command-world witnesses

Every primary and brake quintic was sampled at 21 deterministic 1 ms knots against the same immutable SDF, then conservatively bounded between knots. The primary limiting body counts were `{"left_hand": 31, "left_upper_arm": 29}`. No sampled violation or unknown-space witness occurred: **True**. Primary work used at most depth 0 with 0 unresolved leaves; safe broad certificates legitimately required zero midpoint probes in this trace.

## Typed boundary

The reconstruction row carries ring occupancy/capacity, ingest dispositions, exact/interpolated/predicted/held provenance, lower/upper source timestamps, stable source IDs/sequences, source age, synchronization headroom, and hard eligibility. The WBC no longer reads the producer-owned floating state directly. The subsequent observation row independently checks timestamp authority, while tracking carries actuator mismatch and two-stage headroom. Local and swept geometry, resources, solver work, final selection, and plant response remain separate; none becomes one health score.

## Bounded failure evidence

The first demonstration placement began 3.591 mm inside the represented wall. Feasibility exhausted its bounded work after 1,767,200 halfspace projections with 5.475e-3 maximum violation, returned `MaxIterations`, zeroed the executable candidate, and withheld the frame. The displayed placement begins outside the hard margin so the live row can be inspected continuously. `MaxIterations` is budget exhaustion, not an infeasibility proof.

## Remaining work

The local barrier excludes SDF Hessian curvature and voxel-feature switching. Swept command validation uses a measured global field Lipschitz bound, explicit floating-root paths, compiled angular reach, and declared deterministic root-error growth; it is conservative sphere-probe evidence, not covariance, arbitrary mesh CCD, or a calibrated estimator claim. R117 proves the exact-sample live path. Injected delayed/interpolated/predicted live transport faults and conservative reconstruction-error erosion into joint/collision margins remain separate gates, as do fixed-capacity scene replacement, mesh-specific probes, calibrated actuation, and measured plant response.
