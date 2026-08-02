# Upkie MuJoCo plant audit · upkie-rooted-capture-plant-r130

**Admission: PASS.** This is a retained closed-loop plant differential using the **soft** contact model and **capture** Rust balance composition. Python owns MuJoCo integration, the external torso wrench, and experiment orchestration. Rust receives the observed floating state and returns inverse-dynamics WBC effort plus its rigid-contact witness. In `soft` mode MuJoCo owns collision response; in `prescribed` mode the Rust solved wrench is applied to MuJoCo as an ideal integration/correspondence bridge. The browser preview and this plant are not conflated.

## Boundary under test

```text
MuJoCo q/v/root ──observed state──> Bonesaw Rust WBC ──torque──> MuJoCo
      ^                                                        │
      └──────────── Python-applied torso wrench ────────────────┘
```

The task-reference adapter is deterministic and not learned policy. A fingerprinted Rust tools session owns the Upkie PI state, clamps, signed rolling coordinates, balanced-standing morphology projection, and axle-to-CoM virtual-pitch observation. Capture mode presents **0.200×** of the full DCM velocity offset to that PI loop while the full offset remains the viability-pressure witness. Python owns only MuJoCo state transfer, scenario timing, disturbance, and scoring. The policy-/physics-free state-local WBC corpus remains the solver-semantics gate.

## Timing and jitter

| case | Rust p50 µs | Rust p95 µs | Rust p99 µs | full loop p99 µs | >5 ms |
|---|---|---|---|---|---|
| nominal | 123.6 | 132.7 | 151.2 | 387.4 | 0 |
| 4 N recovery probe | 125.2 | 139.9 | 166.0 | 446.9 | 0 |
| 6 N overload probe | 8125.2 | 9593.8 | 10260.3 | 10582.8 | 1057 |

## Physical response

| case | peak tilt deg | peak Δx mm | min z m | posture RMS rad | recovery s | fell |
|---|---|---|---|---|---|---|
| nominal | 0.045 | 0.09 | 0.5393 | 0.02859 | 0.000 | False |
| recovery probe | 24.907 | 107.63 | 0.5361 | 0.06453 | 1.450 | False |
| overload | 178.870 | 8125.71 | 0.0087 | 1.48379 | 1.755 | True |

## Admission gates

| gate | observed | pass |
|---|---|---|
| finite trace | True | True |
| physical disturbance is observable | peak tilt delta 24.8619 deg | True |
| nominal standing remains stable | peak 0.0450 deg; fall False | True |
| no fall | False | True |
| post-disturbance recovery | 1.4500000000000002 | True |
| station preference re-enters after recovery | 1.87 s; final error -0.000007 m; authority 1.000000 | True |
| controller timed region allocation-free | 0 calls / 0 bytes | True |
| 5 ms loop budget | 0 overruns | True |
| admitted hard equation residual | 8.031e-09 | True |
| bounded fail-closed startup | 5 ticks; longest 5; last 0.025 s | True |
| solver admitted after startup | 0 nonadmitted ticks | True |
| contact is reacquired | 0.005 s maximum flight; last 1.645 | True |
| overload boundary is discriminating | 6 N fall True; 1044 later nonadmitted ticks | True |

## Resource and authority evidence

- Rust timed-region allocations: **0 calls / 0 bytes**.
- Python GC collections during pushed loop: **0**; RSS delta: **0.97 MiB**.
- Peak actuator effort utilization: **0.0837**.
- Peak capture error/pressure and minimum station authority: **0.0333 m / 1.0000 / 0.0000**.
- Station preference re-entry: **1.870 s after push end**; final error **-0.000007 m** at authority **1.0000**.
- Maximum admitted dynamics/contact residual: **7.125e-09 / 8.031e-09**. Rejected diagnostic candidates are reported separately at **4.839e-07 / 1.416e-07** and are never executed.
- Recovery-probe solver status counts: **{'Solved': 1992, 'SolvedWithSlack': 3, 'NumericalOrInvalid': 5}**. The fail-closed startup hold lasts **5 ticks**, ending at **0.025 s**; admitted effort is then continuous through the disturbance.
- Recovery-probe contactless interval: **5.0 ms**; last contactless sample **1.645**. Temporary flight is not called rigid-contact admission.

## Measured recovery boundary

The declared recovery probe is **4 N for 100 ms (0.400 N·s)** and recovers in **1.450 s**. The otherwise identical **6 N (0.600 N·s)** probe falls. This brackets one forward-push envelope only when the recovery probe passes and the overload probe fails; it is never an all-direction robustness claim.

## Deliberate limits

This plant is an evaluation adapter, not a reference implementation of MuJoCo dynamics inside Bonesaw. An explicit floating root is inserted before URDF import, static-body fusion is disabled to preserve Upkie's rotated fixed-link inertias, and the plant uses a ground plane plus direct URDF-limited torque motors without invented armature or damping. MuJoCo wheel/ground contact and Bonesaw's two rigid rolling constraints are not identical; that mismatch is part of the test. There is no estimator delay/noise, motor bandwidth, thermal model, terrain change, contact-mode estimator, network, or hardware calibration in this checkpoint. Per-tick Python dictionary/array boundary traffic remains outside the Rust allocation witness even though Python GC is zero; replacing it with fixed caller-owned plant outputs is still open.

The physical browser gesture remains disabled until this adapter is connected to the live WebSocket process. A green target must never be relabeled as a push. The retained CSV traces contain every control tick for independent review.
