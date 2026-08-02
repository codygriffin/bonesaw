# Upkie MuJoCo plant audit · upkie-mujoco-plant-r128

**Admission: PASS.** This is a retained closed-loop plant differential using the **soft** contact model. Python owns MuJoCo integration, the external torso wrench, and experiment orchestration. Rust receives the observed floating state and returns inverse-dynamics WBC effort plus its rigid-contact witness. In `soft` mode MuJoCo owns collision response; in `prescribed` mode the Rust solved wrench is applied to MuJoCo as an ideal integration/correspondence bridge. The browser preview and this plant are not conflated.

## Boundary under test

```text
MuJoCo q/v/root ──observed state──> Bonesaw Rust WBC ──torque──> MuJoCo
      ^                                                        │
      └──────────── Python-applied torso wrench ────────────────┘
```

The task-reference adapter is deterministic and not learned policy. A fingerprinted Rust tools session owns the Upkie PI state, clamps, signed rolling coordinates, balanced-standing morphology projection, and axle-to-CoM virtual-pitch observation. Python owns only MuJoCo state transfer, scenario timing, disturbance, and scoring. The policy-/physics-free state-local WBC corpus remains the solver-semantics gate.

## Timing and jitter

| case | Rust p50 µs | Rust p95 µs | Rust p99 µs | full loop p99 µs | >5 ms |
|---|---|---|---|---|---|
| nominal | 129.5 | 175.1 | 188.7 | 455.6 | 0 |
| 4 N recovery probe | 130.7 | 146.5 | 164.1 | 415.7 | 0 |
| 6 N overload probe | 140.4 | 9299.4 | 9969.5 | 10183.0 | 271 |

## Physical response

| case | peak tilt deg | peak Δx mm | min z m | posture RMS rad | recovery s | fell |
|---|---|---|---|---|---|---|
| nominal | 0.006 | 0.16 | 0.5393 | 0.02860 | 0.000 | False |
| recovery probe | 24.920 | 137.49 | 0.5374 | 0.08408 | 1.440 | False |
| overload | 172.693 | 475.51 | 0.0795 | 1.19640 | — | True |

## Admission gates

| gate | observed | pass |
|---|---|---|
| finite trace | True | True |
| physical disturbance is observable | peak tilt delta 24.9141 deg | True |
| nominal standing remains stable | peak 0.0063 deg; fall False | True |
| no fall | False | True |
| post-disturbance recovery | 1.44 | True |
| controller timed region allocation-free | 0 calls / 0 bytes | True |
| 5 ms loop budget | 0 overruns | True |
| admitted hard equation residual | 7.105e-09 | True |
| bounded fail-closed startup | 4 ticks; longest 4; last 0.02 s | True |
| solver admitted after startup | 0 nonadmitted ticks | True |
| contact is reacquired | 0.005 s maximum flight; last 1.56 | True |
| overload boundary is discriminating | 6 N fall True; 267 later nonadmitted ticks | True |

## Resource and authority evidence

- Rust timed-region allocations: **0 calls / 0 bytes**.
- Python GC collections during pushed loop: **0**; RSS delta: **0.34 MiB**.
- Peak actuator effort utilization: **0.0730**.
- Maximum admitted dynamics/contact residual: **7.105e-09 / 5.207e-09**. Rejected diagnostic candidates are reported separately at **5.012e-07 / 4.231e-07** and are never executed.
- Recovery-probe solver status counts: **{'Solved': 894, 'SolvedWithSlack': 2, 'NumericalOrInvalid': 4}**. The fail-closed startup hold lasts **4 ticks**, ending at **0.020 s**; admitted effort is then continuous through the disturbance.
- Recovery-probe contactless interval: **5.0 ms**; last contactless sample **1.56**. Temporary flight is not called rigid-contact admission.

## Measured recovery boundary

The declared recovery probe is **4 N for 100 ms (0.400 N·s)** and recovers in **1.440 s**. The otherwise identical **6 N (0.600 N·s)** probe falls and loses solver timing/admission. This brackets one forward-push envelope for this exact model and contact configuration; it is not an all-direction robustness claim.

## Deliberate limits

This plant is an evaluation adapter, not a reference implementation of MuJoCo dynamics inside Bonesaw. An explicit floating root is inserted before URDF import, static-body fusion is disabled to preserve Upkie's rotated fixed-link inertias, and the plant uses a ground plane plus direct URDF-limited torque motors without invented armature or damping. MuJoCo wheel/ground contact and Bonesaw's two rigid rolling constraints are not identical; that mismatch is part of the test. There is no estimator delay/noise, motor bandwidth, thermal model, terrain change, contact-mode estimator, network, or hardware calibration in this checkpoint. Per-tick Python dictionary/array boundary traffic remains outside the Rust allocation witness even though Python GC is zero; replacing it with fixed caller-owned plant outputs is still open.

The physical browser gesture remains disabled until this adapter is connected to the live WebSocket process. A green target must never be relabeled as a push. The retained CSV traces contain every control tick for independent review.
