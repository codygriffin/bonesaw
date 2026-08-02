# Upkie MuJoCo plant audit · upkie-mujoco-plant-r127

**Admission: FAIL.** This is the first retained closed-loop physical differential. Python owns MuJoCo contact, integration, the external torso wrench, and experiment orchestration. Rust receives the observed floating state and returns inverse-dynamics WBC effort. The browser preview and this plant are not conflated.

## Boundary under test

```text
MuJoCo q/v/root ──observed state──> Bonesaw Rust WBC ──torque──> MuJoCo
      ^                                                        │
      └──────────── Python-applied torso wrench ────────────────┘
```

The task-reference adapter is a deterministic upright/posture plus Upkie-like PI wheel-balance composition in Python. It is not learned policy. Moving that robot-specific composition into a fingerprinted Rust tools session is an explicit promotion gate; the policy-/physics-free state-local WBC corpus remains the solver-semantics gate.

## Timing and jitter

| case | Rust p50 µs | Rust p95 µs | Rust p99 µs | full loop p99 µs | >5 ms |
|---|---|---|---|---|---|
| nominal | 143.6 | 8211.9 | 8479.0 | 8691.7 | 118 |
| 8 N torso push | 141.8 | 8102.7 | 8362.5 | 8536.4 | 90 |

## Physical response

| case | peak tilt deg | peak |x| mm | min z m | posture RMS rad | recovery s | fell |
|---|---|---|---|---|---|---|
| nominal | 148.239 | 560.49 | 0.0630 | 1.22687 | — | True |
| push | 148.923 | 535.11 | 0.0634 | 1.23642 | — | True |

## Admission gates

| gate | observed | pass |
|---|---|---|
| finite trace | True | True |
| physical disturbance is observable | peak tilt delta 0.6844 deg | True |
| no fall | True | False |
| post-disturbance recovery | None | False |
| controller timed region allocation-free | 0 calls / 0 bytes | True |
| 5 ms loop budget | 90 overruns | False |
| hard equation residual | 5.973e+01 | False |
| solver remains admitted | {'Solved': 94, 'SolvedWithSlack': 738, 'MaxIterations': 68} | False |

## Resource and authority evidence

- Rust timed-region allocations: **0 calls / 0 bytes**.
- Python GC collections during pushed loop: **0**; RSS delta: **0.34 MiB**.
- Peak actuator effort utilization: **0.8794**.
- Maximum hard dynamics/contact residual: **5.973e+01 / 2.087e+01**.
- Solver status counts: **{'Solved': 94, 'SolvedWithSlack': 738, 'MaxIterations': 68}**.

## Deliberate limits

This plant is an evaluation adapter, not a reference implementation of MuJoCo dynamics inside Bonesaw. The URDF is compiled by MuJoCo with generated sagittal root joints (x, z, and pitch), a ground plane, direct torque motors, and small declared damping/armature. MuJoCo wheel/ground contact and Bonesaw's two rigid rolling constraints are not identical; that mismatch is part of the test. There is no estimator delay/noise, motor bandwidth, thermal model, terrain change, contact-mode estimator, network, or hardware calibration in this checkpoint.

The physical browser gesture remains disabled until this adapter is connected to the live WebSocket process. A green target must never be relabeled as a push. The retained CSV traces contain every control tick for independent review.
