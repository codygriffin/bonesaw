# Upkie rooted capture audit · upkie-rooted-capture-r129

**Admission: PASS.** This is a policy-component, physics-free corpus. Immutable oracle states are supplied on every call; there is no MuJoCo, integration, contact response, learned policy, WBC solve, or browser animation. Rust owns the one-dimensional DCM/capture computation, station-authority fade, reference-matched PI update, wheel-coordinate lowering, rooted frame evaluation, and state. Python owns only corpus construction, timing, scoring, and artifacts.

## Rooted authority boundary

```text
map ──possibly discontinuous reporting correction──> odom
                                                     │
control_world <──smooth external transform───────────┘
      │
      ├── measured CoM + velocity ──> capture viability
      └── odom station anchor ──────> fading preference
```

The authority witness is the full capture state `com_x + com_velocity_x / sqrt(g / height)`. The actuator-facing reference presents **0.200×** of its velocity offset to the Upkie-matched PI loop. Station authority is one inside the release band, follows a C1 smoothstep through the transition band, and is exactly zero at full capture pressure. A `map → odom` correction is consumed for reporting but has no path into capture or station commands.

## Admission gates

| gate | observed | pass |
|---|---|---|
| finite corpus | True | True |
| exact D1 replay | True | True |
| map jump cannot change command/control diagnostics | maximum delta 0.000e+00 | True |
| map jump remains visible in reporting | 10.000000 m | True |
| full-pressure capture command is monotone in CoM velocity | minimum adjacent full-pressure delta 2.684e-03 | True |
| authority blend is continuous at corpus resolution | maximum adjacent command delta 5.763e-03 | True |
| station authority fades monotonically with capture pressure | center 1.000000; edges 0.000000/0.000000 | True |
| station target cannot oppose full-pressure capture | wheel delta 0.000e+00 | True |
| smooth odom transform moves station preference | target delta 1.000000 m | True |
| invalid rooted quaternion rejects atomically | True | True |
| boundary p99 below 1 ms | 6.31 µs | True |

## Boundary timing

| calls | p50 µs | p95 µs | p99 µs | p99.9 µs | max µs |
|---|---|---|---|---|---|
| 10000 | 4.52 | 6.11 | 6.31 | 12.34 | 29.27 |

- Exact D1 replay: **True** across wheel commands and all sixteen diagnostics.
- Map-jump command delta: **0.000e+00**; reporting delta: **10.000000 m**.
- Python GC collections: **0**; RSS delta: **7.469 MiB**.
- Rust hot-path allocation check: **PASS** (every call would return an error on any allocation).

## Deliberate limits

This proves rooted frame isolation and continuous reference semantics, not body response or WBC feasibility. The controller is sagittal and assumes the Upkie rolling axis. It has no contact-mode estimator, lateral capture set, terrain model, actuator bandwidth, delay/noise, thermal state, or hardware calibration. The separate r128 MuJoCo report remains the current physical consequence gate until this reference passes a new plant envelope.
