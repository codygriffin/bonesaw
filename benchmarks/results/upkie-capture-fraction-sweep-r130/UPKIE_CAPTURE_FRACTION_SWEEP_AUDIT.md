# Upkie capture-fraction sweep · upkie-capture-fraction-sweep-r130

**Admission: PASS.** Seven predeclared actuator-facing fractions of the full DCM velocity offset are exercised against the same 10-second, 4 N × 100 ms forward-push MuJoCo plant. This is a parameter robustness study, not a learned-policy comparison. Python owns the plant and scoring; Rust owns capture/station semantics, reference PI state, WBC, admission, and allocation evidence.

The full DCM remains the viability-pressure witness at every fraction. The swept value only changes how much of `com_velocity / omega` is presented as a pitch reference to the Upkie-matched PI loop.

## Results

| fraction | qualified | peak tilt deg | tilt recovery s | station re-entry s | peak Δx m | final station error m | later rejects | Rust p99 µs | >5 ms |
|---|---|---|---|---|---|---|---|---|---|
| 0.00 | False | 24.93 | 1.435 | — | 1.7432 | -1.743205 | 0 | 150.7 | 0 |
| 0.05 | True | 24.93 | 1.455 | 3.530 | 0.1520 | -0.000528 | 0 | 155.5 | 0 |
| 0.10 | True | 24.92 | 1.455 | 2.860 | 0.1136 | -0.000484 | 0 | 148.3 | 0 |
| 0.20 | True | 24.91 | 1.450 | 1.870 | 0.1076 | -0.000007 | 0 | 182.9 | 0 |
| 0.40 | True | 24.87 | 1.410 | 5.620 | 0.1002 | -0.007800 | 0 | 146.6 | 0 |
| 0.60 | False | 174.33 | 1.455 | — | 1.7599 | 0.343567 | 609 | 10301.7 | 616 |
| 1.00 | False | 179.58 | — | — | 1.2590 | 0.306292 | 1017 | 10417.7 | 1023 |

- Largest plant-qualified fraction: **0.40**.
- Recommended fraction: **0.20**, selected for the fastest station re-entry among qualified rows rather than maximum velocity feedback.
- Frozen default under review: **0.20**; qualified: **True**.
- Sweep is discriminating above the qualified envelope: **True**.
- Every qualified row requires no fall, tilt recovery, station-authority re-entry, final station error below 5 cm, bounded fail-closed startup, continuous admission afterward, contact reacquisition, zero Rust timed-region allocations, and zero 5 ms loop overruns.

## Boundary and limits

Each fraction receives a fresh plant, WBC session, PI state, and balanced-standing projection. The retained NPZ contains root pose, capture pressure, station error/authority, status, and controller/loop timing for every tick and fraction. This sweep covers one sagittal impulse, one contact model, and one ideal-observation condition; it does not establish lateral, terrain, delay/noise, thermal, or hardware robustness.
