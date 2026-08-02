# Bonesaw WBC benchmark manifest · r260

> This report composes pinned Python evaluation artifacts; it does not rerun policy or plant code and admits no authority.

## Coverage matrix

| Boundary | Evidence | Result |
|---|---|---|
| End-effector reach | fixed-base Bonesaw vs PlaCo, 5,000 ticks | recorded |
| Bimanual priority conflict | shared target with strict-vs-soft reference | recorded |
| Walking retarget | CMU subject 37/01, three cadences | FAIL (retained red gate) |
| Floating moving liftoff | measured q/v, contact transition, 250/50 Hz contract | PASS |
| Floating full transfer | same contract through support transfer | FAIL (retained stress case) |
| Upkie WheelBalancer | pinned upstream C++ parity and resource profile | PASS |

## Fixed-base tracking and timing

| Scenario | Bonesaw RMS | PlaCo RMS | Bonesaw p99 | PlaCo p99 |
|---|---:|---:|---:|---:|
| end_effector_reach | 0.630 cm | 0.809 cm | 40.5 µs | 108.1 µs |
| bimanual_priority_conflict | 0.475 cm | 0.365 cm | 83.4 µs | 118.4 µs |
| walking_motion_retarget | 4.048 cm | 2.634 cm | 67.1 µs | 143.0 µs |

The fixed-base reach and conflict rows are useful end-effector and priority-regression sentinels. Walking remains a deliberate red behavior row: Bonesaw's recorded overall foot RMS is 5.549 cm against the 5 cm gate, while the independent PlaCo row misses swing-clearance RMS at 3.301 cm. Neither failure is hidden by the faster Rust path.

## Floating measured-feedback behavior

| Profile | Ticks | Nominal prefix | Full p99 | Full dynamics residual | Full acceptance |
|---|---:|---:|---:|---:|---|
| moving_liftoff | 260 | 260 | 3738.0 µs | 1.22e-09 | PASS |
| full_transfer_stress | 600 | 351 | 276773.8 µs | 304 | FAIL |

The moving-liftoff prefix passes the existing behavior and 5 ms p99 gate. The 600-tick transfer stress remains red: the measured-feedback path records contact contingency, infeasible timing/residual windows, and a 281-tick 20 ms deadline miss. This is the highest-value WBC behavior gap and is retained as a required contact-mode/retargeting improvement, not converted into a reset or authority decision.

## Upkie reference

The official aligned law is canonical-bitwise equal with 0 mismatches; Bonesaw p50/p99 is 0.030/0.031 µs against upstream 1.393/2.385 µs. The measured Rust hot loop reports zero allocation calls and bytes. Ten temporal windows are retained for drift/jitter inspection.

No row in this manifest authorizes a plant command. The next behavior checkpoint should improve the floating support-transition path, then rerun the same immutable metric schema with over-time windows, contact/CoP/friction margins, actuator/resource health, and reference parity intact.
