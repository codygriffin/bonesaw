# Bonesaw terminal-box WBC action audit · r247

> State-local mechanism **PASS** · plant non-regression **NOT RUN** · authority **NOT ADMITTED**.

This design audit evaluates three fixed acceleration laws on all 96 spent R246 pre-impact states. The exact Rust floating WBC admits each law independently, then the R224 componentwise terminal box and R247 Rust selector choose only candidates with zero component regression. No policy is queried, no physics is stepped, and no selected acceleration or torque is applied.

| law | selected · zero / damp / recover | strictly improved | max regression | improvement · p50 / p99 | WBC p99 ms | selector p99 µs | WBC |
|---|---|---|---|---|---|---|---|
| medium_pyramidal_implicitfast_surface_r246 | 48 / 0 / 0 | 0 | 0 | 0.000 / 0.000 | 3.218 | 9.759 | PASS |
| rigid_pyramidal_rk4_surface_r246 | 37 / 8 / 3 | 11 | 0 | 0.000 / 1.332 | 3.211 | 3.121 | PASS |

Combined selection is **85 / 8 / 3** over 96 states. The independent full rerun reproduces 34/34 non-timing arrays bitwise; WBC and selector hot paths report zero Rust allocation.

Causality boundary: the evaluator opens only R246 root height and model-predicted contact activation per sample. Its uncertainty widths were already fit on the spent R246 completed labels, so this is deliberately not fresh evidence. The frozen candidate laws must next face a new MuJoCo baseline/candidate plant matrix before command admission can be reconsidered.
