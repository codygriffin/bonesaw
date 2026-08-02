# Bonesaw fresh terminal-box WBC plant A/B · r248

> Mechanism **PASS** · strict plant non-regression **FAIL** · authority **NOT ADMITTED**.

R247's fixed three-candidate laws are evaluated without retuning on two new pyramidal contact laws and disjoint offsets 250,000/260,000. Every pre-impact state is forked: baseline holds zero generalized joint effort for five MuJoCo substeps; candidate holds the torque selected before either branch advances. No policy is queried.

| law | selected · zero / damp / recover | non-regressed | max component regression | harm Δ · p50 / p99 | kinetic J Δ · p50 / p99 | WBC p99 ms | decision |
|---|---|---|---|---|---|---|---|
| compliant_pyramidal_implicitfast_plant_r248 | 48 / 0 / 0 | 2/48 | 127.5 | 25.34 / 114.5 | -0.1623 / 1.069 | 3.554 | REJECT |
| stiff_pyramidal_rk4_plant_r248 | 41 / 3 / 4 | 0/48 | 150.2 | 37.5 / 142.7 | -0.2748 / 1.272 | 3.303 | REJECT |

The strict gate is per-sample and componentwise over the terminal proxy's nine pressure/score outputs plus lower-bounded joint headroom. Both completed branches use R222's zero continuation-acceleration contract; instantaneous MuJoCo contact qacc is not extrapolated over the proxy horizon. Penetration, constraint impulse, kinetic energy, warnings, selection, and timing remain separately archived rather than collapsed into that verdict. Passing would promote only this simulated action profile; authority remains closed pending the ordinary-process deadline audit and broader robustness/calibration evidence.
