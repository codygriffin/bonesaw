# Bonesaw fresh terminal-box WBC plant A/B · r248

> Mechanism **PASS** · strict plant non-regression **FAIL** · authority **NOT ADMITTED**.

R247's fixed three-candidate laws are evaluated without retuning on two new pyramidal contact laws and disjoint offsets 250,000/260,000. Every pre-impact state is forked: baseline holds zero generalized joint effort and candidate holds the selected torque for five 4 ms MuJoCo steps (one 20 ms, 50 Hz WBC tick). The primitive G1 fixture has two measured collision probes per foot; no policy is queried.

| law | selected · zero / damp / recover | non-regressed | max component regression | harm Δ · p50 / p99 | kinetic J Δ · p50 / p99 | WBC p99 ms | decision |
|---|---|---|---|---|---|---|---|
| compliant_pyramidal_implicitfast_plant_r248 | 48 / 0 / 0 | 14/48 | 26.9 | -0.1811 / 24.63 | -0.9944 / 1.299 | 3.538 | REJECT |
| stiff_pyramidal_rk4_plant_r248 | 43 / 1 / 4 | 12/48 | 44.35 | 0.3575 / 44.04 | -0.7085 / 1.343 | 3.351 | REJECT |

The strict gate is per-sample and componentwise over the terminal proxy's nine pressure/score outputs plus lower-bounded joint headroom. Both completed branches use R222's zero continuation-acceleration contract; instantaneous MuJoCo contact qacc is not extrapolated over the proxy horizon. Penetration, constraint impulse, kinetic energy, warnings, selection, and timing remain separately archived rather than collapsed into that verdict. Passing would promote only this simulated action profile; authority remains closed pending the ordinary-process deadline audit and broader robustness/calibration evidence.
