# Bonesaw paired terminal-score plant holdout · r254

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED**.

R254 is the one-shot no-refit plant gate declared by R253. It uses new elliptic/implicitfast and pyramidal/Euler laws at untouched offsets 310,000 and 320,000. Each of 96 states produces exact zero, realized zero-WBC, and realized neutral-recovery candidates; all three execute five 4 ms MuJoCo steps. The fixed R253 closing-speed/tilt group boxes, zero-regression threshold, and 0.01 guaranteed-improvement threshold are consumed without fitting or widening. No policy runs.

| fresh law | offset | selected zero / zero-WBC / neutral | strict safe + improved / nonzero | max selected regression | all boxes covered | warnings |
|---|---|---|---|---|---|---|
| medium_elliptic_euler_residual_r265 | 330000 | 46 / 0 / 2 | 0 / 2 | 9.2 | NO | 0 |
| stiff_elliptic_rk4_residual_r265 | 340000 | 48 / 0 / 0 | 0 / 0 | 0 | NO | 0 |

The run executes 1,440 fresh MuJoCo steps. Frozen-box coverage is incomplete; 2 rows select nonzero effort, 0 improve aggregate consequence, but only 0 is simultaneously component-nonregressing and aggregate-improving. Joint-position pressure and its headroom transform are the largest transfer misses on both laws; aggregate boxes miss too. WBC/selector p99 are 3252.78/0.10 µs with zero measured Rust allocation. The profile is retained as rejected evidence; no widening, refit, command, or authority follows.
