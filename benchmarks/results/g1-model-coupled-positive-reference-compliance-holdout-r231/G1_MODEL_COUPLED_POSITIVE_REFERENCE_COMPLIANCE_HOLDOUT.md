# Bonesaw G1 model-coupled fresh holdout · r231

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · policy/controller/selector/plant steps **0 / 0 / 0 / 0**.

## Contract

- R230 froze five 1 ms model-state/event steps, one compliant update per state step, 32 complete-Delassus projection sweeps, 5 mm sphere support geometry, and the 0.168/0.023/9.893 angular/linear/joint residual box before these labels.
- Medium/pyramidal/implicit-fast and hard/elliptic/RK4 laws use untouched offsets 110,000 and 120,000. Every sample resets; MuJoCo supplies only the completed five-step score label.
- Strict coverage, useful frozen width, 5 ms p99 deadline, bitwise repeat, and zero timed Rust allocation are conjunctive. Passing promotes only this transition profile to bounded terminal selection/non-regression work.
- A complete retained rerun reproduced all 60 non-timing NPZ arrays exactly. Timing is retained as measured evidence and excluded from semantic equality.

## Fresh result

| law | sample coverage | component coverage | exact active sets | fitted width ω/v/joint | p99 ms | profile |
|---|---|---|---|---|---|---|
| medium_pyramidal_implicitfast | 97.917% | 99.9282% | 48/48 | 0.054 / 0.012 / 13.211 | 0.643 | REJECT |
| hard_elliptic_rk4 | 50.000% | 93.9655% | 30/48 | 1.894 / 0.247 / 62.325 | 0.620 | REJECT |

## Decision

At least one untouched law violates a conjunctive frozen gate. Retain the generic mechanism, reject the profile, and do not tune these holdout misses back into R230.
