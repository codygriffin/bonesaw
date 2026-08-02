# Bonesaw pyramid-edge fresh holdout · r244

> Mechanism **FAIL** · frozen profile **REJECTED** · authority **NOT ADMITTED**.

Two new pyramidal laws and disjoint offsets 190,000/200,000 were generated after R243 froze 64 sweeps. Both implicitfast and RK4 rows use model-only pyramid-edge cone ABI 2; RK4 separately uses current-stage integrator ABI 4.

| law | integrator / cone ABI | coverage | exact active | pred-only / missed | width · ang / lin / joint | p99 ms | decision |
|---|---|---|---|---|---|---|---|
| balanced_pyramidal_implicitfast_edge_r244 | 0 / 2 | 95.833% | 47/48 | 1 / 0 | 0.220 / 0.048 / 16.030 | 1.267 | REJECT |
| hard_pyramidal_rk4_edge_r244 | 4 / 2 | 100.000% | 45/48 | 2 / 1 | 0.128 / 0.007 / 6.279 | 5.549 | REJECT |

Prediction received causal arrays only; scoring compares returned and reference final generalized tangents. Promotion requires strict coverage, deadline, repeat, and zero allocation for both rows. Authority remains a separate disabled boundary.
