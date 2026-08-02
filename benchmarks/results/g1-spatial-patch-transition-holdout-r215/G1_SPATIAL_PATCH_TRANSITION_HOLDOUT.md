# Bonesaw G1 spatial-patch transition holdout · r215

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- Rust replaces four independent point boxes per foot with one resultant spatial wrench. Tangential impulse is passive-slip/Coulomb limited; CoP moments and force share the same nonnegative normal impulse. The exact support of this finite-patch box-pyramid is evaluated per generalized coordinate without vertex allocation.
- Before either fresh law ran, the profile was frozen at the r212 50/10/50 generalized acceleration reserve, 100 m/s² and 8 N tangential witnesses, a 10 m/s² normal speed reserve, one whole-body weight of sustained normal load, authored 85×30 mm sole half-extents, restitution upper 1, and zero unmodeled torsional friction.
- Fresh state sequences begin at offsets 30,000 and 40,000. Compliant/elliptic/RK4 and rigid/elliptic/implicit laws differ from r213 and r214. Every sample resets; completed state is scoring-only.

## Fresh result

| contact law | samples | sample coverage | component coverage | root ω width p95 | root v width p95 | joint width p95 | normal upper p95 N·s | bound p99 µs | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|---|
| compliant_elliptic_rk4 | 48 | 100.000% | 100.0000% | 3.529 | 0.655 | 878.035 | 4.192 | 3.410 | yes | REJECT |
| rigid_elliptic_implicit | 48 | 83.333% | 98.9943% | 3.180 | 0.634 | 923.910 | 4.329 | 2.874 | yes | REJECT |

## Decision

Finite-patch coupling is retained as model machinery but the frozen profile is rejected. It removes the eight-point Cartesian product yet full corner-of-patch moment uncertainty still amplifies through small ankle inertias. No holdout quantile is fed back. A useful construction needs a causal center contact prediction plus a state-conditioned kinetic residual, rather than the full patch reachable set as one tick-wide command authority.
