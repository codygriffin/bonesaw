# G1 guarded row-Gram pseudoinverse · R291

> Arithmetic candidate **REJECTED** · hard residuals **PASS** · tracking **FAIL** · authority **CLOSED**.

R291 enables the existing `row-gram-task-pseudoinverse-experiment` feature without changing the production default. It is a guarded factorization shortcut for well-conditioned wide task projections; the frozen integrated G1 replay is the required semantic holdout.

## Frozen policy/physics-free replay

| profile | p50 µs | p99 µs | max µs | root RMS m | CoM RMS m | stance-foot RMS m | max root rotation |
|---|---:|---:|---:|---:|---:|---:|---:|
| control | 1796.3 | 5287.4 | 8207.8 | 13.288 | 13.176 | 12.353 | 0.607 |
| row-Gram | 16.3 | 4021.5 | 8031.1 | 22.830 | 22.785 | 21.925 | 1.821 |

Only 50/113 retained raw arrays match; the first divergence is tick 0 (root/q state). The row-Gram path lowers one-run p99 to 4021.5 µs, but root/CoM/foot RMS rises by 71.8%/72.9%/77.5% and maximum root rotation reaches 1.821 rad.

Hard dynamics/contact residuals remain 3.880e-09/1.260e-10, so this is a semantic/tracking rejection rather than a hard-feasibility failure. The candidate remains available only for feature-level kernel tests; R284 remains the production Jacobi path.

## Decision

Do not promote the row-Gram factorization. A faster local pseudoinverse is not an acceptable WBC optimization when it changes the floating trace at tick zero. The next CPU slice must preserve arithmetic/semantic state before timing is considered.
