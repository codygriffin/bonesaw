# Bonesaw generalized RK4 convergence audit · r232

> Prediction-only profile **FROZEN** · authority **NOT ADMITTED**.

The selection pass can reach only causal state/geometry inputs. Completed R231 impulses and residuals are reopened after the sweep count is frozen, so the spent diagnostic cannot tune the profile.

| sweeps | 2× refinement width · ang / lin / joint | % gate | p99 ms | decision |
|---|---|---|---|---|
| 1 | 0.9010 / 0.1466 / 5.8381 | 58.381% | 3.915 | — |
| 2 | 0.7140 / 0.1165 / 5.1282 | 51.282% | 3.935 | — |
| 4 | 0.4345 / 0.0672 / 2.7385 | 27.385% | 3.455 | — |
| 8 | 0.3451 / 0.0491 / 2.6982 | 26.982% | 3.980 | — |
| 16 | 0.1137 / 0.0193 / 0.9407 | 9.407% | 3.116 | — |
| 32 | 0.0225 / 0.0038 / 0.2496 | 2.496% | 3.184 | — |
| 64 | 0.0025 / 0.0007 / 0.0600 | 0.600% | 4.344 | FREEZE |
| 128 | — | — | 3.954 | — |

The smallest stable profile is **64 projected sweeps**.

The global step is now classical four-stage generalized RK4: each stage refreshes support geometry, floating inverse dynamics, the mass factor, complete Delassus response, point motion, and collision membership. The local explicit contact solve evaluates that stage's right-hand side; weighted stage impulses and accelerations advance the final tangent and pose.
The PyO3 model-coupled ABI keeps ids 0/1/2 stable for explicit, implicit, and exponential-trapezoidal; generalized RK4 is id 3. Scalar id 2 therefore remains the legacy exponential-trapezoidal scheme.

The R231 label diagnostic remains evidence only. A separate untouched law/state-offset corpus is required before any promotion decision.

The spent-label tracking diagnostic compares Rust's returned evolved tangent directly with the reference final tangent. The legacy impulse-through-initial-response proxy is retained under an explicit name and cannot score a state-evolving integrator.

An independent retained rerun reproduces all 59 non-timing NPZ arrays exactly. Timing arrays remain measured evidence and are excluded from semantic equality.
