# Bonesaw contact-response localization audit · r205

> Mechanism **PASS** · prospective-point strict retained coverage **FAIL** · oracle-centroid strict retained coverage **FAIL** · authority **NOT ADMITTED**.

## Contract

- This is a label-only decomposition, not a reachable tube. Every row receives the completed signed world impulse; the oracle row additionally receives its impulse-weighted contact centroid. Neither field is available to online authority.
- All variants retain the same four support-hypothesis accelerations and R203 structured acceleration reserve. The comparison isolates whether R204's remaining velocity misses arise from omitting impulse, evaluating `M⁻¹Jᵀ` at the causal prospective point, or residual generalized momentum beyond both.
- The contact centroid is weighted by normal impulse across all MuJoCo substeps in the 5 ms interval. A no-contact wheel falls back to its prospective point and contributes zero impulse.

## Exact-impulse response localization

| variant | n | retained | fresh | max miss /s | root ω p95 | root v p95 | joint p95 | response p99 µs |
|---|---|---|---|---|---|---|---|---|
| support_acceleration_only | 274 | 10.526% | 62.500% | 73.277879 | 0.227 | 0.069 | 1.383 | 0.000 |
| prospective_point_exact_impulse | 274 | 83.835% | 62.500% | 2.598222 | 0.227 | 0.069 | 1.383 | 8.178 |
| oracle_contact_centroid_exact_impulse | 274 | 81.579% | 62.500% | 5.317296 | 0.227 | 0.069 | 1.383 | 7.766 |

Impulse-weighted centroid availability is **90.9%** of wheel/sample pairs; prospective-to-centroid error is **20.001 mm p95 / 21.702 mm max** when available.

Exact impulse at the prospective point improves retained coverage from **10.526%** to **83.835%**. Replacing that point with the completed normal-impulse centroid lowers coverage to **81.579%** and raises maximum miss from **2.598222/s** to **5.317296/s**.

## Decision

The oracle normal-impulse centroid does not close the misses and is worse than the causal prospective point. A single relocated force point therefore cannot represent the distributed wheel contact. The next contact layer must retain spatial impulse moment/contact distribution, while root/generalized-momentum residual remains a separate calibrated layer. A tighter force-only polytope cannot establish authority by itself, and label-only fields cannot enter command authority.
