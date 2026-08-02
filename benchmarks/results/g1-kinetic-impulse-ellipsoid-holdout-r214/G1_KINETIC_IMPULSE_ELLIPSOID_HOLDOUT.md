# Bonesaw G1 kinetic impulse ellipsoid holdout · r214

> Mechanism **PASS** · frozen profile **REJECTED** · authority **NOT ADMITTED** · controller/policy steps **0**.

## Contract

- Rust bounds a zero-centered generalized impulse set `pᵀM⁻¹p ≤ E₂` directly in the kinetic metric. Exact component support is `sqrt(E₂ (M⁻¹)ᵢᵢ)`, avoiding the coordinate box's sum of every absolute coupled inverse-mass column.
- Before either fresh law ran, `E₂` was frozen analytically as `f² m (gΔt)²` with f=0.50. Its pure-translation speed scale is 0.02453 m/s. No R212 residual or quantile determines it.
- The fresh state sequences begin at offsets 10,000 and 20,000. Contact formulations are medium/pyramidal/implicit and hard/pyramidal/Euler, distinct from both R212 laws. Every sample resets; completed contact impulse remains an evaluation-only oracle label.

## Fresh result

| contact law | samples | sample coverage | component coverage | root ω width p95 | root v width p95 | joint width p95 | missed samples | zero alloc | profile |
|---|---|---|---|---|---|---|---|---|---|
| medium_pyramidal_implicit | 48 | 100.000% | 100.0000% | 1.092 | 0.075 | 19.966 | 0 | yes | REJECT |
| hard_pyramidal_euler | 48 | 97.917% | 99.9282% | 1.092 | 0.075 | 19.966 | 1 | yes | REJECT |

The hard-law miss is state index 20,020 at `left_ankle_roll_joint`; its realized 16.164 rad/s residual exceeds the 7.659 rad/s ellipsoid half-width. A different state (20,005) has no within-window contact and remains covered, so the miss is not an empty-label artifact.

## Decision

The kinetic ellipsoid removes the coordinate box's catastrophic width amplification, but the single analytic energy radius does not satisfy strict coverage and useful width together on both fresh laws. The primitive is retained; the profile is rejected without tuning from missed holdout samples. The next construction must separate causal contact phase/load/slip or root and articulated energy budgets.
