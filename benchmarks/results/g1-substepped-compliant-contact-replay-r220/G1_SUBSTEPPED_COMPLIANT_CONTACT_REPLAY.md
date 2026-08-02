# Bonesaw G1 substepped compliant-contact replay · r220

> Mechanism **PASS** · frozen construction **PASS** · profile/authority **NOT PROMOTED** · replay physics/policy/controller/integration **0 / 0 / 0 / 0**.

## Contract

- Rust carries signed point gap and contact velocity through fixed microsteps. Each microstep predicts penetration, applies explicit Kelvin–Voigt normal impulse, projects tangent impulse to the current circular Coulomb disk, updates all points through the full Delassus operator, and advances gap with post-impulse normal velocity. It is a deterministic model witness, not a continuous-time enclosure.
- The model-owned mapping is `k = s_k z m_eff / τ²`, `c = s_c 2 sqrt(z) m_eff / τ`, using causal state-local effective mass and declared contact relaxation/impedance. A 16× impulse-cap scale and every parameter/sweep row were selected with R218 construction labels; none is hardware calibration or untouched evidence.
- The fitted groupwise residual box uses every R218 label and is eligible only to be frozen for a new-law holdout. Strict construction coverage, 2.0/0.5/10.0 width, deterministic replay, and zero allocation are conjunctive. No construction result is authority.

## Construction sweep

| profile | microsteps | k/c scale | predictor residual p95 ω/v/joint | strict fitted width ω/v/joint | impulse error mean N·s | query p99 µs | gate |
|---|---|---|---|---|---|---|---|
| substeps8 | 8 | 1.00 / 1.00 | 0.551 / 0.084 / 288.473 | 23.095 / 3.661 / 2062.166 | 1.166 | 4.822 | reject |
| substeps16 | 16 | 1.00 / 1.00 | 0.208 / 0.040 / 17.305 | 1.336 / 0.259 / 151.858 | 0.588 | 10.652 | reject |
| substeps32 | 32 | 1.00 / 1.00 | 0.229 / 0.038 / 8.128 | 1.723 / 0.281 / 38.640 | 0.513 | 19.445 | reject |
| substeps64 | 64 | 1.00 / 1.00 | 0.228 / 0.035 / 4.120 | 1.736 / 0.300 / 19.182 | 0.493 | 29.470 | reject |
| substeps96 | 96 | 1.00 / 1.00 | 0.224 / 0.034 / 3.860 | 1.772 / 0.299 / 13.341 | 0.489 | 58.130 | reject |
| substeps128 | 128 | 1.00 / 1.00 | 0.225 / 0.033 / 3.424 | 1.748 / 0.299 / 9.802 | 0.487 | 55.705 | PASS |
| substeps192 | 192 | 1.00 / 1.00 | 0.224 / 0.033 / 3.565 | 1.764 / 0.299 / 9.984 | 0.486 | 78.681 | PASS |
| substeps256 | 256 | 1.00 / 1.00 | 0.223 / 0.033 / 3.601 | 1.764 / 0.299 / 10.206 | 0.484 | 99.619 | reject |
| soft_k05_c075 | 128 | 0.50 / 0.75 | 0.276 / 0.045 / 3.732 | 2.338 / 0.404 / 11.999 | 0.525 | 49.662 | reject |
| stiff_k2_c15 | 128 | 2.00 / 1.50 | 0.108 / 0.027 / 3.807 | 1.103 / 0.178 / 24.271 | 0.456 | 57.508 | reject |
| stiff_k4_c2 | 128 | 4.00 / 2.00 | 0.086 / 0.017 / 5.135 | 0.608 / 0.100 / 40.069 | 0.439 | 53.489 | reject |
| under_damped | 128 | 1.00 / 0.50 | 0.225 / 0.033 / 4.509 | 2.069 / 0.359 / 12.359 | 0.516 | 49.600 | reject |
| over_damped | 128 | 1.00 / 2.00 | 0.160 / 0.035 / 3.170 | 1.331 / 0.217 / 25.199 | 0.462 | 49.686 | reject |

## Decision

Freeze `substeps128` unchanged for new contact laws and state offsets. Its all-label group box is 1.748/0.299/9.802, inside every construction gate, while the compliant query is 55.705 µs p99 with zero Rust allocation. Promotion remains false until an untouched holdout preserves strict coverage, width, repeat, and deadline together.
