# Bonesaw contact-transition interval audit · r201

> Rust mechanism **PASS** · primary impulse coverage **100.000%** · authority **NOT ADMITTED**.

## Contract

- Rust bounds each prospective wheel contact with `J_n ≤ m_eff,max(1+e_max)v_close,max + F_n,max Δt_max`, tangential axes with a friction-cone outer box, and projects the impulse through caller-owned `M⁻¹Jᵀ` into a generalized velocity-jump interval. The hot path is fixed-size and caller-buffered.
- This plant audit evaluates the impulse part first. The prospective bottom-of-wheel distance and velocity are computed before the scored 5 ms interval. Completed-interval MuJoCo impulses are labels only and never enter the bound.
- Effective normal mass is bounded by the full free-floating URDF mass (**5.339 kg**) per contact. The primary profile declares passive restitution `e≤1`, two-body-weight sustained load per contact, and 100 m/s² of unmodeled closing acceleration over 5 ms. It is fixed before case comparison, not learned from a held-out residual.
- Tangential MuJoCo impulse is reported as a magnitude; it is checked against `√2 μJ_n,max`, the magnitude radius of Rust's independent two-axis outer box.

## Named-case result

| bound profile | n | sample coverage | component coverage | max normal miss N·s | max tangent miss N·s | normal bound p95 N·s | utilization p95 | Rust p99 µs |
|---|---|---|---|---|---|---|---|---|
| no_closing_reserve | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 0.703 | 0.180 | 0.235 |
| passive_50mps2 | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 3.373 | 0.031 | 0.180 |
| passive_100mps2 | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 6.042 | 0.017 | 0.147 |
| passive_250mps2 | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 14.051 | 0.007 | 0.150 |
| half_mass_100mps2 | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 3.283 | 0.031 | 0.140 |
| inelastic_100mps2 | 266 | 100.000% | 100.000% | 0.000000 | 0.000000 | 3.283 | 0.031 | 0.140 |

Primary worst sample: **left_1n/drop5_matched/tick 700**; normal/tangential exceedance **0.000000/0.000000 N·s**, prospective distance **0.000756 m**, closing speed **0.014 m/s**.

## Decision

Even 100% impulse coverage would admit only this physical outer-bound mechanism. Authority additionally requires a Rust-model-derived `M⁻¹Jᵀ`, componentwise realized velocity-jump coverage on fresh morphology/friction/contact-timing holdouts, useful interval width, plant consequence non-regression, and the independent 5 ms timing gate. Exact MuJoCo geometry and impulse remain evaluation oracles.
