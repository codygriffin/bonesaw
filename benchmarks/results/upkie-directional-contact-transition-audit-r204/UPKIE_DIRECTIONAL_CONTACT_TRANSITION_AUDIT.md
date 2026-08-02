# Bonesaw directional contact-transition audit · r204

> Rust mechanism **PASS** · primary retained/fresh strict coverage **FAIL** · authority **NOT ADMITTED**.

## Contract

- R204 keeps R203's structured acceleration reserve and model-exact normal impulse. Each tangential axis is bounded by the smaller of Coulomb capacity and a passive slip-arrest witness `m_eff v_slip + F_t Δt`.
- The world-frame impulse label is the signed sum across the complete 5 ms plant interval. It is used only for scoring. Exact prospective velocity and MuJoCo contact impulse remain simulator-oracle fields without online sensor authority.
- This is a directional outer box, not a coupled multi-contact Delassus polytope. Coverage and width are reported separately; width reduction alone cannot admit authority.

## Directional reserve sweep

| profile | n | retained velocity | fresh velocity | impulse coverage | impulse util p95 | root ω p95 | root v p95 | joint p95 | joint/base | bound p99 µs |
|---|---|---|---|---|---|---|---|---|---|---|
| coulomb_box_r203_structured | 274 | 100.000% | 100.000% | 100.000% | 4.1% | 29.779 | 2.255 | 1154.415 | 1.000× | 1.085 |
| exact_slip_zero_tangent_load | 274 | 97.368% | 100.000% | 9.854% | — (fails) | 5.082 | 0.655 | 28.423 | 0.025× | 1.271 |
| passive_100mps2_8n | 274 | 98.120% | 100.000% | 100.000% | 5.9% | 9.687 | 0.991 | 58.020 | 0.050× | 1.237 |
| passive_250mps2_8n | 274 | 98.120% | 100.000% | 100.000% | 4.5% | 17.306 | 1.892 | 84.796 | 0.073× | 1.249 |
| passive_100mps2_16n | 274 | 98.120% | 100.000% | 100.000% | 5.0% | 10.198 | 1.031 | 78.421 | 0.068× | 1.259 |

Primary signed world-impulse utilization is **5.9% p95 / 69.2% max** with **100.000%** component coverage and **0.000000 N·s** maximum exceedance.

Primary uncovered retained samples: **backward_4n/drop5/tick 200** (root_wy, 0.098644/s); **backward_4n/drop10/tick 200** (root_wy, 0.099587/s); **backward_4n/drop10/tick 201** (root_wy, 0.132260/s); **backward_4n/drop5_matched/tick 200** (root_wy, 0.100133/s); **handle_forward_4n/drop10/tick 201** (root_vx, 0.004526/s).

## Decision

The directional mechanism may be retained only if it covers the world-impulse and generalized-velocity labels with materially smaller widths. It cannot establish online authority until the witnesses have typed sensor/load provenance, another morphology/contact-law holdout passes, terminal consequence is non-regressing, and the independent 5 ms gate passes.
