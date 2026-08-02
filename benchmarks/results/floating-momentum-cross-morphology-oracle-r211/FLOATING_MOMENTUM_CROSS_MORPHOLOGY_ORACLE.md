# Bonesaw floating momentum cross-morphology oracle · r211

> Pinocchio D1 oracle **PASS** · morphologies **2** · physics steps **0** · policy/controller steps **0** · authority **NOT ADMITTED**.

## Contract

- Rust's generic `ContactTransitionModelSession` now owns the same allocation-free generalized-momentum residual and exact full-inverse-mass interval queries previously exposed only through the Upkie example session.
- Python supplies deterministic state/box corpora. Pinocchio independently assembles each floating mass matrix, reorders its tangent to `[root angular; root linear; joints]`, and computes the exact linear image of every box.
- Root orientation is fixed to identity so world/root tangent conventions coincide exactly. This is a state-local model-product oracle: no contact labels, policy, controller, integration, or physics rollout are used.

## Cross-morphology result

| model | joint / gen dof | states | max abs error | mass cond p95 | box p99 µs | residual p99 µs | zero alloc | gate |
|---|---|---|---|---|---|---|---|---|
| upkie | 6 / 12 | 64 | 1.705e-13 | 3.313e+04 | 8.781 | 6.500 | yes | PASS |
| g1_23dof_mode_10 | 23 / 29 | 64 | 2.984e-13 | 1.995e+05 | 77.813 | 52.119 | yes | PASS |

The absolute D1 threshold is **1.0e-09**. Singleton covectors, signed boxes, and three-candidate residual batches are scored independently on every state.

## Decision

The generic Rust mechanism crosses from wheeled Upkie to the 23-DOF G1 morphology and agrees with an independent Pinocchio mass-matrix oracle. This admits model machinery only. It does not validate R204/R210 residual calibration on G1, a second contact law, terminal consequence, deadline composition, or hardware authority.
