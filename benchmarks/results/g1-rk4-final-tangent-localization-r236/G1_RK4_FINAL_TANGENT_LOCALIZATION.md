# Bonesaw RK4 final-tangent localization · r236

> SPENT-LABEL DIAGNOSTIC ONLY · construction/selection/authority **INELIGIBLE** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.

R233's original scorer projected the model's final impulse through the initial mass response. That is invalid once Rust evolves state, geometry, dynamics, and response. R233 now scores the returned final generalized tangent directly; the old proxy remains named diagnostic evidence only.

| law | exact sets | direct width · ang / lin / joint | joint width on exact sets | normal-impulse RMSE L / R | pitch-moment RMSE L / R |
|---|---|---|---|---|---|
| medium_elliptic_rk4 | 47/48 | 0.285 / 0.043 / 10.446 | 10.446 | 0.195 / 0.730 | 0.0141 / 0.0311 |
| hard_pyramidal_rk4 | 44/48 | 0.699 / 0.110 / 11.248 | 10.618 | 0.747 / 1.957 | 0.0344 / 0.0802 |

## Worst joint rows

- `medium_elliptic_rk4`: s22 left_ankle_roll_joint +5.223 rad/s (exact), s7 right_ankle_roll_joint -4.422 rad/s (exact), s23 left_ankle_roll_joint +4.316 rad/s (exact), s36 left_ankle_pitch_joint -2.355 rad/s (exact).
- `hard_pyramidal_rk4`: s1 right_ankle_pitch_joint -5.624 rad/s (set mismatch), s3 right_ankle_roll_joint +5.309 rad/s (exact), s14 right_ankle_roll_joint -3.491 rad/s (set mismatch), s4 right_ankle_roll_joint +2.979 rad/s (exact).

## Localization

Medium's worst 10.446 rad/s joint width occurs entirely inside rows whose contact sets are already exact; hard still has both exact-set and set-mismatch tails. The largest coordinates are ankle pitch/roll. The official fixture already uses the same four 5 mm sphere contacts per foot, so primitive count/radius is not the missing mechanism. Remaining work is stage-local force and within-foot wrench distribution plus reference constraint-solver semantics—not another scalar activation mask.

No coefficient, sweep count, residual box, selector, or command authority is changed from this spent-label diagnostic.

An independent retained rerun reproduces all six NPZ arrays exactly.
