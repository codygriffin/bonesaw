# G1 alternating multi-step reference

**PASS · 14/14 gates**

This is an offline, policy-free and physics-free reference. Python authors the alternating sequence; Rust plans and analytically samples every support transfer and swing jet. No integrated robot state or simulated contact is used.

- `4` alternating steps, `8` contact edges, `2317` ticks / `11.585 s`.
- Net foot-center progress: `0.217 m`.
- Maximum touchdown speed: `0.000163 m/s`; stance-foot speed: `0.00e+00 m/s`.
- Rust analytic sampling: `0.844 ms` total, `0` allocations / `0` B.
- Complete reference bitwise repeat: `True` (`8e25126780e82847cdec5dd19ed56d8d2db529f765c12fecb7c171011e7ed2e1`).

## Per-step plan

| step | swing | global liftoff | global touchdown | first / second CoP margin | applied landing x | retargeted |
|---:|---:|---:|---:|---:|---:|---:|
| 0 | left | 300 | 529 | 80.00 / 19.60 mm | 0.036 m | False |
| 1 | right | 879 | 1108 | 0.79 / 0.87 mm | 0.098 m | False |
| 2 | left | 1458 | 1687 | 0.79 / 0.87 mm | 0.160 m | False |
| 3 | right | 2037 | 2266 | 0.79 / 0.87 mm | 0.222 m | False |

## Predeclared gates

- PASS `at_least_two_steps`
- PASS `one_liftoff_and_touchdown_per_step`
- PASS `strictly_alternating_swing_feet`
- PASS `touchdown_matches_liftoff_order`
- PASS `no_flight_ticks`
- PASS `stance_foot_velocity_is_zero`
- PASS `touchdown_speed_le_0_02mps`
- PASS `segment_position_boundary_delta_le_0_1mm`
- PASS `segment_velocity_boundary_delta_le_0_02mps`
- PASS `all_rust_segments_repeat_exactly`
- PASS `complete_sequence_repeats_bitwise`
- PASS `rust_hot_sampling_has_zero_allocations`
- PASS `each_transfer_starts_at_zero_com_acceleration`
- PASS `net_forward_progress_ge_0_20m`
