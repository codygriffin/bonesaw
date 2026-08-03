# Upkie kinematic measured-contact replay · R299

> PASS · actual MuJoCo collision masks · zero policy · zero dynamics integration · authority closed.

R299 authors only root poses. Each of 50 nominal 250 Hz frames runs `mj_forward` and the production wheel-subtree/ground contact extractor. The newest observation in each five-frame window enters the existing persistent Rust adapter and WBC at 50 Hz. `mj_step` is patched to fail, so no plant trajectory or physics consequence is claimed.

| tick | phase | roll rad | z offset m | measured | debounced | hard | WBC status | residual | step µs |
|---:|---|---:|---:|:---:|:---:|:---:|---:|---:|---:|
| 0 | double | 0.000 | 0.000 | `11` | `00` | `00` | 1 | 2.501e-14 | 124.6 |
| 1 | double | 0.000 | 0.000 | `11` | `00` | `00` | 1 | 2.833e-14 | 107.9 |
| 2 | double | 0.000 | 0.000 | `11` | `11` | `11` | 0 | 1.343e-12 | 135.5 |
| 3 | left_only | -0.080 | 0.012 | `10` | `11` | `10` | 1 | 1.672e-12 | 123.0 |
| 4 | left_only | -0.080 | 0.012 | `10` | `10` | `10` | 1 | 1.672e-12 | 121.7 |
| 5 | right_only | 0.080 | 0.012 | `01` | `10` | `00` | 1 | 9.932e-14 | 112.6 |
| 6 | right_only | 0.080 | 0.012 | `01` | `00` | `00` | 1 | 9.932e-14 | 112.4 |
| 7 | right_only | 0.080 | 0.012 | `01` | `01` | `01` | 1 | 4.207e-12 | 129.3 |
| 8 | flight | 0.000 | 0.500 | `00` | `01` | `00` | 1 | 3.680e-14 | 114.1 |
| 9 | flight | 0.000 | 0.500 | `00` | `00` | `00` | 1 | 3.680e-14 | 119.0 |

## Gates

| gate | result |
|---|:---:|
| actual_mujoco_contact_extractor | PASS |
| all_contact_patterns_measured | PASS |
| fifty_measured_250hz_frames | PASS |
| newest_frame_enters_wbc | PASS |
| edges_exist_inside_windows | PASS |
| activation_debounce | PASS |
| deactivation_debounce | PASS |
| hard_rows_are_raw_subset | PASS |
| flight_fails_closed | PASS |
| accepted_exact_observations | PASS |
| wbc_outputs_finite | PASS |
| rust_hot_loop_zero_allocations | PASS |
| zero_policy_zero_physics_integration | PASS |
| exact_semantic_replay | PASS |

This admits the collision-extraction → observation → debounce → hard-row seam for a deterministic kinematic fixture. It does not admit a physically realized transfer, tracking performance, contact-force accuracy, hardware sensing, or command authority.
