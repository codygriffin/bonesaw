# Upkie continuous landing-request envelope — R313

Status: **QUALIFIED DEFAULT-OFF MECHANISM**; public promotion **REJECTED**.

R313 keeps R312's measured-contact phase boundary and adds a Rust-owned, allocation-free continuous authority envelope. Authority attenuates smoothly as root tilt, horizontal speed, or height approaches the limits `[0.2, 2.0, 0.34, 0.08]`. No reset or solver-budget change is allowed.

| force Y (N) | R310 baseline terminal | R312 terminal | R313 terminal | Δ vs baseline | R313 active ticks |
|---:|---:|---:|---:|---:|---:|
| -8 | 74 | 78 | 79 | +5 | 6 |
| -6 | 164 | 83 | 82 | -82 | 7 |
| -4 | — | — | — | — | 0 |
| -2 | — | — | — | — | 0 |
| +2 | — | — | — | — | 0 |
| +4 | — | — | — | — | 0 |
| +6 | 87 | 82 | 82 | -5 | 6 |
| +8 | 69 | 69 | 68 | -1 | 3 |

## Mechanism gates

- PASS `complete_r311_force_holdout`
- PASS `causal_measured_window`
- PASS `mode_firewall`
- PASS `finite_outputs`
- PASS `zero_landing_allocations`
- PASS `landing_deadline`
- PASS `controller_and_worker_deadlines`
- PASS `candidate_replay_exact`

## Physical promotion gates

- OPEN `candidate_never_moves_terminal_boundary_earlier`
- OPEN `candidate_reduces_terminal_fall_count`
- OPEN `candidate_completes_every_case`

## Conclusion

R313 supplies a Rust-owned continuous landing-request envelope, but the measured holdout remains negative: attenuation does not remove the terminal cases and still moves at least one boundary earlier. The envelope remains default-off safety/research evidence.
