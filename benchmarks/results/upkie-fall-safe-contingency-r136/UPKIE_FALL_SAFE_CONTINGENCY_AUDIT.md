# Bonesaw Upkie explicit contingency transition · r136

> Evaluation **PASS**. Live deployment **REJECTED**. This admits a bounded mechanism, not lateral or overload recovery.

## Outcome

The Rust supervisor leaves nominal and the qualified 4 N sagittal recovery byte-exact. Once tilt, planar angular rate, height, or consecutive solver rejection consumes authority, it continuously blends the primary acceleration request into bounded Rust-authored velocity damping. A five-tick startup lease is explicit; stale admitted torque is faded by the twelfth consecutive rejection. Every adverse row still falls, but every declared terminal kinetic-energy and root-rotation witness is lower than baseline.

| case | baseline | candidate | terminal KE J | terminal root ω rad/s | max stale age ticks | baseline exact |
|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | 0.0000 → 0.0000 | 0.000 → 0.000 | 5 → 5 | YES |
| forward_4n | RECOVERED | RECOVERED | 0.0000 → 0.0000 | 0.000 → 0.000 | 5 → 5 | YES |
| left_1n | FALL 2.260s | FALL 2.525s | 2.7147 → 0.5688 | 9.104 → 7.834 | 48 → 9 | NO |
| left_2n | FALL 1.985s | FALL 2.685s | 86.5592 → 75.2227 | 26.213 → 11.792 | 26 → 20 | NO |
| right_2n | FALL 1.990s | FALL 2.020s | 4.1332 → 0.6132 | 12.636 → 4.986 | 20 → 11 | NO |
| forward_6n_overload | FALL 4.615s | FALL 1.495s | 32.9364 → 1.2002 | 15.344 → 0.250 | 41 → 10 | NO |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| green_recovery_preserved | PASS |
| green_semantics_exact | PASS |
| adverse_supervisor_engages | PASS |
| adverse_terminal_energy_reduced | PASS |
| adverse_terminal_root_rotation_reduced | PASS |
| stale_command_age_reduced | PASS |
| candidate_replay_exact | PASS |
| zero_rust_allocation | PASS |
| finite_metrics | PASS |

## Deployment gate

| gate | result |
|---|---|
| no_earlier_adverse_boundary | REJECTED |

## Contract

- The terminal boundary remains root height below 350 mm or tilt above 45°. Earlier entry into a low-energy fall is not recovery and is not scored as one.
- `MaxIterations` candidates remain non-executable. The supervisor uses status only as a continuous lease-pressure input; it never relaxes hard admission.
- Damping targets are written in allocation-free Rust and then pass through the ordinary floating WBC. Python owns MuJoCo, A/B orchestration, kinetic-energy scoring, and artifacts.
- Candidate replay excludes clocks but includes full root/joint/torque/status semantics. Green baseline/candidate exactness includes those same fields.
