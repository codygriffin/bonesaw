# Bonesaw request-gated viability planner plant A/B · r150

> Evaluation **PASS**. Live deployment **REJECTED**. This composes the r147 selector and r148 supervisor without allowing an inactive planner to replace r137 double-support execution.

## Outcome

Measured reduced support is used only when the physical roll/lateral capture pressure has crossed activation and the Rust supervisor has a fresh executable request. Otherwise the admitted r137 double-support program remains unchanged. This isolates whether request-gated support switching repairs r149's green regressions while retaining any adverse recovery.

Green regressions: **none**. Earlier adverse boundaries: **['forward_6n_overload', 'left_2n', 'left_4n', 'right_2n', 'right_4n', 'diagonal_4n', 'handle_forward_4n', 'forward_4n_friction_0p03']**. Newly recovered adverse rows: **none**.

| case | r137 | candidate | Δ s | active | reduced | queries | p99 µs | max µs | replay |
|---|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 270.3 | 293.5 | YES |
| forward_2n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 270.8 | 524.0 | YES |
| forward_4n_reference | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 277.9 | 330.3 | YES |
| forward_6n_overload | FALL 4.820s | FALL 4.690s | -0.130 | 102 | 68 | 1249 | 5187.5 | 76963.2 | YES |
| backward_2n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 281.3 | 303.9 | YES |
| backward_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 273.4 | 290.6 | YES |
| left_1n | FALL 2.470s | FALL 2.560s | +0.090 | 160 | 117 | 1688 | 5346.3 | 8317.0 | YES |
| left_2n | FALL 2.350s | FALL 1.850s | -0.500 | 38 | 30 | 483 | 9577.8 | 16728.1 | YES |
| left_4n | FALL 2.610s | FALL 1.545s | -1.065 | 69 | 69 | 780 | 16239.3 | 16679.9 | YES |
| right_2n | FALL 2.015s | FALL 1.625s | -0.390 | 117 | 103 | 1252 | 5130.9 | 179580.9 | YES |
| right_4n | FALL 2.415s | FALL 1.550s | -0.865 | 74 | 45 | 819 | 5357.7 | 8192.8 | YES |
| diagonal_4n | FALL 2.125s | FALL 1.930s | -0.195 | 78 | 57 | 877 | 5002.7 | 5883.0 | YES |
| up_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 276.2 | 289.1 | YES |
| down_4n | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 277.2 | 304.8 | YES |
| handle_forward_4n | FALL 4.630s | FALL 4.490s | -0.140 | 194 | 165 | 2136 | 5387.7 | 6087.0 | YES |
| short_8n_50ms | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 280.3 | 305.1 | YES |
| long_2n_200ms | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 276.1 | 472.2 | YES |
| forward_2n_three_pulses | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 272.9 | 293.3 | YES |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | +0.000 | 0 | 0 | 300 | 276.8 | 295.3 | YES |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 1.585s | -0.025 | 57 | 53 | 665 | 4638.2 | 5157.0 | YES |

## Gates

| gate | result |
|---|---|
| complete_frozen_matrix | PASS |
| current_live_reproduced | PASS |
| every_green_qualification_preserved | PASS |
| no_new_or_earlier_adverse_boundary | FAIL |
| at_least_one_new_adverse_recovery | FAIL |
| candidate_exact_replay | PASS |
| zero_timed_rust_allocation | PASS |
| finite_and_numeric_clean | PASS |
| candidate_p99_inside_5ms | FAIL |

## Boundary

- Preserving green behavior is necessary but not sufficient: every existing adverse boundary must also be no earlier, and at least one previously falling row must recover.
- The 250 ms local score remains a proposal heuristic. Rust owns request lifetime and final WBC admission; MuJoCo remains external consequence evidence.
