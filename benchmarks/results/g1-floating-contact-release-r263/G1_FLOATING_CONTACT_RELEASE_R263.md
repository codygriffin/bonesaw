# G1 floating contact-release recovery r263

This report compares the fixed-cap8 r262 trace with the current r263 Rust
session. It is a CPU, policy-free, physics-free artifact comparison: Python
loads retained Rust outputs and does not reconstruct the controller.

## Result

The r263 latch prevents a failed contact from being rebuilt every tick, and
the bounded free-body fallback keeps the state advancing after an unsolved
contact. Functional walking admission remains **closed**.

| Gate | Result |
|---|---:|
| `baseline_exhibits_state_stall` | PASS |
| `candidate_has_no_state_stall` | PASS |
| `candidate_moves_after_contingency` | PASS |
| `candidate_no_failed_or_infeasible` | PASS |
| `candidate_zero_20ms_misses` | PASS |
| `candidate_p99_under_5ms` | PASS |
| `candidate_release_is_bounded` | PASS |

## Runtime/state comparison

| profile | first contingency | longest exact state stall | post-contingency motion | p50 µs | p99 µs | max µs | >5 ms | >20 ms | release ticks |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| r262 cap8 | 300 | 171 | True | 1665.9 | 4027.8 | 16698.0 | 2 | 0 | 0 |
| r263 release cap8 | 300 | 0 | True | 1758.2 | 3974.2 | 5172.9 | 1 | 0 | 1 |

Candidate status counts: `{"0": 199, "1": 101, "4": 299, "5": 1}`.

The fallback is intentionally fail-closed with respect to contact authority:
once a contact solve cannot be admitted, hard contact rows are suppressed
until reset. The state may continue under bounded free-body damping/gravity,
but this is not a claim that the commanded walk is physically realized.
