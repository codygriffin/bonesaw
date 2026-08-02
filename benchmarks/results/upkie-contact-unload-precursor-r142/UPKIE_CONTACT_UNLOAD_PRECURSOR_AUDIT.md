# Bonesaw pre-contact-loss load precursor · r142

> Evaluation **FAIL**. This admits a continuous warning signal, not a contact estimator or recovery policy.

## Outcome

The current live r137 controller is replayed over the frozen 20-case matrix. Exact per-wheel plant contact remains the event label; the candidate warning is the positive WBC normal-force share on the wheel that later loses contact. Fixed 10/20/30% thresholds require two consecutive 5 ms samples inside a 500 ms pre-loss window. Green rows are checked for false alerts independently.

| case | outcome | first loss s | wheel | min L/R share | 10% lead s | 20% lead s | 30% lead s | replay |
|---|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | — | — | 0.500/0.500 | — | — | — | YES |
| forward_2n | RECOVERED | 1.110 | left | 0.500/0.500 | — | — | — | YES |
| forward_4n_reference | RECOVERED | 1.200 | left | 0.500/0.500 | — | — | — | YES |
| forward_6n_overload | FALL | 1.090 | left | 0.000/0.335 | — | — | — | YES |
| backward_2n | RECOVERED | 1.075 | left | 0.500/0.500 | — | — | — | YES |
| backward_4n | RECOVERED | 1.115 | left | 0.500/0.500 | — | — | — | YES |
| left_1n | FALL | 1.680 | left | 0.097/0.262 | — | — | — | YES |
| left_2n | FALL | 1.515 | left | 0.000/0.335 | — | — | — | YES |
| left_4n | FALL | 1.035 | right | 0.000/0.000 | — | — | — | YES |
| right_2n | FALL | 1.645 | right | 0.416/0.000 | — | — | — | YES |
| right_4n | FALL | 1.885 | right | 0.049/0.158 | — | — | — | YES |
| diagonal_4n | FALL | 1.115 | left | 0.344/0.000 | — | — | — | YES |
| up_4n | RECOVERED | — | — | 0.500/0.500 | — | — | — | YES |
| down_4n | RECOVERED | — | — | 0.500/0.500 | — | — | — | YES |
| handle_forward_4n | FALL | 1.035 | left | 0.011/0.091 | — | — | — | YES |
| short_8n_50ms | RECOVERED | 1.045 | left | 0.500/0.500 | — | — | — | YES |
| long_2n_200ms | RECOVERED | 1.110 | left | 0.500/0.500 | — | — | — | YES |
| forward_2n_three_pulses | RECOVERED | 1.110 | left | 0.500/0.500 | — | — | — | YES |
| forward_4n_friction_0p1 | RECOVERED | 1.200 | left | 0.498/0.500 | — | — | — | YES |
| forward_4n_friction_0p03 | FALL | 1.165 | left | 0.147/0.186 | — | — | — | YES |

## Threshold discrimination

| threshold | loss rows covered | green false alerts |
|---|---|---|
| 10% | 0 | 0 |
| 20% | 0 | 0 |
| 30% | 0 | 0 |

## Gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| green_rows_retain_double_contact | FAIL |
| failure_contact_loss_discriminates | FAIL |
| twenty_percent_precursor_has_coverage | FAIL |
| twenty_percent_no_green_false_alert | PASS |
| exact_replay | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Boundary

- Normal-load share is model output and remains separate from exact contact observation, debounced mode, WBC admission, command freshness, and plant outcome.
- A threshold crossing may authorize a viability layer to start preparing a support action; it cannot author a contact edge or execute an unadmitted reduced-support solve.
- Hardware requires calibrated force/torque or motor-current evidence and delay/noise/dropout validation.
