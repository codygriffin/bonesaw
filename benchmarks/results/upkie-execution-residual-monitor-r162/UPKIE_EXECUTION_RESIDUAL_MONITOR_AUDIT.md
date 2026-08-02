# Bonesaw causal execution-residual monitor · r162

> Mechanism gate **PASS** · command authority **NOT PROMOTED**.

## Result

Static forecast-error cells failed on r160 and r161. This replacement is a fixed-capacity Rust monitor: a 5 ms prediction is authored from the final exact-WBC acceleration, checked against the envelope that existed before the observation arrived, and only then admitted into a 32-sample rolling window. Exact support changes and missing evidence reset warmup.

The fresh third holdout contains **7,532** comparable ticks: **6,567** covered ticks and **70** causal exceedances. **69** exceedances saw a later covered tick within 250 ms. Worst observed error/bound was **3961.077×**; the maximum Rust transition time was **6,833 ns**, with **0 allocation calls / 0 bytes** in the measured transition.

## Third holdout

| case | comparable | covered | exceeded | recovered ≤250ms | worst ratio | p99 ns | replay |
|---|---|---|---|---|---|---|---|
| r162_forward_0p75n | 772 | 618 | 7 | 7 | 2754.020 | 631 | YES |
| r162_forward_2n | 700 | 645 | 4 | 4 | 3961.077 | 652 | YES |
| r162_forward_4p25n | 330 | 251 | 5 | 5 | 965.778 | 741 | YES |
| r162_backward_2p25n | 808 | 712 | 6 | 6 | 711.885 | 771 | YES |
| r162_backward_4p25n | 244 | 196 | 1 | 1 | 504.145 | 641 | YES |
| r162_left_2n | 486 | 404 | 10 | 10 | 583.254 | 839 | YES |
| r162_right_4n | 399 | 328 | 6 | 6 | 430.666 | 651 | YES |
| r162_diagonal_3p75n | 512 | 427 | 5 | 4 | 1141.384 | 615 | YES |
| r162_up_2p5n | 708 | 593 | 8 | 8 | 1423.004 | 641 | YES |
| r162_down_2p5n | 674 | 624 | 6 | 6 | 457.064 | 642 | YES |
| r162_forward_2p5n_200ms | 359 | 293 | 4 | 4 | 3495.739 | 719 | YES |
| r162_handle_backward_2p5n | 1193 | 1166 | 3 | 3 | 366.866 | 642 | YES |
| r162_right_2n_two_pulses | 347 | 310 | 5 | 5 | 142.415 | 571 | YES |

## Gates

| gate | result |
|---|---|
| third_holdout_complete | PASS |
| third_holdout_disjoint_from_r161 | PASS |
| third_holdout_replay_exact | PASS |
| causal_preupdate_monitor_exercised | PASS |
| disturbance_exceedance_exercised | PASS |
| recovery_after_exceedance_exercised | PASS |
| rust_transition_zero_allocation | PASS |
| bounded_monitor_latency_under_100us | PASS |

## Authority boundary

This signal may lower forecast confidence or color a UI health bar. It cannot admit a candidate, alter torque, or turn a previously rejected request into an executable command. A rolling residual envelope is empirical observability, not a reachability proof; promotion still requires a separate causal plant gate.
