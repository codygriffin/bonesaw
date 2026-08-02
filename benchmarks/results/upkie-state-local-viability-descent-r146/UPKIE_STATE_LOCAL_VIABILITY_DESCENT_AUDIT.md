# Bonesaw frozen-state local viability descent · r146

> Evaluation **PASS**. The source states come from retained MuJoCo boundaries, but this audit performs no policy step, external plant step, contact integration, or clock read. It makes 245 fixed WBC queries per state and applies one declared constant-acceleration forecast.

## Outcome

For each of 16 frozen states, the score is the larger of normalized roll-capture and lateral-capture magnitude after 250 ms. `zero` is the WBC response to a zero root request; `best` is the minimum over the fixed roll/lateral/yaw request lattice under the exact measured support mask. A ratio below one proves local forecast descent exists at that state. It does not prove that a causal controller can select, sustain, or realize the action through changing contact.

Descent exists in 15/16 rows, but only 9/16 best forecasts finish inside the declared unit boundary and 15/16 select at least one request-lattice edge. The evidence therefore rejects a static-gain interpretation: the next candidate needs bounded finite-horizon response estimation, regularization/slew, an exact verification solve, and the external plant gate.

| case | before fall s | L/R | now | zero | best | best/zero | request roll/lat/yaw | status |
|---|---|---|---|---|---|---|---|---|
| left_1n | 0.50 | 10 | 0.120 | 3.200 | 0.129 | 0.040 | +0/+250/+80 | SolvedWithSlack |
| left_1n | 0.25 | 00 | 0.513 | 5.761 | 0.635 | 0.110 | +250/-250/+80 | SolvedWithSlack |
| left_1n | 0.10 | 00 | 0.303 | 2.902 | 0.624 | 0.215 | +0/+0/+80 | SolvedWithSlack |
| left_1n | 0.05 | 00 | 0.834 | 2.830 | 0.523 | 0.185 | -250/+0/-20 | SolvedWithSlack |
| left_2n | 0.50 | 11 | 0.003 | 0.006 | 0.003 | 0.445 | +0/+40/-80 | SolvedWithSlack |
| left_2n | 0.25 | 10 | 0.984 | 11.308 | 6.718 | 0.594 | +0/+250/-80 | SolvedWithSlack |
| left_2n | 0.10 | 00 | 1.310 | 1.505 | 1.098 | 0.730 | +0/+0/-80 | SolvedWithSlack |
| left_2n | 0.05 | 00 | 3.896 | 29.369 | 20.331 | 0.692 | +100/+100/-80 | SolvedWithSlack |
| right_2n | 0.50 | 11 | 0.001 | 0.001 | 0.001 | 0.834 | -40/-40/+80 | SolvedWithSlack |
| right_2n | 0.25 | 11 | 0.002 | 0.002 | 0.002 | 1.000 | +0/+0/+0 | Solved |
| right_2n | 0.10 | 00 | 0.286 | 2.607 | 1.609 | 0.617 | -250/+100/+80 | SolvedWithSlack |
| right_2n | 0.05 | 00 | 1.059 | 2.677 | 0.105 | 0.039 | -250/+40/+80 | SolvedWithSlack |
| forward_6n_overload | 0.50 | 11 | 0.007 | 0.016 | 0.001 | 0.038 | -100/-250/+0 | SolvedWithSlack |
| forward_6n_overload | 0.25 | 10 | 0.017 | 4.236 | 2.183 | 0.515 | +0/+40/+80 | SolvedWithSlack |
| forward_6n_overload | 0.10 | 00 | 1.248 | 3.601 | 1.331 | 0.370 | -250/-100/+0 | SolvedWithSlack |
| forward_6n_overload | 0.05 | 00 | 1.945 | 13.085 | 7.436 | 0.568 | -250/+0/-80 | SolvedWithSlack |

## Support summary

| support | rows | descent rows | ≤0.5 rows | median best/zero | worst best/zero |
|---|---|---|---|---|---|
| double | 4 | 3 | 2 | 0.639 | 1.000 |
| single | 3 | 3 | 1 | 0.515 | 0.594 |
| zero | 9 | 9 | 5 | 0.370 | 0.730 |

## Integrity gates

| gate | result |
|---|---|
| matrix_complete | PASS |
| double_single_zero_support_covered | PASS |
| every_snapshot_has_executable_candidate | PASS |
| exact_replay | PASS |
| zero_rust_allocation | PASS |
| finite | PASS |

## Boundary

- The score is a declared local forecast, not a viability certificate, fall probability, or aggregate authority verdict.
- Root-origin lateral acceleration approximates CoM capture evolution over 250 ms; a promoted controller must replace this with an exact predicted state/contact program and then pass the external plant matrix.
- Zero- and single-support descent may come from internal angular-momentum exchange and cannot create ground impulse. Contact return and command realization remain separate admissions.
