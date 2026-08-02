# Bonesaw contact-impulse residual source audit · r197

> Instrumentation **PASS** · authority **NOT ADMITTED**.

## Boundary

- Python records MuJoCo wheel normal/tangential impulse and generalized constraint impulse over each complete 5 ms plant interval. These witnesses are offline only and never enter the Rust controller, candidate selection, or replay state.
- The audit compares realized roll/pitch plus six-joint acceleration with (a) the hypothesis matching measured physical support, (b) the best of all four discrete support hypotheses, and (c) the componentwise min/max qdd envelope before terminal scoring.
- A correlation is descriptive source localization, not a calibrated bound, causal proof, conformal certificate, or hardware claim.
- R196 already rejects smooth current-state residual conditioning under leave-one-case-out evaluation. R197 asks which completed-interval physical witness co-varies with that rejected tail; it does not refit the same model under another name.

## Aggregate result

- Samples: **266**. Measured physical support is the minimum-error discrete hypothesis on **91.4%**.
- Four-support qdd envelope coverage: **0.0%**; post-score pressure coverage: **8.6%**.
- Best-support qdd error p95/max: **223.257/14718.472**. Qdd envelope exceedance p95/max: **190.325/14705.576**.
- Constraint-impulse correlation with best-support qdd error / pressure exceedance: **+0.445/+0.399**.
- Normal/tangential wheel-impulse correlation with best-support qdd error: **+0.432/+0.907**. The latter is the strongest measured source association, but it uses the completed interval and is therefore not a causal online feature.

| case | profile | n | physical is best | qdd covered | pressure covered | best qdd err max | constraint impulse max |
|---|---|---|---|---|---|---|---|
| nominal | drop5 | 23 | 100.0% | 0.0% | 0.0% | 88.201 | 0.143 |
| nominal | drop10 | 22 | 100.0% | 0.0% | 0.0% | 88.345 | 0.143 |
| nominal | drop5_matched | 11 | 100.0% | 0.0% | 0.0% | 88.225 | 0.143 |
| forward_4n_reference | drop5 | 23 | 100.0% | 0.0% | 13.0% | 112.048 | 0.199 |
| forward_4n_reference | drop10 | 22 | 100.0% | 0.0% | 4.5% | 115.005 | 0.207 |
| forward_4n_reference | drop5_matched | 11 | 100.0% | 0.0% | 9.1% | 112.399 | 0.188 |
| backward_4n | drop5 | 23 | 100.0% | 0.0% | 4.3% | 103.154 | 0.178 |
| backward_4n | drop10 | 22 | 100.0% | 0.0% | 4.5% | 105.096 | 0.153 |
| backward_4n | drop5_matched | 11 | 90.9% | 0.0% | 9.1% | 170.418 | 0.196 |
| left_1n | drop5 | 7 | 71.4% | 0.0% | 14.3% | 163.074 | 0.143 |
| left_1n | drop10 | 10 | 80.0% | 0.0% | 20.0% | 1718.362 | 0.143 |
| left_1n | drop5_matched | 9 | 55.6% | 0.0% | 11.1% | 14718.472 | 0.521 |
| right_1n_mirror | drop5 | 8 | 87.5% | 0.0% | 0.0% | 734.321 | 1.065 |
| right_1n_mirror | drop10 | 8 | 75.0% | 0.0% | 12.5% | 88.345 | 0.143 |
| right_1n_mirror | drop5_matched | 5 | 80.0% | 0.0% | 0.0% | 1384.304 | 0.143 |
| handle_forward_4n | drop5 | 13 | 84.6% | 0.0% | 38.5% | 736.824 | 0.298 |
| handle_forward_4n | drop10 | 22 | 77.3% | 0.0% | 18.2% | 14420.769 | 0.453 |
| handle_forward_4n | drop5_matched | 5 | 80.0% | 0.0% | 20.0% | 960.308 | 0.182 |
| forward_4n_friction_0p03 | drop5 | 6 | 83.3% | 0.0% | 0.0% | 224.519 | 0.246 |
| forward_4n_friction_0p03 | drop10 | 3 | 100.0% | 0.0% | 0.0% | 152.622 | 0.142 |
| forward_4n_friction_0p03 | drop5_matched | 2 | 50.0% | 0.0% | 0.0% | 219.471 | 0.162 |

## Decision

The four support masks are retained as visible model hypotheses, but the best mask is still tested against a nonzero model-to-plant residual. R197 does not install an empirical margin or alter authority. The next admissible model must condition and validate a causal pre-step witness on fresh held-out cases, or replace acceleration prediction with a bounded momentum/contact-impulse transition.
