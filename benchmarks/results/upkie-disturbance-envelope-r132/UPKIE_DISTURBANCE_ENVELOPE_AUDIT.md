# Upkie multidimensional disturbance envelope · upkie-disturbance-envelope-r132

**Evaluation admission: PASS.** This report freezes a discriminating plant-consequence matrix; it does not require every controller row to recover. Python owns MuJoCo integration, case construction, external wrench application, friction variation, and scoring. Persistent Rust sessions own rooted capture/station state, floating WBC, contact/rolling equations, hierarchy, torque, and allocation counters.

The matrix separates force axis/sign, impulse duration, application body, and plant friction. The live controller remains intentionally sagittal: red lateral or low-friction rows are measured capability boundaries, not evaluator failures. MuJoCo solver warnings are retained and promoted to NUMERIC_FAULT rather than silently reported as an ordinary fall.

## Case matrix

| case | family | force xyz N | impulse N·s | friction | outcome | qualified | qualification blockers | peak Δp mm | peak tilt ° | recovery s | torque use | warnings | peak qacc abs | Rust p99 µs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | nominal | 0/0/0 | 0.000 | 1.00 | RECOVERED | True | — | 0.1 | 0.05 | 0.000 | 0.067 | 0 | 1.80e+00 | 141.6 |
| forward_2n | axis_x | 2/0/0 | 0.200 | 1.00 | RECOVERED | True | — | 52.0 | 12.53 | 1.305 | 0.069 | 0 | 2.40e+02 | 151.2 |
| forward_4n_reference | axis_x | 4/0/0 | 0.400 | 1.00 | RECOVERED | True | — | 107.6 | 24.91 | 1.950 | 0.084 | 0 | 5.46e+02 | 152.3 |
| forward_6n_overload | axis_x | 6/0/0 | 0.600 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×86; loop overrun×98 | 351.0 | 92.87 | — | 1.000 | 0 | 5.77e+04 | 9365.0 |
| backward_2n | axis_x | -2/0/0 | 0.200 | 1.00 | RECOVERED | True | — | 49.1 | 12.37 | 1.245 | 0.073 | 0 | 2.37e+02 | 148.1 |
| backward_4n | axis_x | -4/0/0 | 0.400 | 1.00 | RECOVERED | True | — | 97.6 | 24.48 | 1.915 | 0.086 | 0 | 4.42e+02 | 147.2 |
| left_1n | axis_y | 0/1/0 | 0.100 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×508; loop overrun×519 | 522.6 | 158.97 | — | 1.000 | 0 | 1.85e+05 | 10117.0 |
| left_2n | axis_y | 0/2/0 | 0.200 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×607; loop overrun×613 | 2881.2 | 177.89 | — | 1.000 | 0 | 1.02e+06 | 10929.0 |
| left_4n | axis_y | 0/4/0 | 0.400 | 1.00 | NUMERIC_FAULT | False | numeric:mjWARN_BADQACC; fall; no recovery; WBC nonadmitted×741; loop overrun×748 | 28793.7 | 177.63 | — | 1.000 | 1 | 4.61e+06 | 10777.3 |
| right_2n | axis_y | 0/-2/0 | 0.200 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×485; loop overrun×498 | 2833.6 | 170.59 | — | 1.000 | 0 | 2.83e+06 | 10457.6 |
| right_4n | axis_y | 0/-4/0 | 0.400 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×570; loop overrun×572 | 5303.6 | 178.76 | — | 1.000 | 0 | 1.59e+06 | 9832.7 |
| diagonal_4n | axis_xy | 2.82843/2.82843/0 | 0.400 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×349; loop overrun×359 | 547.3 | 108.50 | — | 0.956 | 0 | 4.53e+04 | 9369.4 |
| handle_forward_4n | application_point | 4/0/0 | 0.400 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×225; loop overrun×234 | 1126.5 | 148.09 | — | 1.000 | 0 | 1.54e+05 | 9682.9 |
| short_8n_50ms | duration | 8/0/0 | 0.400 | 1.00 | RECOVERED | True | — | 108.1 | 25.18 | 1.970 | 0.085 | 0 | 6.17e+02 | 149.3 |
| long_2n_200ms | duration | 2/0/0 | 0.400 | 1.00 | RECOVERED | True | — | 105.8 | 23.99 | 1.905 | 0.081 | 0 | 5.97e+02 | 146.1 |
| forward_4n_friction_0p1 | friction | 4/0/0 | 0.400 | 0.10 | RECOVERED | False | WBC nonadmitted×8 | 107.6 | 24.90 | 1.950 | 0.084 | 0 | 5.47e+02 | 144.5 |
| forward_4n_friction_0p03 | friction | 4/0/0 | 0.400 | 0.03 | FALL | False | fall; no recovery; WBC nonadmitted×594; loop overrun×600 | 5618.7 | 177.95 | — | 1.000 | 0 | 8.56e+05 | 10205.0 |

## Evaluation gates

| gate | observed | pass |
|---|---|---|
| complete frozen matrix | 17/17 cases · ['application_point', 'axis_x', 'axis_xy', 'axis_y', 'duration', 'friction', 'nominal'] | True |
| all traces finite | True | True |
| canonical forward recovery retained | RECOVERED | True |
| exact canonical semantic replay | True | True |
| matrix is discriminating | 7 qualified / 10 failed | True |
| lateral sign symmetry is measured | 0.016518901017536548 | True |
| friction boundary is discriminating | RECOVERED @ 1.00 / FALL @ 0.03 | True |
| MuJoCo warnings are explicitly classified | 1 | True |
| Rust timed region allocation-free | True | True |

## Interpretation

- Qualified controller cases: **['nominal', 'forward_2n', 'forward_4n_reference', 'backward_2n', 'backward_4n', 'short_8n_50ms', 'long_2n_200ms']**.
- Failed/unsettled controller cases: **['forward_6n_overload', 'left_1n', 'left_2n', 'left_4n', 'right_2n', 'right_4n', 'diagonal_4n', 'handle_forward_4n', 'forward_4n_friction_0p1', 'forward_4n_friction_0p03']**.
- Exact semantic repeat of the canonical 4 N row: **True**.
- Left/right 2 N response asymmetry: **1.652e-02** relative peak translation.
- Canonical/low-friction discrimination: **True**; μ=0.10 physically recovers but exposes later WBC non-admission, while μ=0.03 crosses the fall boundary.
- MuJoCo numeric warnings captured across the matrix: **1**.
- `qualified=false` is not collapsed into one reason: FALL, UNSETTLED, and NUMERIC_FAULT remain distinct, alongside contact loss, later WBC nonadmission, effort use, loop budget, and recovery timing.

## Deliberate limits

This is still an ideal-observation soft-contact MuJoCo consequence test, not a learned policy and not hardware. It does not add lateral capture logic to make the chart greener. Plant friction is varied while the Rust contact model retains its compiled coefficient, deliberately exposing model mismatch. Terrain slope, delayed/noisy observation, repeated impulses, simultaneous contacts, motor bandwidth, thermal derating, and a measured contact-mode estimator remain separate future axes. The policy-/physics-free state-local contact replay remains the semantic gate beneath this plant matrix.
