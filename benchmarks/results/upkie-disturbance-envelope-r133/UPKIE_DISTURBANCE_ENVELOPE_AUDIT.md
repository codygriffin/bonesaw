# Upkie multidimensional disturbance envelope · upkie-disturbance-envelope-r133

**Evaluation admission: PASS.** This report freezes a discriminating plant-consequence matrix; it does not require every controller row to recover. Python owns MuJoCo integration, case construction, external wrench application, friction variation, and scoring. Persistent Rust sessions own rooted capture/station state, floating WBC, contact/rolling equations, hierarchy, torque, and allocation counters.

The matrix separates all three force axes, sign, impulse duration, repeated impulses, application body, and plant friction. The live controller remains intentionally sagittal: red lateral or low-friction rows are measured capability boundaries, not evaluator failures. Every row terminates at its first declared 45°/350 mm fall boundary or pre-boundary numeric fault, so post-fall solver behavior cannot pollute control latency or be mislabeled as recovery.

## Case matrix

| case | family | force xyz N | impulse N·s | pulses | friction | outcome | qualified | qualification blockers | peak Δp mm | peak tilt ° | recovery s | torque use | warnings | peak qacc abs | Rust p99 µs | terminal s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | nominal | 0/0/0 | 0.000 | 1 | 1.00 | RECOVERED | True | — | 0.1 | 0.05 | 0.000 | 0.067 | 0 | 1.80e+00 | 151.3 | 6.000 |
| forward_2n | axis_x | 2/0/0 | 0.200 | 1 | 1.00 | RECOVERED | True | — | 52.0 | 12.53 | 1.305 | 0.069 | 0 | 2.40e+02 | 151.0 | 6.000 |
| forward_4n_reference | axis_x | 4/0/0 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 107.6 | 24.91 | 1.950 | 0.084 | 0 | 5.46e+02 | 153.5 | 6.000 |
| forward_6n_overload | axis_x | 6/0/0 | 0.600 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×44; loop overrun×45 | 228.8 | 32.69 | — | 0.757 | 0 | 1.52e+04 | 9301.8 | 4.615 |
| backward_2n | axis_x | -2/0/0 | 0.200 | 1 | 1.00 | RECOVERED | True | — | 49.1 | 12.37 | 1.245 | 0.073 | 0 | 2.37e+02 | 172.4 | 6.000 |
| backward_4n | axis_x | -4/0/0 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 97.6 | 24.48 | 1.915 | 0.086 | 0 | 4.42e+02 | 144.8 | 6.000 |
| left_1n | axis_y | 0/1/0 | 0.100 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×72; loop overrun×74 | 199.5 | 25.07 | — | 0.877 | 0 | 3.62e+04 | 9947.3 | 2.260 |
| left_2n | axis_y | 0/2/0 | 0.200 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×56; loop overrun×57 | 274.0 | 49.92 | — | 0.872 | 0 | 6.20e+04 | 9373.9 | 1.985 |
| left_4n | axis_y | 0/4/0 | 0.400 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×41; loop overrun×44 | 174.6 | 45.00 | — | 1.000 | 0 | 1.11e+04 | 9060.6 | 1.260 |
| right_2n | axis_y | 0/-2/0 | 0.200 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×23; loop overrun×24 | 140.8 | 46.95 | — | 0.543 | 0 | 3.55e+03 | 8220.2 | 1.990 |
| right_4n | axis_y | 0/-4/0 | 0.400 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×42; loop overrun×44 | 202.3 | 22.20 | — | 0.804 | 0 | 7.76e+03 | 9235.1 | 2.215 |
| diagonal_4n | axis_xy | 2.82843/2.82843/0 | 0.400 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×75; loop overrun×81 | 218.4 | 46.69 | — | 0.956 | 0 | 4.53e+04 | 9691.2 | 2.095 |
| up_4n | axis_z | 0/0/4 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 4.3 | 2.40 | 0.335 | 0.067 | 0 | 1.52e+01 | 137.5 | 6.000 |
| down_4n | axis_z | 0/0/-4 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 5.0 | 2.76 | 0.380 | 0.076 | 0 | 1.44e+01 | 146.2 | 6.000 |
| handle_forward_4n | application_point | 4/0/0 | 0.400 | 1 | 1.00 | FALL | False | fall; no recovery; WBC nonadmitted×79; loop overrun×84 | 318.9 | 45.19 | — | 1.000 | 0 | 1.05e+05 | 9346.7 | 4.190 |
| short_8n_50ms | duration | 8/0/0 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 108.1 | 25.18 | 1.970 | 0.085 | 0 | 6.17e+02 | 154.9 | 6.000 |
| long_2n_200ms | duration | 2/0/0 | 0.400 | 1 | 1.00 | RECOVERED | True | — | 105.8 | 23.99 | 1.905 | 0.081 | 0 | 5.97e+02 | 160.7 | 6.000 |
| forward_2n_three_pulses | repeated | 2/0/0 | 0.600 | 3 | 1.00 | RECOVERED | True | — | 104.1 | 12.53 | 1.690 | 0.070 | 0 | 4.14e+02 | 149.4 | 6.000 |
| forward_4n_friction_0p1 | friction | 4/0/0 | 0.400 | 1 | 0.10 | RECOVERED | False | WBC nonadmitted×8 | 107.6 | 24.90 | 1.950 | 0.084 | 0 | 5.47e+02 | 149.1 | 6.000 |
| forward_4n_friction_0p03 | friction | 4/0/0 | 0.400 | 1 | 0.03 | FALL | False | fall; no recovery; WBC nonadmitted×36; loop overrun×36 | 260.4 | 28.36 | — | 0.529 | 0 | 5.16e+03 | 9284.7 | 1.560 |

## Evaluation gates

| gate | observed | pass |
|---|---|---|
| complete frozen matrix | 20/20 cases · ['application_point', 'axis_x', 'axis_xy', 'axis_y', 'axis_z', 'duration', 'friction', 'nominal', 'repeated'] | True |
| all traces finite | True | True |
| canonical forward recovery retained | RECOVERED | True |
| exact canonical semantic replay | True | True |
| matrix is discriminating | 10 qualified / 10 failed | True |
| lateral sign pair has a repeatable boundary | FALL at 1.985/1.990 s; relative timing difference 2.513e-03; path difference 4.859e-01 | True |
| friction boundary is discriminating | RECOVERED @ 1.00 / FALL @ 0.03 | True |
| MuJoCo warnings are explicitly classified | 0 | True |
| no pre-boundary numeric fault | 0 | True |
| failure rows stop at first boundary | True | True |
| Rust timed region allocation-free | True | True |

## Interpretation

- Qualified controller cases: **['nominal', 'forward_2n', 'forward_4n_reference', 'backward_2n', 'backward_4n', 'up_4n', 'down_4n', 'short_8n_50ms', 'long_2n_200ms', 'forward_2n_three_pulses']**.
- Failed/unsettled controller cases: **['forward_6n_overload', 'left_1n', 'left_2n', 'left_4n', 'right_2n', 'right_4n', 'diagonal_4n', 'handle_forward_4n', 'forward_4n_friction_0p1', 'forward_4n_friction_0p03']**.
- Exact semantic repeat of the canonical 4 N row: **True**.
- The ±2 N lateral rows both end in FALL at **1.985/1.990 s** (**2.513e-03** relative timing difference). Their first-boundary peak translations differ by **4.859e-01**, retained as real path asymmetry rather than called symmetric.
- Canonical/low-friction discrimination: **True**; μ=0.10 physically recovers but exposes later WBC non-admission, while μ=0.03 crosses the fall boundary.
- MuJoCo numeric warnings captured across the matrix: **0**.
- Failure rows stopped at their first declared boundary: **True**.
- `qualified=false` is not collapsed into one reason: FALL, UNSETTLED, and NUMERIC_FAULT remain distinct, alongside contact loss, later WBC nonadmission, effort use, loop budget, and recovery timing.

## Deliberate limits

This is still an ideal-observation soft-contact MuJoCo consequence test, not a learned policy and not hardware. It does not add lateral capture logic to make the chart greener. Plant friction is varied while the Rust contact model retains its compiled coefficient, deliberately exposing model mismatch. Repeated sagittal impulse tolerance is now measured, but terrain slope, delayed/noisy observation, simultaneous contacts, motor bandwidth, thermal derating, and a measured contact-mode estimator remain separate future axes. The policy-/physics-free state-local contact replay remains the semantic gate beneath this plant matrix.
