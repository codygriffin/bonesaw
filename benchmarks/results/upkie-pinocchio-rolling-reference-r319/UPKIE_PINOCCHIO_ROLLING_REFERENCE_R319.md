# Upkie exact RollingWheel Pinocchio reference · R319

## Result

**NOT YET PARITY-QUALIFIED.** Pinocchio independently rebuilds the floating rigid-body products and wheel-center Jacobians; PlaCo's low-level generic QP independently rebuilds the constrained hierarchy without using its robot, contact, task, or DynamicsSolver APIs. Both use Bonesaw's exact nonholonomic rolling acceleration row and augmented virtual-work force map on the same 256 frozen R123 states.

> There is no policy, integration, state propagation, simulator, or physics rollout. The physical RollingWheel rows are exact; the generic QP preserves each achieved hierarchy image through a declared ±2e-9 numerical band. Every acceleration, effort, unilateral-normal, normal-cap, and square-friction inequality is present.

## Differential result

| quantity | RMS delta | p99 absolute | maximum absolute |
|---|---|---|---|
| generalized acceleration | 1.689e+00 | 1.864e-01 | 6.036e+01 |
| actuator torque | 4.826e-03 | 2.352e-02 | 8.580e-02 |
| rolling-basis contact force | 2.237e+00 | 2.028e+01 | 2.104e+01 |

The broad distribution hides a localized active-set disagreement: `239/256` states agree within 1.0 over the complete 24-variable solution, while `17` states exceed it. Both solvers are hard-feasible; the parity failure is an optimizer-path difference, not a missing rolling row.

## State parity and active-set boundary

| complete-solution threshold | states inside |
|---|---|
| 2e-5 | 114 |
| 2e-4 | 122 |
| 1e-3 | 125 |
| 1e-2 | 174 |
| 1 | 239 |

Near-active hard-bound states: reference `166/256`; Bonesaw `154/256`; mask agreement `242/256`.

## Hard equations and bounds

- Reference maximum dynamics/rolling residual: `2.747e-13`.
- Bonesaw maximum residual under independent Pinocchio products: `2.256e-11`.

| implementation | min accel margin | min effort margin | min normal margin | min friction margin | near-active states |
|---|---|---|---|---|---|
| Bonesaw | 0.000e+00 | 1.089704 | 24.902577 | 0.000000 | 154 |
| reference | -8.527e-14 | 1.089704 | 24.902577 | 10.667328 | 166 |

## Shared-target tracking

| target | Bonesaw RMS | reference RMS |
|---|---|---|
| root angular | 0.000126 | 0.000000 |
| root horizontal | 0.886564 | 0.886399 |
| root height | 0.000120 | 0.000000 |
| joint | 141.783789 | 142.074766 |

## CPU, jitter, and memory

| implementation | solve p50 / p99 / max µs | jitter p99 µs | queries/s | peak RSS MiB | GC |
|---|---|---|---|---|---|
| Bonesaw R317 | 118.8 / 131.2 / 147.4 | 15.0 | 8384 | 88.71 | 0 |
| Pinocchio + generic QP | 2622.0 / 2761.5 / 3219.9 | 140.0 | 380 | 97.69 | 0 |

The reference timing includes construction and solution of the independent constrained hierarchy through PlaCo's generic QP API. Its one-time Pinocchio product build costs `28.079 ms` for all 256 states; that cost is retained separately because Bonesaw emits products inside each query.

## Execution windows

| states | active | max solution delta | qdd RMS | torque RMS | force RMS | reference p99 µs |
|---|---|---|---|---|---|---|
| 0–31 | 32 | 6.036e+01 | 4.630e+00 | 1.317e-02 | 2.111e+00 | 2779.3 |
| 32–63 | 29 | 2.104e+01 | 1.180e+00 | 3.563e-03 | 5.200e+00 | 2748.7 |
| 64–95 | 17 | 2.960e-02 | 3.191e-03 | 1.236e-05 | 1.531e-04 | 2745.1 |
| 96–127 | 32 | 6.736e-02 | 4.133e-03 | 3.441e-05 | 2.733e-04 | 2697.0 |
| 128–159 | 13 | 1.486e-01 | 1.242e-02 | 7.835e-05 | 6.942e-04 | 2745.4 |
| 160–191 | 9 | 2.004e+01 | 1.885e-02 | 5.429e-05 | 2.046e+00 | 2748.0 |
| 192–223 | 2 | 6.749e-02 | 1.046e-02 | 2.965e-05 | 4.532e-04 | 2784.3 |
| 224–255 | 32 | 2.042e+01 | 1.272e-02 | 7.124e-05 | 2.084e+00 | 2782.0 |

## Gates

- PASS `all_256_states_solved`
- PASS `independent_solution_satisfies_rolling_and_dynamics_below_2e_8`
- PASS `bonesaw_solution_satisfies_pinocchio_rolling_and_dynamics_below_2e_8`
- PASS `independent_reference_repeats_bit_exactly`
- PASS `independent_solution_satisfies_every_hard_inequality`
- OPEN `generalized_acceleration_matches_bonesaw_within_2e_5`
- OPEN `actuator_torque_matches_bonesaw_within_2e_4_nm`
- OPEN `rolling_force_matches_bonesaw_within_2e_4_n`

## Boundary

R319 closes independent nonholonomic row and hard-bound reconstruction, but not complete optimizer parity: acceleration-bound active-set choices remain measurably different. It does not establish closed-loop stability, measured-contact transfer, calibrated actuator/thermal authority, or hardware timing. Those remain separate consequence gates.

## Provenance

| artifact | version or SHA-256 |
|---|---|
| Pinocchio | 3.8.0 |
| PlaCo | 0.9.23 |
| NumPy | 2.3.5 |
| model | d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483 |
| input | f31c69dd9b1f8bab7f58a8870b53e390b45bae848c348668009440e24c1e1e3e |
| bonesaw | 3574846fab8ac26935cd722aa497b89bd8d4f259841d9cb14db35fa2e89232d5 |
| bonesaw_metrics | 9d3f854ed90be8ced310f028636fabb8e2cca65d8cb54220177493f18ace2ad7 |
| evaluator | 35e9ff4629916e1f1c37884ec3581a026810266bc7824095cf1dc2645abadf38 |
