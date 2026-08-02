# Bonesaw support-hypothesis terminal envelope A/B · r195

> Mechanism **PASS** · realization bracketing **FAIL** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Explicit uncertainty model

- Missing support evidence no longer collapses the plant to one fictitious contact mode. Rust fixed-effort dynamics realizes withhold, retained effort, and support-free effort under none/left/right/double support; Rust then takes a componentwise harm envelope and applies the unchanged conservative typed chooser.
- Every action × support query is state-local and bounded. MuJoCo remains an offline consequence oracle only. Unavailable alternatives fail closed, ballistic evidence remains common, and no envelope component is hidden inside one scalar score.
- For every executed selection, the audit finite-differences the following 5 ms plant velocity, rescales that realized acceleration through the same Rust scorer, and tests each pressure against the selected action's four-support envelope. The measured physical support hypothesis remains separately visible, so model error cannot be mislabeled as support uncertainty.
- Stored first-run work: **40,982 control ticks**; attempted **270** support envelopes and independently re-audited **266** valid online selections.
- **4** attempts had at least one inadmissible baseline support witness. All remained typed withhold: authority 0, non-executable, exactly zero actuator effort. They are mechanism evidence, not plant-bracketing samples.

## Result

- Componentwise realization coverage is **23/266 (8.65%)**. Maximum envelope exceedance is **267.523 pressure**; maximum measured-physical-hypothesis error norm is **271.247**.
- Selected actions withhold/retained/support-free are **252/1/13**. Measured none/left/right/double support counts are **19/5/9/233**.
- Profiles without a terminal selection on every unavailable tick: **[{'case': 'forward_4n_friction_0p03', 'profile': 'drop10_r195_envelope', 'unavailable_ticks': 6, 'queried_ticks': 3}, {'case': 'forward_4n_friction_0p03', 'profile': 'drop5_matched_r195_envelope', 'unavailable_ticks': 3, 'queried_ticks': 2}]**. Invalid fixed-support dynamics fails closed to typed withhold; it is not silently removed from the envelope.
- The four support modes expose support uncertainty but do not bound plant realization error. The mechanism remains diagnostic and is not promoted to authority.

## Plant consequence

| case | dropout | exact | r191 single | r195 envelope | r195 fall Δ s |
|---|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 1.790s | FALL 1.790s | -0.680 |
| left_1n | drop10 | FALL 2.470s | FALL 2.735s | FALL 2.735s | +0.265 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.170s | FALL 2.170s | -0.350 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.365s | FALL 2.365s | -0.155 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 5.970s | FALL 3.385s | -1.245 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 4.230s | FALL 5.805s | +1.175 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.635s | FALL 1.700s | +0.090 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.595s | FALL 1.595s | -0.015 |

## Realization bracketing

| case | profile | samples | actions | physical masks | coverage | max exceedance | physical-hypothesis error max |
|---|---|---|---|---|---|---|---|
| nominal | drop5 | 23 | {'0': 23, '1': 0, '2': 0} | {'0': 0, '1': 0, '2': 0, '3': 23} | 0.0% | 0.718 | 1.320 |
| nominal | drop10 | 22 | {'0': 22, '1': 0, '2': 0} | {'0': 0, '1': 0, '2': 0, '3': 22} | 0.0% | 0.799 | 1.422 |
| nominal | drop5_matched | 11 | {'0': 11, '1': 0, '2': 0} | {'0': 0, '1': 0, '2': 0, '3': 11} | 0.0% | 0.669 | 1.303 |
| forward_4n_reference | drop5 | 23 | {'0': 22, '1': 0, '2': 1} | {'0': 0, '1': 0, '2': 0, '3': 23} | 13.0% | 1.590 | 2.426 |
| forward_4n_reference | drop10 | 22 | {'0': 22, '1': 0, '2': 0} | {'0': 0, '1': 0, '2': 0, '3': 22} | 4.5% | 1.595 | 2.610 |
| forward_4n_reference | drop5_matched | 11 | {'0': 11, '1': 0, '2': 0} | {'0': 0, '1': 0, '2': 0, '3': 11} | 9.1% | 1.595 | 2.399 |
| backward_4n | drop5 | 23 | {'0': 21, '1': 0, '2': 2} | {'0': 0, '1': 0, '2': 0, '3': 23} | 4.3% | 1.286 | 1.603 |
| backward_4n | drop10 | 22 | {'0': 20, '1': 0, '2': 2} | {'0': 0, '1': 0, '2': 0, '3': 22} | 4.5% | 1.344 | 1.614 |
| backward_4n | drop5_matched | 11 | {'0': 10, '1': 0, '2': 1} | {'0': 1, '1': 0, '2': 0, '3': 10} | 9.1% | 2.610 | 5.708 |
| left_1n | drop5 | 7 | {'0': 6, '1': 1, '2': 0} | {'0': 2, '1': 1, '2': 0, '3': 4} | 14.3% | 0.760 | 1.611 |
| left_1n | drop10 | 10 | {'0': 8, '1': 0, '2': 2} | {'0': 1, '1': 1, '2': 4, '3': 4} | 20.0% | 21.064 | 22.498 |
| left_1n | drop5_matched | 9 | {'0': 8, '1': 0, '2': 1} | {'0': 1, '1': 2, '2': 1, '3': 5} | 11.1% | 267.523 | 271.247 |
| right_1n_mirror | drop5 | 8 | {'0': 8, '1': 0, '2': 0} | {'0': 2, '1': 0, '2': 0, '3': 6} | 0.0% | 1.329 | 2.891 |
| right_1n_mirror | drop10 | 8 | {'0': 8, '1': 0, '2': 0} | {'0': 1, '1': 0, '2': 1, '3': 6} | 12.5% | 1.207 | 1.614 |
| right_1n_mirror | drop5_matched | 5 | {'0': 4, '1': 0, '2': 1} | {'0': 2, '1': 0, '2': 0, '3': 3} | 0.0% | 21.291 | 21.293 |
| handle_forward_4n | drop5 | 13 | {'0': 12, '1': 0, '2': 1} | {'0': 3, '1': 0, '2': 0, '3': 10} | 38.5% | 14.040 | 14.805 |
| handle_forward_4n | drop10 | 22 | {'0': 20, '1': 0, '2': 2} | {'0': 1, '1': 1, '2': 3, '3': 17} | 18.2% | 153.097 | 154.154 |
| handle_forward_4n | drop5_matched | 5 | {'0': 5, '1': 0, '2': 0} | {'0': 1, '1': 0, '2': 0, '3': 4} | 20.0% | 1.847 | 2.990 |
| forward_4n_friction_0p03 | drop5 | 6 | {'0': 6, '1': 0, '2': 0} | {'0': 1, '1': 0, '2': 0, '3': 5} | 0.0% | 1.231 | 13.622 |
| forward_4n_friction_0p03 | drop10 | 3 | {'0': 3, '1': 0, '2': 0} | {'0': 2, '1': 0, '2': 0, '3': 1} | 0.0% | 1.945 | 1.964 |
| forward_4n_friction_0p03 | drop5_matched | 2 | {'0': 2, '1': 0, '2': 0} | {'0': 1, '1': 0, '2': 0, '3': 1} | 0.0% | 3.394 | 4.547 |

Aggregate full-component coverage: **23/266 (8.6%)**.

| pressure | maximum envelope exceedance |
|---|---|
| impact_speed_pressure | 0.000000 |
| tilt_pressure | 3.160476 |
| angular_rate_pressure | 2.470431 |
| joint_position_pressure | 13.020948 |
| joint_velocity_pressure | 267.522596 |
| actuator_effort_pressure | 0.000000 |
| admission_pressure | 0.000000 |

## Runtime

- Twelve fixed-effort hypotheses / Rust aggregation maxima: **1329.660 / 0.692 µs**.
- Independent realized-interval rescoring maximum: **0.862 µs** with zero Rust allocation.
- Full loop: **70** 5 ms overruns; loop/controller maxima **6.578 / 5.793 ms**.
- Fixed-effort, scoring, aggregation, selection, and controller hot paths report zero Rust allocation; Python GC remains zero.
