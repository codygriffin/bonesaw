# Bonesaw terminal candidate realization calibration · r192

> Cross-case calibration **REJECTED** · controller promotion **NO**.

## Result

The audit measures **481** selected terminal actions across **7 cases × 5 profiles**. It compares each r191 candidate's exact roll/pitch and six-joint acceleration against the finite difference of measured MuJoCo velocities over the next complete 5 ms control interval.
Strict leave-one-case-out componentwise-max bounds cover **96.881%** of complete samples and **99.142%** of scalar component values; the worst exceedance is **26.496×**. A 5% reserve covers **97.089%** of complete samples with a **25.234×** worst exceedance.

| selected action | samples | cases | |error|∞ p50 | p99 | max |
|---|---|---|---|---|---|
| withhold | 337 | 7 | 173.733 | 745.806 | 14862.612 |
| retained | 20 | 7 | 2.930 | 3339.095 | 4069.007 |
| support_free | 124 | 7 | 175.546 | 1811.327 | 2862.454 |

## Admission gates

| gate | result |
|---|---|
| full_case_matrix | PASS |
| finite | PASS |
| all_actions_observed | PASS |
| every_action_spans_multiple_cases | PASS |
| leave_one_case_out_exercised | PASS |
| strict_loco_all_component_coverage | FAIL |
| five_percent_reserve_loco_all_component_coverage | FAIL |
| zero_rust_allocation | PASS |
| zero_python_gc | PASS |

## Interpretation

- This is a measured realization audit, not a new selector. A failed holdout bound cannot authorize a candidate and cannot be hidden inside the r191 aggregate score.
- The realized value includes actuator/contact response and any declared external wrench during that 5 ms interval. It therefore tests the exact place where a WBC qdd prediction becomes a plant claim.
- Only the selected candidate has a physical counterfactual. Unselected candidate accelerations remain model predictions and are not treated as realized evidence.
- Python owns the corpus, split, and statistics. Rust remains the source of candidate qdd, typed selection, and allocation evidence.

## Scope

Upkie MuJoCo plant · measured world root angular velocity + six joint velocities · 5 ms interval finite difference · leave-one-named-case-out maximum bounds · no policy · no learned model · no hardware or injury claim.
