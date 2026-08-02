# Bonesaw G1 stiff-contact activation localization · r229

> Label-explicit diagnostic **PASS** · label-derived construction/selection **INELIGIBLE** · profile **NOT PROMOTED** · authority **NOT ADMITTED** · physics/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.

## Boundary

- This audit reads the completed immutable R228 contact impulses explicitly. It localizes a rejected holdout; it cannot choose a threshold, freeze a profile, or authorize a command.
- The activity test is numerical bookkeeping at 1e-12 N·s. The causal prestate response is reconstructed with 48 MuJoCo forward queries and 48 allocation-audited Rust point-response queries, but no model is integrated.
- Point indices 0–3 are the left-foot spheres and 4–7 are the right-foot spheres. A predicted-only point is not automatically an error: both passing laws contain some low-impulse predicted-only points.
- A retained independent rerun reproduces all ten semantic arrays, normalized metrics, and this Markdown report exactly.

## Activation localization

| law | covered rows | covered rows with predicted-only points | uncovered rows | uncovered rows with predicted-only points | actual points missed by prediction |
|---|---|---|---|---|---|
| soft_pyramidal_euler | 48 | 13 | 0 | 0 | 0 |
| stiff_elliptic_implicitfast | 42 | 8 | 6 | 6 | 0 |

Every stiff miss predicts at least one additional point, and no actual loaded point is absent from the prediction. But predicted-only activation is not sufficient by itself: it also occurs in 8/42 covered stiff rows and 13/48 covered soft rows.

Within this spent holdout, summed predicted-only impulse is strictly separated: the largest covered stiff row is 0.176 N·s and the smallest uncovered row is 0.241 N·s. This is localization evidence only, not an eligible threshold.

## Six rejected stiff rows

| state | MuJoCo active | reduced active | predicted-only impulse N·s | original max abs residual | label-mask max abs residual | label-mask coverage |
|---|---|---|---|---|---|---|
| 100007 | [4] | [4, 5] | 0.347 | 6.027 | 0.559 | PASS |
| 100008 | [4] | [4, 5] | 0.506 | 10.367 | 2.351 | PASS |
| 100009 | [4, 5] | [4, 5, 6, 7] | 1.354 | 7.751 | 33.726 | FAIL |
| 100014 | [1] | [1, 5] | 0.241 | 4.816 | 0.387 | PASS |
| 100036 | [6] | [1, 6] | 0.764 | 11.792 | 1.139 | PASS |
| 100037 | [1, 3, 6] | [0, 1, 2, 3, 6, 7] | 4.911 | 23.488 | 90.433 | FAIL |

The label-oracle counterfactual zeros predicted impulse at completed-label inactive points without re-solving coupling. It repairs four of six original misses (42/48 → 46/48), but states 100009 and 100037 remain outside the frozen box and worsen sharply. Therefore a contact mask or magnitude threshold is not the missing mechanism.

## Causal first-impact cohort

A separate label-free construction evaluates ballistic surface crossing only at the authored five 1 ms plant ticks. It uses prestate gap, normal velocity, and smooth free acceleration; completed impulses enter only after the cohort is frozen for scoring.

Across the six rejected rows, the first cohort is a subset of the eventual MuJoCo active set in 6/6 rows, introduces 0 false points, and is already the exact final active set in 5/6 rows. The remaining row starts at point 6 and later grows to points 3 and 1. The cohort therefore identifies initial activation but cannot be treated as a terminal predictor; later contacts require contact-coupled state evolution on the declared clock.

## Decision

Build the next construction around the independently declared event clock: admit the first ballistic cohort, advance the generalized state, update point geometry and velocity, recompute the full Delassus response, and only then test later cohorts. Freeze that mechanism from model equations and convergence only, then spend a new untouched law/state holdout. R228 remains rejected and no selector, plant action, or authority is admitted.
