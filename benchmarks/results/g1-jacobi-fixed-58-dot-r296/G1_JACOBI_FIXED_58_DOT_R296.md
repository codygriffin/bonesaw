# G1 fixed-58 Jacobi dot experiment · R296

> Arithmetic candidate **REJECTED** · semantic replay **PASS** · allocation gate
> **PASS** · pinned CPU-work gate **FAIL** · authority **CLOSED**.

R296 tested an opt-in specialization of the measured G1 Jacobi coupling dot.
The candidate used a fixed 58-element scalar loop when the decision-space row
count was 58, preserving the existing multiply/add order. It was evaluated
against the current R293 default with the policy-free, physics-free G1 oracle
admission corpus; no plant action, policy query, or authority decision was
introduced.

## Acceptance gates

| gate | result |
|---|---|
| 56 retained non-timing arrays exact in all five control/candidate pairs | PASS |
| zero measured Rust hot-loop allocations | PASS |
| lower median retired instructions | FAIL |
| lower retired instructions in every pinned pair | FAIL |
| default behavior or authority change | NONE |

All five CPU-4-pinned alternating-process pairs increased retired
instructions. The candidate median delta was **+0.30950%**; pair deltas were
`+0.30754%`, `+0.30944%`, `+0.30956%`, `+0.30968%`, and `+0.30863%`.
The candidate therefore fails the exact-work promotion rule even though host
cycles and wall time varied in both directions. The result is deterministic
enough to reject the specialization rather than attribute the mixed timing to
the dot loop.

## Timing context

The complete process was the existing G1 admission run (two 2,317-tick Rust
passes plus Python orchestration and artifact serialization). Median candidate
latency deltas were `-2.55%` p50 and `-12.97%` p99 in the shorter three-pair
rerun, but those host timing signals cannot override a stable retired-work
regression. RSS remained effectively unchanged and the Rust allocation gate
stayed zero.

## Decision and next gate

The feature was removed before checkpointing. R293's exact dense matvec
row-slice path remains the production CPU default. The ordinary sub-5 ms p99
gate and floating support-transfer behavior gate remain open. The next
behavior experiment must use a deterministic 250 Hz MuJoCo source trace and
50 Hz WBC replay with sequential raw masks (`11`, `10`, `01`, `00`), debounce,
fail-closed hard eligibility, state evolution, and reset/timing witnesses;
the current authored G1 contact schedule is not sufficient evidence for that
claim.

No policy, physics, walking, contact-transfer, thermal, CUDA, or hardware
authority claim follows from R296.
