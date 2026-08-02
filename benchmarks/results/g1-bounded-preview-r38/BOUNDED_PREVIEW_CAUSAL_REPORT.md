# G1 bounded-reference causal audit · r38

## Outcome

Revision r38 fixes two evaluation-reference defects before attempting another
controller mechanism:

1. `dcm-backward-preview` was a whole-buffer backward pass, so appending future
   samples rewrote earlier CoM/DCM targets and `support_preview_ticks` did not
   describe the actual horizon.
2. floating CMU lateral-root registration estimated its affine scale from the
   requested output duration, so a truncated final gait cycle changed every
   earlier root target.

The corrected reference is causal over a declared receding horizon and is
prefix-stable outside the finite-difference/end-preview boundary. The new
regression test compares all root, endpoint, contact, cadence, and phase arrays
from 600- and 800-tick reconstructions.

This is a reference-semantics pass, not a walking-policy pass. With the same r37
Rust controller policy, a bounded 200-tick (1.0 s) DCM horizon produces the best
corrected full-transfer trace: first contingency at tick 480, no rejected
solves, 9.73 cm minimum landing-position error, and 1.626 m/s minimum
whole-patch tangential speed. The unchanged touchdown gate requires 2.5 cm and
0.20 m/s, so the transfer remains red and no false contact is admitted.

## Why the old result could not be tuned safely

For a discrete LIPM reference with natural frequency `ω`, the DCM boundary law
is evaluated backward over a fixed interval `H`:

`ξ[k] = zmp[k] + exp(-ω dt) · (ξ[k+1] - zmp[k])`

The former implementation seeded this recurrence once at the last sample in
the caller's array. Its effective horizon was therefore `N-k`, not the
configured `support_preview_ticks`. The same nominal 600-tick prefix embedded
in 600- and 800-tick requests differed by as much as 29.36 cm in CoM target and
1.10 m/s in target velocity before the fix.

The replacement performs a receding fixed-horizon solve at each tick, seeded
from support at `min(k+H, N-1)`. Future support beyond `H` cannot affect the
current reference. Only the final `H` samples retain the explicitly documented
terminal boundary dependence.

Separately, lateral root registration now uses the pinned canonical CMU cycle's
bilateral single-support root means. It never estimates morphology registration
from the runtime output duration.

## Prefix invariance evidence

The direct unit regression reconstructs the cached CMU subject 37/trial 1 walk
at 600 and 800 ticks and requires exact equality for:

- root positions;
- all four endpoint positions;
- bilateral stance labels;
- cadence scale; and
- source phase.

The bounded DCM unit tests additionally prove that extending a trace does not
alter its completed preview prefix and that support changes beyond the horizon
have zero influence before the expected causal boundary.

In the real G1 pair, root and endpoint targets, contacts, cadence, and source
phase are byte-identical through tick 599. With `H=40`, CoM targets are
byte-identical through tick 565; the first difference is exactly the terminal
preview boundary. The simulated states remained identical through tick 431.
At tick 432 the runtime-bounded solver took a different amount of iterative work
under host timing, so the physical traces diverged before the reference
boundary. This is reported as runtime-work sensitivity, not reference leakage.

## Corrected bounded-horizon sweep

All rows use the same pinned Unitree G1 model, 600 × 5 ms ticks, r37 8 cm / 0.50
m/s capture-landing policy, zero-tick landing freeze, 200-tick precontact,
feedback velocity-envelope authority, and unchanged hard/touchdown gates.

| horizon | first contingency | failed | root RMS | stance RMS | swing RMS | DCM RMS | min landing position | min tangential speed |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 40 ticks | 408 | 77 | 104.58 cm | 67.53 cm | 84.93 cm | 26.08 cm | 52.45 cm | 1.687 m/s |
| 80 ticks | 465 | 23 | 89.41 cm | 64.29 cm | 99.65 cm | 99.58 cm | 37.82 cm | 1.104 m/s |
| 120 ticks | 334 | 0 | 125.34 cm | 93.59 cm | 120.23 cm | 106.47 cm | 73.89 cm | 0.935 m/s |
| **200 ticks** | **480** | **0** | **59.63 cm** | **32.23 cm** | **52.25 cm** | **39.36 cm** | **9.73 cm** | 1.626 m/s |
| 300 ticks | 424 | 0 | 74.31 cm | 46.20 cm | 62.30 cm | 34.60 cm | 28.87 cm | 1.331 m/s |

The sweep is deliberately not ranked by a scalar score. H=200 is retained
because it preserves the longest clean prefix without rejected solves and has
the closest physically measured landing. Its full-run root attitude still
reaches 179.89°, signed DCM margin reaches -152.60 cm, and no pending touchdown
sample passes either the position or tangential gate.

## Repeatability and execution-time tails

Three independent H=200 runs produced bit-for-bit identical target, state,
status, and solver-work arrays. Every run entered its first contingency at tick
480 with 126 solved, 102 solved-with-slack, 252 precontact-transition, 96
normal-contact-contingency, 24 release-contingency, and zero rejected ticks.

| run | p50 | p95 | p99 | behavioral trace |
|---:|---:|---:|---:|---|
| 1 | 2.608 ms | 81.673 ms | 183.857 ms | exact |
| 2 | 2.676 ms | 81.544 ms | 181.453 ms | exact |
| 3 | 2.611 ms | 80.788 ms | 182.239 ms | exact |

The large tail occurs only after the deliberately retained controller failure
drives clipped/contingency solver work. It is not promoted as a real-time
walking result.

## Unchanged regression cases

- Moving G1 liftoff: functional **PASS**, real-time **PASS**, 4.125 ms p99,
  zero contingency/rejected ticks, 0.787/0.0002/0.342 cm root/stance/swing RMS.
- Synthetic 1 cm toe-step: functional **PASS** in all five repeats. Four of five
  runs pass the 5 ms p99 gate; median p99 is 4.623 ms. The retained first run is
  timing-red at 5.904 ms p99 with an 11.432 ms maximum scheduler tail.
- Rust workspace: 92 tests pass (`bonesaw-core` 90, `bonesaw-tools` 2).
- Python causal-reference tests: 3 pass, including the cached-CMU prefix test.
- Hot-loop native allocation sentinels remain 0 calls / 0 bytes per tick.

## Reference implementations

The refreshed [reference comparison](../reference-r38/REFERENCE_COMPARISON.md)
uses the r38 moving-liftoff and H=200 transfer artifacts while retaining the
fresh matched PlaCo, Pinocchio 4.0, and upstream Upkie runs:

- Bonesaw is faster than PlaCo at p99 in all three matched fixed-base scenarios
  (52.8 vs 170.4 µs reach, 102.8 vs 124.8 µs conflict, and 72.3 vs 159.3 µs
  walking). PlaCo tracks the walking corpus better (2.634 vs 4.048 cm overall
  RMS), while Bonesaw tracks reach better and the conflict case slightly worse.
- Bonesaw peaks near 43–50 MB RSS versus PlaCo near 96–102 MB in the isolated
  workers. Both record zero Python GC collections during measured loops.
- The official-parameter Rust Upkie law is bitwise exact for 100,000 sequential
  samples: zero command mismatches and zero numeric error. Its typed call is
  0.030/0.031 µs p50/p99 versus 1.393/2.314 µs for the upstream C++ dictionary
  adapter; this is boundary cost, not a whole-WBC speed claim.
- Pinocchio differential oracles pass all 50 states for Upkie and G1. Maximum
  G1 floating bias/inverse-dynamics error is 1.71e-13.

## Architectural decision and next experiment

The fixed-horizon reference construction remains Python eval-land: it is pinned
corpus interpretation, not runtime control. Rust continues to own every measured
state transition, DCM/ZMP feedback, landing retarget, touchdown admission, solve,
and integration tick.

The r38 evidence sharpens the next CPU-core experiment. Endpoint relocation and
a physically honest delayed contact are insufficient. A measured phase policy
must retime one coherent root/CoM/swing jet, preserve contact ordering, expose
its phase cursor/rate as explicit controller state, and preview whether both
position and material-point velocity can enter the immutable touchdown gate.
CUDA batching remains deferred.

## Artifacts

- Selected trace: `benchmarks/results/g1-transfer-r38-preview-200`
- Exact repeats: `g1-transfer-r38-preview-200-repeat-2` and `-repeat-3`
- Prefix pair: `g1-transfer-r38-prefix-stable-600` and `-800`
- Horizon cases: `g1-transfer-r38-preview-{80,120,200,300}`
- Green liftoff: `benchmarks/results/floating-g1-liftoff-r38-reference-fix`
- Toe-step repeat set: `benchmarks/results/g1-synthetic-step-r38-reference-fix*`
- Full independent comparison: `benchmarks/results/reference-r38`

Every case retains Markdown, machine-readable JSON, and compressed raw NPZ
arrays. No acceptance threshold was relaxed after observing the data.
