# Bonesaw r30 · Jacobi energy-cache A/B

## Decision

Revision r30 adopts a cached column-energy kernel for the one-sided Jacobi
pseudoinverse. The cache is recomputed in the original column/summation order at
every sweep boundary; within a sweep, the two energies affected by a rotation
are updated algebraically while the coupling dot product is still evaluated
from the matrix. This removes repeated column norm scans without adding heap
allocation or changing convergence tolerances.

This is an algorithmic revision, not a parity-only optimization. The cached and
uncached kernels agree with the independent established Jacobi implementation
within `1e-10` relative error over random rectangular and rank-deficient
matrices, but rounding differences can cross later active-boundary decisions in
the G1 controller. Therefore the controller trajectory is reaccepted against
the unchanged physical, feasibility, and 5 ms gates.

## A/B protocol

- Host: AMD Ryzen 7 3700X, Linux x86-64, CPython 3.12.
- Workload: official Unitree G1 23-DOF mode-10 URDF, deterministic 160-tick
  one-centimetre toe-step, 5 ms reference period, four contact points per sole.
- Boundary: one fixed-shape PyO3 batch call; Rust owns every measured WBC tick.
- Control: identical r30 source with `CACHE_JACOBI_COLUMN_ENERGIES=false`.
- Candidate: `CACHE_JACOBI_COLUMN_ENERGIES=true`; five unpinned sequential
  repeats after one release build.
- Timing fields are excluded from behavioral fingerprints. No acceptance
  tolerance, task weight, motion reference, or contact schedule changed.

## Runtime results

| build/run | p50 µs | p95 µs | p99 µs | max µs | thread CPU ms | peak RSS MiB | jitter p99 µs | combined |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| uncached control | 3347.8 | 6359.2 | 8449.1 | 9818.0 | 617.2 | 41.89 | 3365.3 | FAIL |
| cached 1 | 2382.4 | 3683.4 | 4178.8 | 4792.9 | 411.6 | 41.90 | 1548.1 | PASS |
| cached 2 | 2358.6 | 3745.0 | 4450.0 | 4860.2 | 408.3 | 41.93 | 1782.3 | PASS |
| cached 3 | 2385.5 | 3732.3 | 4156.1 | 4939.7 | 404.3 | 42.02 | 1674.4 | PASS |
| cached 4 | 2396.3 | 3732.5 | 4364.4 | 4764.0 | 411.1 | 42.03 | 1906.0 | PASS |
| cached 5 | 2422.3 | 5254.1 | 5926.5 | 6132.9 | 464.7 | 41.94 | 2247.6 | FAIL |

Five-run cached medians are `4.364 ms` p99 and `411.1 ms` thread CPU. Against
the isolated uncached control, that is a 48.3% p99 reduction and 33.4% thread
CPU reduction. Four of five cached runs independently meet the unchanged 5 ms
p99 gate; the retained fifth run exposes an unpinned scheduler/CPU tail rather
than being discarded. All five cached runs have one identical non-timing
fingerprint.

## Behavior and hard constraints

| profile | gate | root RMS cm | stance RMS cm | swing RMS cm | root max deg | p99 ms | fallback / release / infeasible / failed | dynamics / contact max |
|---|---|---:|---:|---:|---:|---:|---|---|
| r30 toe-step canonical trace | PASS | 3.567 | 0.002 | 0.639 | 0.139 | 4.120 | 0 / 0 / 0 / 0 | `1.21e-9` / `4.58e-11` |
| r30 moving liftoff | PASS | 0.748 | 0.000 | 0.173 | 2.256 | 3.911 | 0 / 0 / 0 / 0 | `1.22e-9` / `5.16e-11` |
| r30 support-preview transfer | functional FAIL, CPU PASS | 23.670 | 26.328 | 28.588 | 9.339 | 4.808 | 0 / 0 / 0 / 0 | `1.62e-9` / `7.07e-11` |
| uncached toe-step control | timing FAIL | 4.191 | 0.240 | 0.639 | 1.413 | 8.449 | 0 / 0 / 0 / 0 | `1.21e-9` / `4.63e-11` |

The transfer result is important negative evidence: r30 removes false rejection
and meets the CPU deadline for the support-preview policy, but does not yet
produce acceptable sustained walking. Controller/reference design—not a hidden
hard-row relaxation—remains the blocking issue.

## Feasibility-correctness repair found during the A/B

The broader native corpus exposed two pre-existing hybrid-accelerator faults:

1. A structurally contradictory zero-norm row could leave a partial halfspace
   list with no matching multiplier storage. The error path now clears both
   buffers and has a dedicated regression.
2. Exhausting or cycling the bounded active set was incorrectly treated as
   proof of primal infeasibility. The active set now detects repeated working
   sets in a fixed `[u64; 64]` stack buffer. On accelerator failure, the solver
   restores the exact saved Dykstra iterate and correction state, then consumes
   the remainder of the original bounded Dykstra budget before returning a
   typed feasibility verdict.

Both repairs are allocation-free. The 5,000-tick native fixed and floating WBC
sentinels return zero infeasible ticks, repeat exactly outside timing, and keep
dynamics/contact maxima at `7.59e-14`/`5.20e-18` and
`5.14e-10`/`5.19e-11`, respectively. Their p99 values are `0.801 ms` and
`0.861 ms`.

## Independent reference comparison

The full r30 reference artifact is
[`../reference-r30/REFERENCE_COMPARISON.md`](../reference-r30/REFERENCE_COMPARISON.md).
It retains:

- Pinocchio 4.0 floating rigid-body products on Upkie and G1, with maximum
  errors `2.13e-14` and `1.71e-13`;
- a 100,000-step direct build of Upkie's pinned C++ `WheelBalancer`, with zero
  command/integral bit mismatches under upstream parameters and zero Rust
  hot-loop allocation;
- PlaCo kinematic reach, conflict, and data-backed CMU walking comparisons;
- complete per-step latency, jitter, tracking, CPU, RSS, fault, context-switch,
  GC, status, solver-work, and temporal-window tables.

Bonesaw's Python reference process uses roughly 42–50 MiB RSS across the
kinematic scenarios versus roughly 96–102 MiB for PlaCo, with zero Python GC
collections inside the measured calls. The data-backed walking gate remains
red for both implementations for different reasons: Bonesaw misses its 5 cm
foot RMS gate (`5.549 cm`), while PlaCo misses the 3 cm clearance gate
(`3.301 cm`).

## Remaining risks

- One of five unpinned toe-step repeats exceeds the 5 ms p99 deadline. A pinned,
  isolated hardware-counter run and longer tail campaign remain necessary
  before claiming deployment-grade jitter.
- Cached within-sweep energies are numerically stable and independently tested,
  but not bitwise equivalent to uncached summation. Regression gates must keep
  tracking behavior and typed status traces visible.
- The default 600-tick full-transfer policy still enters fallback/release and
  infeasible states with very high latency. The support-preview policy avoids
  those states but remains behaviorally red; neither is sustained-walking proof.
- The next dense-kernel target is clipped semantic-task pseudoinverse
  recomputation. Any reuse must carry an exact nullspace-validity invariant and
  must preserve the final `1e-8` original-row recheck.

## Reproduction

```bash
scripts/run-g1-synthetic-step.sh
scripts/run-reference-comparison.sh --output benchmarks/results/reference-r30
cargo test --workspace
cargo run --release -p bonesaw-tools --bin bonesaw-eval -- --ticks 5000 --json
```

Raw NPZ/JSON/report directories referenced above are retained beside this
report; no trace was filtered because it failed a gate.
