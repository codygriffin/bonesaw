# Bonesaw r29 hybrid feasibility A/B

## Decision

Revision r29 adopts an eight-sweep Dykstra prefix followed, only when the hard
polytope is still outside the near-feasible band, by a preallocated Euclidean
active-set projection. Active-set work is capped at 64 iterations. The hard
acceptance tolerance remains `1e-8`; no contact, tracking, or deadline gate was
relaxed.

The change is accepted because the 260-tick moving-liftoff profile becomes
combined green, the 160-tick toe-step remains functionally green, and the
600-tick CMU stress no longer reports a primal-infeasible tick. The CMU profile
is still behaviorally red and the toe-step still misses 5 ms p99, so neither is
presented as sustained-walking success.

## Algorithms compared

| Build | Feasibility seed |
|---|---|
| r28 control | Dykstra until `1e-8`, near-feasible polish below `1e-3`, otherwise up to `max(512, 32 × rows)` sweeps |
| r29 hybrid | identical Dykstra order for at most 8 sweeps; immutable-origin primal/dual active set after that; 64-iteration cap; final check against the original bounds and rows |

Both paths use cached row norms, skip only exactly-zero transpose updates, use
caller-owned flat buffers, and expose solver-work counters through fixed NumPy
arrays. Python constructs references, executes one Rust batch call, retains raw
arrays, and generates the scenario reports.

## G1 scenario results

| Profile | Build | Functional | 5 ms p99 | p50 / p95 / p99 / max | Max Dykstra work | Max active-set work |
|---|---|---:|---:|---:|---:|---:|
| 160-tick toe-step | r28 control | PASS | FAIL | 2.483 / 20.119 / 23.549 / 29.359 ms | 755 sweeps / 176,670 rows | 1 iteration / 1 pseudoinverse |
| 160-tick toe-step | r29 hybrid | PASS | FAIL | 2.499 / 4.825 / 6.293 / 7.252 ms | 8 sweeps / 1,872 rows | 2 iterations / 1 pseudoinverse |
| 260-tick moving liftoff | r28 control | PASS | FAIL | 2.449 / 9.174 / 11.175 / 12.121 ms | 493 sweeps / 109,446 rows | 1 iteration / 1 pseudoinverse |
| 260-tick moving liftoff | r29 hybrid | PASS | PASS | 2.459 / 3.744 / 4.122 / 4.425 ms | 8 sweeps / 1,776 rows | 3 iterations / 2 pseudoinverses |
| 600-tick CMU transfer stress | r28 control | FAIL | FAIL | 10.657 / 173.330 / 177.348 / 201.688 ms | 6,720 sweeps / 1,411,200 rows | 1 iteration / 1 pseudoinverse |
| 600-tick CMU transfer stress | r29 hybrid | FAIL | FAIL | 3.080 / 8.922 / 13.547 / 116.484 ms | 8 sweeps / 1,872 rows | 14 iterations / 13 pseudoinverses |

The single 116 ms CMU maximum is retained rather than trimmed. That stress
trace is not a deadline pass.

## Tracking and hard residuals

| Profile | Build | Root / stance / swing RMS | Max root rotation | Dynamics / contact residual | Infeasible / fallback / release |
|---|---|---:|---:|---:|---:|
| Toe-step | r28 | 4.298 / 0.095 / 0.639 cm | 1.585° | `1.21e-9` / `4.62e-11` | 0 / 0 / 0 |
| Toe-step | r29 | 4.191 / 0.240 / 0.639 cm | 1.413° | `1.21e-9` / `4.62e-11` | 0 / 0 / 0 |
| Liftoff | r28 | 0.706 / 0.00035 / 0.326 cm | 4.390° | `1.22e-9` / `5.15e-11` | 0 / 0 / 0 |
| Liftoff | r29 | 0.763 / 0.00031 / 0.388 cm | 3.713° | `1.22e-9` / `5.15e-11` | 0 / 0 / 0 |
| CMU stress | r28 | 100.205 / 97.187 / 87.108 cm | 179.360° | `1.28e-9` / `7.30e-11` | 126 / 22 / 49 |
| CMU stress | r29 | 126.196 / 95.801 / 137.885 cm | 179.407° | `8.15e-9` / `1.76e-10` | 0 / 160 / 54 |

The hybrid is intentionally not byte-equivalent to r28: it computes a
different feasible point after the bounded prefix. Toe-step status codes are
identical on all 160 ticks; maximum state deltas are 2.56 mm root translation,
0.0393 rad configuration, and 1.872 rad/s velocity. Liftoff status codes are
also identical on all 260 ticks; maximum deltas are 4.60 mm, 0.0807 rad, and
4.663 rad/s. The unstable CMU stress diverges materially (185 status ticks), so
its red tracking metrics cannot be used as a parity claim.

## Repeat timing, CPU, jitter, and memory

Five unpinned toe-step runs were retained for each build. r28 p99 median/range
is 24.162 ms / 20.991–30.503 ms. r29 p99 median/range is 6.349 ms /
6.246–8.265 ms, a 73.7% median reduction. Median whole-call thread CPU falls
from 806.149 ms to 475.791 ms (41.0%). All 37 r29 non-timing arrays are
byte-for-byte identical across the five repeats.

For the displayed single toe-step pair, p99 absolute latency jitter falls from
12.142 ms to 2.323 ms and thread CPU from 806.149 ms to 464.494 ms. RSS delta is
236 versus 228 KiB, peak RSS is 41.72 versus 41.79 MiB, and Python GC
collections are zero in both. The RSS differences are allocator/measurement
noise, not an optimization claim. Native allocation sentinels remain the
authority for the Rust hot loop.

Execution-over-time deciles, per-status latency, task-pseudoinverse work,
faults, context switches, forces, support phases, and every raw state sample are
retained in each linked scenario artifact:

- [r28 toe-step repeat](../g1-synthetic-step-r28-repeat-1/FLOATING_WALK_CORPUS.md)
- [r29 toe-step latest](../g1-synthetic-step-latest/FLOATING_WALK_CORPUS.md)
- [r28 liftoff control](../floating-g1-liftoff-r29-r28-control/FLOATING_WALK_CORPUS.md)
- [r29 liftoff latest](../floating-g1-liftoff-latest/FLOATING_WALK_CORPUS.md)
- [r28 CMU control](../g1-cmu-transfer-r29-r28-control/FLOATING_WALK_CORPUS.md)
- [r29 CMU stress](../g1-cmu-transfer-r29-hybrid-final/FLOATING_WALK_CORPUS.md)

## Rejected thresholds

- Active set from the cold equality seed: p99 6.15 ms on the toe-step, but root
  RMS becomes 5.204 cm and fails the functional gate.
- Two Dykstra sweeps: toe-step stays green and CMU is fast, but moving-liftoff
  root rotation becomes 5.284° and fails its 5° gate.
- Four Dykstra sweeps: moving liftoff is green, but the CMU trajectory enters a
  long active-row add/remove cycle. The run was interrupted and this threshold
  was rejected.
- Eight sweeps: all bounded active sets converge within 14 iterations on the
  three validation profiles. The separate 64-iteration kernel cap prevents an
  unseen cycle from consuming unbounded control time.

## Remaining work

The toe-step plateau after r29 is now dominated by semantic task
pseudoinverses/clipped-step recomputation rather than hundreds of thousands of
feasibility-row visits. The next CPU optimization must profile that kernel and
preserve the current functional gates. The CMU reference/controller policy
also remains unsuitable for sustained-transfer acceptance; removing false
primal rejection does not make its metre-scale tracking error acceptable.
