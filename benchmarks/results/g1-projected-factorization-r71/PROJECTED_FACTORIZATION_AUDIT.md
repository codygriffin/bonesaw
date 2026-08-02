# Bonesaw projected-solve factorization audit · r70–r71

## Decision · REJECT BOTH

R70 replaces well-conditioned, full-row-rank wide task SVDs with a guarded row-Gram Cholesky solve. It passes all 43 admission gates, removes 47,918 reported Jacobi sweeps (39.27%) and 15 downstream inverse calls, and remains allocation-free. It nevertheless changes 22/56 non-timing fields, changes the monotonic clipping path, and fails the established r63 replay contract. The candidate is not production.

The independent r64 Pinocchio/NumPy oracle still passes 6/6 on the r70 fixed-effort queries. That is useful negative evidence: the row-Gram solution is a valid state-local optimizer on the sampled equality problems, but a different floating-point factorization is not equivalent to Bonesaw's deterministic bounded active-set trajectory.

R71 specializes only a one-row task projection and reproduces the established kernel bit-for-bit. All 56 non-timing arrays are exact in the full corpus and in every one of five pinned A/B pairs. The retained-instruction median changes by +0.000103%; paired changes span -0.000654% to +0.000360%. Since the candidate does not reduce instructions in every pair, it is a workload/noise-floor rejection rather than a semantic failure.

> Neither result weakens a gate. Production remains r66 feasible-set row compaction. GPU batching remains deferred.

## Candidate matrix

| candidate | semantic result | work result | reference result | decision |
|---|---|---|---|---|
| r70 guarded row Gram | 22/56 fields changed; 19 r63 replay fields fail | -39.27% sweeps; -15 inverse calls | 43/43 admission; independent r64 6/6 | REJECT |
| r71 exact rank one | 56/56 exact in corpus and five pinned pairs | +0.000103% median instructions; mixed pair signs | 43/43 admission; zero allocation | REJECT |

## R70 semantic deltas

| field | changed samples | maximum absolute delta | r63 replay contract |
|---|---|---|---|
| actuator torque | 53290 | 18.68 | FAIL |
| clipped steps | 594 | 11 | FAIL |
| clipped steps by priority | 699 | 11 | FAIL |
| contact normal force | 14549 | 70.811 | FAIL |
| contact residual | 2316 | 1.50165e-14 | not replay-gating |
| dynamics residual | 2254 | 3.00204e-13 | not replay-gating |
| generalized acceleration | 67143 | 125.517 | FAIL |
| limiting actuator | 20 | 10 | FAIL |
| maximum constraint violation | 2254 | 3.00204e-13 | not replay-gating |
| maximum torque utilization | 2316 | 0.27242 | FAIL |
| minimum bound margin | 1017 | 3.24746 | FAIL |
| minimum friction margin | 2180 | 10.778 | FAIL |
| minimum support margin | 2227 | 0.0172644 | FAIL |
| minimum torque headroom | 2316 | 83.3604 | FAIL |
| minimum torque margin | 2317 | 11.6966 | FAIL |
| task clipped | 165 | 1 | FAIL |
| task jacobi sweeps | 2312 | 91 | FAIL |
| task jacobi sweeps by priority | 6457 | 64 | FAIL |
| task pseudoinverse calls | 493 | 12 | FAIL |
| task pseudoinverse calls by priority | 558 | 12 | FAIL |
| task rms | 18038 | 42.2794 | FAIL |
| witness acceleration rms | 2316 | 37.6519 | FAIL |

## R71 pinned counter evidence

| signal | candidate − control median |
|---|---|
| retired instructions | +0.000103% |
| process CPU | +0.418% |
| p50 tick | +0.563% |
| p99 tick | +0.309% |

## Interpretation

- Full-row-rank algebra is not enough to preserve this solver's bounded semantics. Floating-point changes before a hard-limit comparison can select a different monotonic active set and produce a different lower-priority optimum.
- Independent reference agreement and replay equivalence answer different questions. R70 demonstrates why both are required.
- An arithmetic-exact specialization is promotable only when the retained workload contains enough matching shapes to reduce stable instructions. R71 does not.
- The next CPU candidate should preserve the established rotation and comparison order while reducing memory traffic inside multi-row Jacobi sweeps, or introduce a separately versioned solver semantics rather than silently replacing the current one.

## Gates

### R70

- PASS `control_passes_all_43_admission_gates`
- PASS `candidate_passes_all_43_admission_gates`
- PASS `candidate_keeps_zero_measured_rust_allocations`
- FAIL `r63_replay_contract_passes`
- PASS `r64_independent_pinocchio_numpy_oracle_passes`
- FAIL `all_56_non_timing_arrays_are_bit_exact`

### R71

- PASS `candidate_passes_all_43_admission_gates`
- PASS `all_56_non_timing_arrays_are_bit_exact`
- PASS `all_five_pinned_pairs_are_bit_exact`
- FAIL `candidate_reduces_median_retired_instructions`
- FAIL `candidate_reduces_instructions_in_every_pair`
- PASS `candidate_keeps_zero_measured_rust_allocations`
