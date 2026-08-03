# G1 rejected-Jacobi-pair cache · R294

> Arithmetic candidate **REJECTED** · exact semantic sentinel **PASS** · CPU-work gate **FAIL** · authority **CLOSED**.

R294 tests whether a rejected Jacobi column pair can skip its coupling dot on a later sweep when neither column nor its bit-exact re-anchored energy changed. The cache reuses caller-owned pseudoinverse output scratch, adds no solve-loop allocation, and is available only through `jacobi-rejected-pair-cache-experiment`.

## Motivation and exactness

Sampling the frozen support-transfer profile attributed 77.20% of samples to the flat pseudoinverse and concentrated the largest local instruction mass in the scalar coupling-dot dependency chain. The candidate stores a generation for each column and remembers both generations for a rejected pair. A rotation or bit-changing energy re-anchor invalidates every affected entry; generation overflow clears the fixed square cache.

All 317 `bonesaw-core` tests pass in both default and feature builds. The dedicated rejection-cache test observes a real cache hit while preserving every matrix, right-vector, energy, and sweep-count bit. A separate 100-tick native G1 sentinel has identical non-timing JSON with SHA-256 `f19ebca5aeba0b5b18c1f088eb39435cb28a1daa443a8fbea0f90288c4c7ecbd` in both builds.

## Pinned complete-process counter preflight

| pair | instructions control | candidate | delta | branches delta | native p50 control / candidate µs |
|---|---:|---:|---:|---:|---:|
| 1 | 29,428,934,122 | 29,886,665,017 | +1.55538% | +6.81245% | 1275.4 / 1264.7 |
| 2 | 29,428,938,770 | 29,886,665,736 | +1.55536% | +6.81239% | 1690.7 / 1267.5 |
| 3 | 29,428,934,414 | 29,886,663,873 | +1.55537% | +6.81252% | 1274.0 / 1288.0 |

The instruction regression is stable to 0.000014 percentage points across the three CPU-4-pinned AB/BA pairs. Mean branches rise 6.81245%. Pair 2 crosses a visible host-load transition, so the apparent mean cycle improvement and wall-time values are not decision evidence. Pairs 1 and 3 also have mixed p50/p99 direction; neither can overturn a deterministic work-count failure.

## Decision

Do not promote or run the expensive 2,317-tick Python corpus replay. The cache initialization, generation checks, invalidation, and extra branches cost more than the coupling dots they avoid in the native G1 workload. R293 remains the production default, the candidate remains opt-in for reproducible negative evidence, and the ordinary 5 ms support-transfer p99 gate remains open at the R293 median of 5.175 ms. No policy, physics, walking, contact, CUDA, or authority claim changes.
