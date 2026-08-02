# Bonesaw fixed-level CPU mirror solve · r97

## Outcome

**PASS.** R97 freezes the allocation-free `CpuMirrorF32` hierarchy that the future CUDA solve must reproduce: stable row order, 64 hard-projection sweeps, 32 row-action sweeps per active soft level, two hard/prior-level restoration sweeps per task-row update, f32 FMA arithmetic, and no early exit. Its semantics are explicitly **FixedLevelApproximate**. The deployed CPU reference remains **StrictLexicographic f64**.

This is a solver-algorithm admission, not CUDA execution, actuator authority, a plant rollout, or proof that a requested pose is physically realizable. CUDA solve and CUDA Graph remain **NOT IMPLEMENTED**.

## Example authority stack shown in the architecture review

| layer | concrete r97 example | independent witness |
|---|---|---|
| Invariant | NormalPoint z contact rows | final hard residual |
| Viability | point x acceleration +1 | level RMS + preservation drift |
| Intent | conflicting point x acceleration -1 | lower-level compromise only |
| Resources | generalized-acceleration bounds | minimum bound margin + clipping |
| Solver budget | 64 hard + 32/level fixed sweeps | initial/best/final residual + MaxIterations |
| Backend | CpuExactF64 vs CpuMirrorF32 | separate semantics and D3 |
| Admission | command vs retained candidate | budget exhaustion keeps command zero |

The stack is intentionally not collapsed into a health score. Invariant failure blocks admission; Viability is protected from lower Intent; bounds are a separate resource surface; fixed compute budget is separate from mathematical infeasibility; backend agreement is separate from physical authority; and an unadmitted candidate is separate from an executable command.

## Continuous budget exhaustion, not a timeout cliff

| signal | result |
|---|---|
| mirror status | Max Iterations |
| strict reference status | Max Iterations |
| hard residual, initial → best → final | 1.25 → 1 → 1 |
| best candidate retained and finite | True |
| admitted command exactly zero | True |
| infeasibility claim | NONE — fixed work exhaustion does not prove primal infeasibility |

The contradictory two-contact agent always consumes the fixed 64-sweep hard budget. Diagnostics retain how far the solve got. The candidate is queryable for observability and later warm-start research, but it cannot cross the command boundary.

## Conformance gates

| gate | result |
|---|---|
| 500-call complete-output byte replay (D1) | True |
| compatible mirror vs strict command maximum absolute delta (D3) | 0.0 |
| D3 / hard / preservation tolerance | 0.0005 |
| maximum admitted hard violation | 0.0 |
| maximum priority-preservation drift | 0.0 |
| typed MaxIterations / InvalidProblem / InvalidInput | True |
| invalid-agent neighbor isolation unit gate | True |
| inactive padded lanes exactly zero | True |
| hot execute allocation unit gate | True |

## Per-agent semantic corpus

Level RMS order is Invariant / Viability / Intent / Preference / Style. Cartesian RMS includes all three task axes, including a deliberate z-axis conflict between the Viability point target and the invariant normal contact.

| agent | CpuMirrorF32 | CpuExactF64 | hard initial → best → final | level RMS | task sweeps |
|---|---|---|---|---|---|
| 0 | Solved With Residual | Solved | 0.25 → 0 → 0 | 0/0.1443/1.164/0/0 | 64 |
| 1 | Max Iterations | Max Iterations | 1.25 → 1 → 1 | 0/0/0/0/0 | 0 |
| 2 | Invalid Problem | Invalid Problem | 0 → 0 → 0 | 0/0/0/0/0 | 0 |
| 3 | Invalid Input | Invalid Input | 0 → 0 → 0 | 0/0/0/0/0 | 0 |
| 4 | Max Iterations | Max Iterations | 0.25 → 0.15 → 0.15 | 0/0/0/0/0 | 0 |
| 5 | Solved | Solved | 0 → 0 → 0 | 0/0/0/0/0 | 0 |
| 6 | Solved With Residual | Solved | 0.25 → 0 → 0 | 0/0.1443/1.164/0/0 | 64 |

The corpus covers compatible conflicting soft levels, contradictory hard rows, invalid bounds, invalid emitted data, a hard-row/bound conflict, an empty problem, and an isolated compatible neighbor. The exact solver agrees on executable commands for the compatible agents in this fixture; status names remain backend-specific rather than being coerced into one enum.

## Timing and fixed memory

| backend · seven-agent batch | mean µs | p50 µs | p95 µs | p99 µs | max µs | p99−p50 µs |
|---|---|---|---|---|---|---|
| CpuMirrorF32 fixed-level | 861.580 | 842.451 | 1034.864 | 1291.818 | 1666.566 | 449.367 |
| CpuExactF64 strict | 1779.341 | 1765.864 | 1785.922 | 1931.867 | 3079.152 | 166.003 |

| preallocated mirror boundary | bytes |
|---|---|
| solver scratch | 13974 |
| bounds input | 6144 |
| command + candidate + diagnostics output | 9248 |
| total | 29366 |

Timing is retained raw as a 500-sample same-process release-build distribution, per complete seven-agent batch. It is descriptive on this host, not a universal speed claim and not CUDA evidence. The fixed-memory count covers owned solve input, output, and scratch buffers; upstream model-product and row-emission storage is excluded and labeled separately in earlier reports.

## Frozen identity and next gate

| field | value |
|---|---|
| semantics | fixed_level_approximate |
| algorithm + descriptor SHA-256 | 695c600907401e9c264d29b02fe8b8bc6a16b58b8c7dfed3454edc2443a2fd19 |
| generalized coordinates | 24 |
| capacity / stride | 17 / 32 |
| hidden CPU fallback | False |
| CUDA solve | not_implemented |
| CUDA Graph | not_implemented |

The next implementation slice may port this exact fingerprinted algorithm to a fixed-buffer CUDA kernel. Device admission still requires independent compiler/runtime presence, D1 repeat, D3 against this mirror, permutation/chunking/padding/isolation/error-injection gates, memory scaling, timing, direct-versus-Graph equivalence, and no-fallback proof. It may not inherit this CPU pass.
