# Bonesaw task/contact emission mirror · r79

> PROMOTE CPU ROW EMISSION ONLY. This does not admit the hierarchical solver or CUDA; no device claim is made.

R79 lowers fixed point attractors and three-axis contact locks over the r78 geometric products. Stable ID, point-query provenance, strict priority, weight, bandwidth, activation, Jacobian, physical acceleration, Jdot-v compensation, and right-hand side remain explicit fixed-shape data.

## Admission decision

| Gate | Evidence | Decision |
|---|---|---|
| Descriptor semantics | stable IDs + query slots + priority/weight/bandwidth | PASS |
| Task algebra | independent NumPy critical-damping composition | PASS |
| Contact algebra | J qdd = desired - Jdot-v rows | PASS |
| Inactive masks | all fixed row storage exactly zero | PASS |
| Repeat/reindex/isolation | bitwise exact | PASS |
| Measured Rust allocations | 0 calls · 0 B | PASS |
| Shared-qdd feasibility | reported per state, never folded into row correctness | DIAGNOSTIC |
| Hierarchical solve / CUDA | not in this manifest / device unavailable | NOT RUN |

## Independent composition oracle

| Model | States | Largest task algebra error | Largest contact error | Shared-qdd residual p50/p99 | Result |
|---|---|---|---|---|---|
| toy_humanoid | 16 | 1.526e-05 | 0.000e+00 | 6.164e-13/1.932e-12 | PASS |
| upkie | 16 | 7.629e-06 | 0.000e+00 | 3.299e-01/5.479e+00 | PASS |

The oracle is deliberately staged: Pinocchio/finite-difference admission of position, J, Jv, and Jdot-v happened in r78; r79 independently composes target jets, critical damping, masks, and bias subtraction in NumPy. A least-squares generalized acceleration is then fit to every simultaneously active task and contact row only to expose conflict. It is not a pass condition because a correct WBC must represent infeasible requests before a bounded solver reports compromise.

## Example authority stack for architectural review

| Layer | Concrete r79 example | Meaning |
|---|---|---|
| Invariant | left/right contact-lock rows 7951/7952 | material point acceleration equalities |
| Viability | torso point attractor 7901 · 1.5 Hz · weight 1.5 | stability-preserving intent with explicit J/Jdot-v |
| Intent | handle point attractor 7902 · 2.25 Hz · weight 0.75 | operator target jet |
| Preference | not declared in this two-task r79 plan | absence remains distinct from zero residual |
| Style | not declared in this two-task r79 plan | terminal refinement remains a later manifest row |
| Feasibility | shared-qdd residual distribution | continuous incompatibility evidence, not an aggregate score |
| Resources | joint/effort/thermal layers remain separate | row emission cannot promise realizability |
| Backend | CpuMirrorF32 admitted; CUDA unavailable | implementation conformance is not physical authority |

## Exactness and isolation

| Invariant | Result |
|---|---|
| Repeat | True |
| Permutation / padding / 7+10 chunking | True / True / True |
| Active NaN target | agent 8 zeroed and typed InvalidInput; every neighbor unchanged |

## Host timing · Upkie two tasks + two contacts

| Agents | Emission p50/p99 | Full pipeline p50/p99 | Full/agent p50 | Alloc |
|---|---|---|---|---|
| 1 | 0.832/1.353 µs | 51.627/75.026 µs | 51627.5 ns | 0 / 0 B |
| 32 | 8.897/13.748 µs | 1423.493/1485.552 µs | 44484.1 ns | 0 / 0 B |
| 256 | 77.056/86.544 µs | 11665.609/11874.984 µs | 45568.8 ns | 0 / 0 B |

All timing samples are retained. Stage timers exclude NumPy↔SoA copies. Weight and priority remain descriptor metadata for the later hierarchical solver; emitted J and RHS stay in physical units.

## Deferred boundary

The next slice must assemble inequalities and solve the emitted hierarchy under a bounded-work contract, while reporting hard infeasibility, task compromise, iteration exhaustion, and physical resource pressure separately. CudaMirrorF32 must later reproduce this exact manifest on real hardware and pass D1/D2/D3; no CPU fallback may claim device execution.
