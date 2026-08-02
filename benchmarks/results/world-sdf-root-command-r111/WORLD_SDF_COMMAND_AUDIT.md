# Bonesaw floating-root swept world-SDF command authority · r111

## Outcome

**PASS.** The CPU command controller now certifies the WBC primary trajectory and the independently generated brake trajectory against an immutable world SDF before either can gain command authority. The audit supplies states and desired accelerations directly; there is no policy, plant integration, contact response, or physics engine.

The independent NumPy oracle reconstructed every actuator quintic and both floating-root prediction witnesses for 321 transactions. Maximum sampled-distance, first-violation-time, continuity-lower-bound, distance-rate-bound, and root-prediction errors were `2.776e-16` m, `0` ns, `2.776e-16` m, `3.997e-15` m/s, and `1.509e-18`. The least dense-truth-minus-certificate gap was `6.103e-05` m; a negative value would invalidate the certificate.

The discriminating root-motion fixture starts at joint position `-0.5770` rad with root x velocity `-0.275` m/s. Freezing the root would report `23.000` mm clearance, but the explicit primary root sweep reaches `17.500` mm and first violates at `11000000` ns. The independently swept brake remains at `20.250` mm, so authority transfers to Contingency.

| plan | sampled collision | continuous-only failure | certified |
|---|---|---|---|
| primary | 3 | 0 | 318 |
| contingency | 1 | 0 | 320 |

Selections were `{'primary': 318, 'contingency': 2, 'rejected': 1}`. Sampled geometry, between-sample continuity, and unknown-space faults have separate bits and witnesses; they are not collapsed into one health score.

## Bounded adaptive refinement

The deterministic fixture `q=-0.5550`, `v=-0.7500` m/s, requested acceleration `-60.0` m/s² demonstrates useful refinement. The global 1 ms certificate was `0.019096` m and withheld the primary. Depth-bounded midpoint refinement raised the proven lower bound to `0.020106` m against dense truth `0.020250` m, using 23 leaves, 3 added probe samples, depth 3, and no unresolved leaves.

This is continuous-time command admission with a fixed work budget. A certificate that remains below the 0.020 m threshold is withheld; the controller does not wait extra servo steps hoping the pose becomes feasible.

## Unknown-space authority

With Reject semantics, the primary left the known volume at `16000000` ns, set the dedicated unknown bit, and transferred authority to the known-clear brake (`selection=1`). With OccupiedBoundary semantics, the field boundary itself is occupied: the 0.2 m sphere already overlaps it, so both primary and brake are measured collisions (`source=1`) and the selector rejects (`selection=2`). Unknown and occupied space therefore remain distinguishable while both fail closed.

## Timing and allocation

| transaction | p50 / p99 / max µs | alloc calls / bytes | replay |
|---|---|---|---|
| WBC + primary/brake + two swept-world certificates | 17.262 / 23.945 / 28.254 | 0 / 0 | True |

Timing covers Rust WBC solve, actuator mapping, exact primary/brake construction, 21-knot scans for both plans, bounded continuous certification, and selection. Python reset/statistics/report work is outside the timed region.

## Example authority stack

1. **Intent/task rows** — desired generalized and end-effector accelerations; may be relaxed by their authored priority.
2. **Local world viability (HOCBF)** — state-local acceleration authority near the SDF wall; separately reports margin and residual.
3. **Primary sampled world geometry** — exact 1 ms command-knot evidence with probe, body, source, and first-violation time.
4. **Primary continuous world clearance** — Lipschitz/rate certificate plus bounded refinement work; can withhold an otherwise sampled-clear primary.
5. **Contingency sampled + continuous world clearance** — independently witnesses the brake; never inherits primary validity.
6. **Joint, actuator, support, effort, thermal, and solver-budget authority** — orthogonal typed limits remain visible alongside world geometry.
7. **Command selector** — primary, contingency, or reject. Only a valid plan is emitted; later plant tracking and reliability remain separate evidence.

## Contract and remaining boundary

The field epoch and probe set are immutable for the controller instance. Sphere covers conservatively represent authored robot collision geometry; this is not arbitrary mesh CCD. The rate proof composes joint motion with an explicit local-SE(3) floating-root prediction in smooth `control_world`; root linear travel and a conservative authored-reach angular bound are included at every knot and adaptive midpoint. This witness is not an executable root command or a plant rollout. Moving fields, external-obstacle epochs, calibrated actuator/thermal response, and mesh validation remain explicit later milestones.
