# Bonesaw robust floating-root swept world-SDF authority · r112

## Outcome

**PASS.** The CPU command controller now certifies the WBC primary trajectory and independently generated brake against an immutable world SDF after robustifying both by a deterministic floating-root forecast-error envelope. The audit supplies states and desired accelerations directly; there is no policy, plant integration, contact response, physics engine, covariance, or probability claim.

The independent NumPy oracle reconstructed every actuator quintic, both floating-root prediction witnesses, and their translation/attitude error erosion for 321 transactions. Maximum sampled-distance, first-violation-time, continuity-lower-bound, distance-rate-bound, root-prediction, and erosion errors were `2.776e-16` m, `0` ns, `2.776e-16` m, `3.997e-15` m/s, `1.509e-18`, and `3.036e-18` m. The least dense-truth-minus-certificate gap was `9.673e-05` m; a negative value would invalidate the certificate.

The discriminating root-motion fixture starts at joint position `-0.5750` rad with root x velocity `-0.275` m/s. Freezing the root would report `25.000` mm clearance, but the explicit primary root sweep reaches `17.376` mm and first violates at `13000000` ns. The independently swept brake remains at `20.126` mm, so authority transfers to Contingency.

The prediction-error-only fixture holds the nominal root still. With zero declared error it has `21.000` mm and selects Primary; the configured envelope erodes at most `2.124` mm, reducing robust clearance to `18.876` mm and withholding the command at `2000000` ns. This is continuous loss of authority from explicit forecast quality, not a delayed timeout.

| plan | sampled collision | continuous-only failure | certified |
|---|---|---|---|
| primary | 3 | 1 | 317 |
| contingency | 1 | 0 | 320 |

Selections were `{'primary': 317, 'contingency': 3, 'rejected': 1}`. Sampled geometry, between-sample continuity, and unknown-space faults have separate bits and witnesses; they are not collapsed into one health score.

## Bounded adaptive refinement

The deterministic fixture `q=-0.5550`, `v=-0.3000` m/s, requested acceleration `-100.0` m/s² demonstrates useful refinement. The global 1 ms certificate was `0.019208` m and withheld the primary. Depth-bounded midpoint refinement raised the proven lower bound to `0.020271` m against dense truth `0.020626` m, using 22 leaves, 2 added probe samples, depth 2, and no unresolved leaves.

This is continuous-time command admission with a fixed work budget. A certificate that remains below the 0.020 m threshold is withheld; the controller does not wait extra servo steps hoping the pose becomes feasible.

## Unknown-space authority

With Reject semantics, the primary left the known volume at `16000000` ns, set the dedicated unknown bit, and transferred authority to the known-clear brake (`selection=1`). With OccupiedBoundary semantics, the field boundary itself is occupied: the 0.2 m sphere already overlaps it, so both primary and brake are measured collisions (`source=1`) and the selector rejects (`selection=2`). Unknown and occupied space therefore remain distinguishable while both fail closed.

## Timing and allocation

| transaction | p50 / p99 / max µs | alloc calls / bytes | replay |
|---|---|---|---|
| WBC + primary/brake + two swept-world certificates | 18.105 / 26.912 / 44.584 | 0 / 0 | True |

Timing covers Rust WBC solve, actuator mapping, exact primary/brake construction, 21-knot scans for both plans, bounded continuous certification, and selection. Python reset/statistics/report work is outside the timed region.

## Example authority stack

1. **Intent/task rows** — desired generalized and end-effector accelerations; may be relaxed by their authored priority.
2. **Local world viability (HOCBF)** — state-local acceleration authority near the SDF wall; separately reports margin and residual.
3. **Floating-root forecast** — primary/brake local-SE(3) paths in smooth `control_world`; prediction evidence, never base actuation.
4. **Root forecast error** — deterministic translation/attitude radii and growth rates become probe-specific clearance erosion; separate from the nominal path.
5. **Primary sampled world geometry** — exact 1 ms robust command-knot evidence with probe, body, source, and first-violation time.
6. **Primary continuous world clearance** — motion plus error-growth Lipschitz/rate certificate and bounded refinement work.
7. **Contingency sampled + continuous world clearance** — independently witnesses the brake; never inherits primary validity.
8. **Joint, actuator, support, effort, thermal, and solver-budget authority** — orthogonal typed limits remain visible alongside world geometry.
9. **Command selector** — primary, contingency, or reject. Only a valid plan is emitted; later plant tracking and reliability remain separate evidence.

## Contract and remaining boundary

The field epoch and probe set are immutable for the controller instance. Sphere covers conservatively represent authored robot collision geometry; this is not arbitrary mesh CCD. The rate proof composes joint motion, an explicit local-SE(3) floating-root prediction in smooth `control_world`, and deterministic radial prediction-error growth. Root linear travel, conservative authored angular reach, and pose-error erosion participate at every knot and adaptive midpoint. The bounds are externally declared worst-case evidence, not learned covariance, an executable root command, or a plant rollout. Moving fields, external-obstacle epochs, estimator calibration, actuator/thermal response, and mesh validation remain explicit later milestones.
