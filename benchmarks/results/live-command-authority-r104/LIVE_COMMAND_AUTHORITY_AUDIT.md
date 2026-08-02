# Bonesaw primitive command-authority stream · r104

## Outcome

**PASS.** The 13-signal WebSocket authority contract now evaluates command geometry with one tight primitive per authored shape: exact spheres, conservative capsules around cylinders, authored capsules, and oriented boxes. Sampled geometry, the between-sample certificate, and final selection remain three independent rows rather than an aggregate health score.

The retained 60-frame, 80 mm torso-pull trace uses the guided browser pose while running a parallel policy-/physics-free command-admission query. A collision-aware reference posture places the arms 0.25 rad outward. The primary is admitted on 54 frames and the independently valid braking contingency is selected on 6; no frame is labelled realized plant motion.

| signal | retained minimum/max | interpretation |
|---|---|---|
| primary sampled primitive clearance | 20.105758036113766 | direct 21-point evidence |
| primary continuous lower bound | 19.910641778638354 | conservative half-interval guard |
| contingency continuous lower bound | 20.02935271023271 | independent brake certificate |
| raw WBC max query | 3839.176 | µs; separate 5 ms clock |
| command-admission max query | 1318.716 | µs; separate 5 ms clock |
| four command-query batch | 5079.503000000001 | µs; separate 20 ms clock |

Selections: `{'primary': 54, 'contingency': 6}`. Flags: `{'0x000': 36, '0x001': 18, '0x200': 5, '0x201': 1}`. Minimum sampled pairs: `{12: 30, 70: 30}` / `{'left_thigh ↔ right_thigh': 30, 'chest ↔ left_forearm': 30}`. First violating pairs: `{}` / `{}`; first times: `{}`. Maximum dynamics/contact residuals remain `1.670e-09` / `4.899e-11`.

## Architectural finding

R103's one-sphere bounds rejected all 60 frames with 247.1 mm represented penetration. R104 removes that modeling artifact without adding a permissive body-pair exclusion: collisionless revolute/fixed carrier links collapse into physical adjacency, prismatic carriers remain testable, and the sampled primitive minimum is 20.106 mm. All 60 frames are clear on the 1 ms grid.

The continuous lower bound reaches 19.911 mm, just below the 20 mm requirement, while the braking certificate stays at or above 20.029 mm. Those contingency selections demonstrate why sampled clearance and continuous authority must remain distinct. Mesh CCD, world collision, calibrated actuation, and plant response remain unavailable.
