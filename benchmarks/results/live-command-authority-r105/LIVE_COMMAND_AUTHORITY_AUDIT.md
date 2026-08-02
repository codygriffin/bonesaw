# Bonesaw pair-tight command-authority stream · r105

## Outcome

**PASS.** The 13-signal WebSocket authority contract now forms every between-sample clearance certificate from one physical primitive pair: that pair's sampled minimum minus that same pair's conservative relative-speed bound over half a sample interval. Sampled geometry, pair-tight continuity, and final selection remain three independent rows rather than an aggregate health score.

The retained 60-frame, 80 mm torso-pull trace uses the guided browser pose while running a parallel policy-/physics-free command-admission query. The primary is admitted on 52 frames, the independently valid braking contingency is selected on 5, and 3 frames reject both; no frame is labelled realized plant motion.

| signal | retained minimum/max | interpretation |
|---|---|---|
| primary sampled primitive clearance | 19.70979816371976 | direct 21-point evidence |
| primary continuous lower bound | 19.601241344621968 | conservative half-interval guard |
| contingency continuous lower bound | 19.999414062767634 | independent brake certificate |
| primary limiting-pair speed | 0.7302697356895475 | m/s conservative maximum |
| raw WBC max query | 2002.0720000000001 | µs; separate 5 ms clock |
| command-admission max query | 1242.4370000000001 | µs; separate 5 ms clock |
| four command-query batch | 4510.242 | µs; separate 20 ms clock |

Selections: `{'primary': 52, 'contingency': 5, 'rejected': 3}`. Flags: `{'0x000': 35, '0x001': 17, '0x080': 4, '0x201': 1, '0x480': 2, '0x481': 1}`. Minimum sampled pairs: `{12: 30, 70: 30}` / `{'left_thigh ↔ right_thigh': 30, 'chest ↔ left_forearm': 30}`. Continuous limiting pairs: `{12: 30, 70: 30}` / `{'left_thigh ↔ right_thigh': 30, 'chest ↔ left_forearm': 30}`; pair-speed range `0.000–0.730` m/s. First violating pairs: `{70: 7}` / `{'chest ↔ left_forearm': 7}`; first times: `{15000000: 1, 13000000: 2, 18000000: 1, 12000000: 2, 17000000: 1}`. Maximum dynamics/contact residuals remain `1.670e-09` / `4.899e-11`.

## Architectural finding

R104 used tight primitives but formed its continuous reserve from the global sampled minimum and the global maximum speed, even when those witnesses belonged to different pairs. R105 retains the same deterministic primitive sweep and topology-derived exclusions, stores a sampled minimum for every validation pair in caller-owned scratch, and computes each pair's reserve independently before selecting the limiting lower bound. The sampled primitive minimum is 19.710 mm; 53 frames are clear on the 1 ms grid and 7 expose a sampled violation.

The pair-tight continuous lower bound reaches 19.601 mm against the 20 mm requirement, while the braking certificate reaches 19.999 mm. The browser retains 120 ticks of sampled clearance, continuous reserve, requirement, and Primary/Contingency/Rejected selection so authority transitions remain inspectable. Pair-specific adaptive interval subdivision, mesh CCD, world collision, calibrated actuation, and plant response remain unavailable.
