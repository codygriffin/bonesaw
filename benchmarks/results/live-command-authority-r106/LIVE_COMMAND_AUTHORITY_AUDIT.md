# Bonesaw adaptive command-authority stream · r106

## Outcome

**PASS.** The 13-signal WebSocket authority contract keeps base sampled geometry, bounded adaptive continuity, and final command selection independent. Pairs already certified by the cheap R105 bound skip interval work; only globally unresolved pairs evaluate analytic subinterval velocity extrema and deterministic midpoint primitive samples, to a fixed maximum depth of three.

The retained 60-frame, 80 mm torso-pull trace uses the guided browser pose while running a parallel policy-/physics-free command-admission query. The primary is admitted on 55 frames, the independently valid braking contingency is selected on 4, and 1 frames reject both; no frame is labelled realized plant motion.

| signal | retained minimum/max | interpretation |
|---|---|---|
| primary sampled primitive clearance | 19.70979816371976 | direct 21-point evidence |
| primary continuous lower bound | 19.601241344621968 | conservative half-interval guard |
| contingency continuous lower bound | 19.999871179336434 | independent brake certificate |
| primary limiting-pair speed | 0.7302697356895475 | m/s conservative maximum |
| primary midpoint pair queries | 0.0 | maximum bounded refinement work |
| primary unresolved leaves | 0.0 | maximum after depth-3 budget |
| raw WBC max query | 3700.5879999999993 | µs; separate 5 ms clock |
| command-admission max query | 2335.581 | µs; separate 5 ms clock |
| four command-query batch | 5395.740000000001 | µs; separate 20 ms clock |

Selections: `{'primary': 55, 'contingency': 4, 'rejected': 1}`. Flags: `{'0x000': 38, '0x001': 17, '0x080': 2, '0x081': 2, '0x180': 1}`. Minimum sampled pairs: `{12: 30, 70: 30}` / `{'left_thigh ↔ right_thigh': 30, 'chest ↔ left_forearm': 30}`. Continuous limiting pairs: `{12: 30, 70: 30}` / `{'left_thigh ↔ right_thigh': 30, 'chest ↔ left_forearm': 30}`; pair-speed range `0.000–0.730` m/s. Refinement maximum/mean: `0` / `0.00` midpoint pair queries; unresolved-leaf maximum: `0`; reached-depth counts: `{0: 60}`. First violating pairs: `{70: 5}` / `{'chest ↔ left_forearm': 5}`; first times: `{15000000: 1, 12000000: 1, 13000000: 1, 16000000: 1, 11000000: 1}`. Maximum dynamics/contact residuals remain `1.670e-09` / `4.899e-11`.

## Architectural finding

R105 made the broad certificate pair-consistent but its less-pessimistic selections changed persistent command state and exposed later sampled violations. R106 preserves those sampled failures as hard evidence: midpoint refinement may add a real first-violation pair/time, but can never erase one. On clear-but-uncertain intervals, subdivision spends bounded work to distinguish certified clearance from unresolved reserve. The represented sampled minimum is 19.710 mm; 55 frames are clear and 5 expose a sampled violation.

The adaptive continuous lower bound reaches 19.601 mm against the 20 mm requirement, while the braking certificate reaches 20.000 mm. The browser retains 120 ticks and now annotates current refinement work and unresolved leaves. Closest-feature rate bounds, mesh CCD, world collision, calibrated actuation, and plant response remain unavailable.
