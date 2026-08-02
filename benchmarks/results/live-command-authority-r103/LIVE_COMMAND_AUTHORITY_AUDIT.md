# Bonesaw live command-authority stream · r103

## Outcome

**PASS.** The WebSocket capability contract now carries 13 unique typed signals, including sampled command geometry, conservative continuous clearance, and final command selection. The browser renders these as three independent authority rows and does not aggregate them with WBC feasibility, actuator realization, thermal state, or plant response.

The retained 60-frame torso-pull trace uses the guided browser pose while running a parallel policy-/physics-free command-admission query. That separation is deliberate: a visually reachable pose is not relabeled as an executable or realized command.

| signal | retained minimum/max | interpretation |
|---|---|---|
| primary sampled proxy clearance | -247.1220959879042 | direct 21-point evidence |
| primary continuous lower bound | -247.19565287752906 | conservative half-interval guard |
| contingency continuous lower bound | -247.12209363602713 | independent brake certificate |
| raw WBC max query | 2246.3630000000003 | µs; separate 5 ms clock |
| command-admission max query | 2190.807 | µs; separate 5 ms clock |
| four command-query batch | 8601.173 | µs; separate 20 ms clock |

Selections: `{'rejected': 60}`. Flags: `{'0x180': 41, '0x181': 19}`. First sampled collision pairs: `{0: 60}`; first times: `{0: 60}`. Maximum dynamics/contact residuals remain `1.322e-09` / `4.411e-11`.

## Architectural finding

The current toy collision shapes use exact spheres but conservative bounding spheres for boxes and cylinders. The live standing/squat trace is therefore rejected at the geometry layer even while the raw floating WBC hard rows pass. This is useful honest evidence—the browser shows the command cannot be certified—but the roughly 247.1 mm proxy penetration is dominated by coarse proxy geometry, not an exact mesh/capsule collision claim.

The next geometry milestone should replace one-sphere box/cylinder bounds with tighter conservative primitive coverage or exact capsule/box distance plus an authored exclusion matrix. Until then, the red command rows are correctly labelled represented-proxy admission, and the guided pose remains visible without pretending that rejected authority was executed.
