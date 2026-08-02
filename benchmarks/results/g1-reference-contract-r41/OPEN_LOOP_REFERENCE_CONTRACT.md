# Bonesaw R41 open-loop G1 reference contract

This eval intentionally runs **no controller policy and no physics rollout**. It scores only authored root, CoM, foot, and contact-schedule arrays. Tracked state, effective/retimed references, solve status, contact forces, residuals, solver work, and latency are forbidden inputs.

The centroidal check is a conditional necessary—not sufficient—certificate. Under zero angular-momentum rate it asks whether the authored CoM acceleration can produce a CoP inside the 1 cm-eroded finite sole hull with positive normal force and friction ratio at most 1.0.

| reference | gate | CoM a max | CoM jerk max | CoP min margin | CoP outside | friction max | DCM min margin | reach max | touchdown vxy/vz |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| synthetic step | FAIL | 0.00 m/s² | 0.0 m/s³ | -10.11 cm | 25.00% | 0.000 | -10.11 cm | 0.759 m | 0.001 / 0.036 m/s |
| CMU 0.35x + DCM preview | FAIL | 85.37 m/s² | 17387.0 m/s³ | -438.36 cm | 1.67% | 8.276 | -57.83 cm | 0.831 m | 0.099 / 1.683 m/s |
| CMU 0.35x + static support preview | FAIL | 3822.17 m/s² | 1495019.3 m/s³ | -19451.87 cm | 4.00% | 375.514 | -751.51 cm | 0.831 m | 0.099 / 1.683 m/s |
| CMU 0.10x + DCM preview | FAIL | 50.77 m/s² | 10339.3 m/s³ | -331.68 cm | 1.67% | 4.921 | -47.11 cm | 0.786 m | 0.028 / 1.683 m/s |
| PlaCo WPG · matched first step | FAIL | 1.53 m/s² | 344.6 m/s³ | -6.61 cm | 0.33% | 0.156 | -13.35 cm | 0.825 m | 0.008 / 0.007 m/s |

## Interpretation

The synthetic step is a deliberately simple control and also fails the open-loop balance contract even though its closed-loop tracking regression is green; it holds the CoM between the feet during single support. The CMU-derived references fail before Bonesaw chooses a task priority or integrates one state tick: finite-difference contact edges create extreme CoM acceleration/jerk, negative normal-force requests, and CoP/friction demands outside the support contract. The static support-preview variant is especially discontinuous. Scaling horizontal motion to 0.10× reduces the demand but does not change the failure class.

This locates the next architectural boundary: compare reference generators open-loop under the same initial state, footsteps, timing, sole geometry, and command. Only references that pass this contract should enter the WBC tracking and closed-loop physics suites.

## Independent PlaCo WPG result

PlaCo 0.9.23 plans the same initial CoM, initial sole poses, first touchdown pose, 199-tick starting double support, and 229-tick first swing. Only its `WalkPatternGenerator` runs: no `WalkTasks`, IK, WBC, state integration, or simulator is instantiated.

Relative to the CMU-derived reference, PlaCo reduces maximum CoM acceleration by `55.9×`, maximum jerk by `50.5×`, and maximum friction ratio by `53.1×`. Its touchdown position and velocity gates pass with wide margin.

The strict reference gate still rejects it: the zero-angular-momentum CoP is outside the newly active right sole on ticks `[199, 200]`. Tick 199 misses by `6.612 cm`; tick 200 is only 0.014 mm outside, then all remaining CoP samples are inside. This is a support-boundary defect, not a WBC failure and not a reason to weaken the gate.

PlaCo's public WPG provides CoM and trunk orientation but not pelvis translation. The comparison derives pelvis translation from PlaCo CoM minus the identical initial G1 CoM-to-root offset; that adapter affects only the reach check, not the centroidal certificate.

The standalone generator is bitwise repeatable: `True`. One isolated run records `873.9 ms` planning and `14.8 ms` to sample 600 ticks. Peak process RSS is `220.1 MiB`; it includes Python, PlaCo, Pinocchio, model import, and planner setup and is not an incremental planner allocation claim.

Sources: https://placo.readthedocs.io/en/stable/placo/placo__humanoid.html and https://github.com/Rhoban/placo at pinned revision e6c288604639d67b979a16cb2ad26913413c8e3a.
