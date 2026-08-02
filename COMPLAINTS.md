# Bonesaw complaints and acceptance criteria

This is the live unresolved punch list. Completed work is removed and retained
in revisioned reports, the README, and `docs/IMPLEMENTATION_STATUS.md`.

## Open browser gate: viewport frame time

Scheduling, interpolation, cached geometry, disconnected ghost state, visible
green controls, orbit/pan/wheel zoom, coalesced drag, the MuJoCo-derived filled
plane, exact visual-ground clearance, orange measured collision wireframe,
physical CoM/contact/constraint witnesses, and simultaneous labelled
preview/measured plant state are implemented. The
remaining gate is measurement in the actual review browser with `?perf=1`:
retain frame/draw p50/p95/p99, missed-frame percentage, longest frame,
pointer-to-camera latency, snapshot jitter, command acknowledgement, and
controller solve time. Draw p95 must remain below 16.7 ms on a 60 Hz display.

This gate is **NOT RUN** in the current CLI session because no in-app browser
target is available. Transport or controller timing cannot substitute for it.

## Open CPU authority gate: useful physical transition tube

The RK4 stage-force and implicitfast-integrator conflation complaints are now
resolved mechanisms. MuJoCo excludes constraint forces from implicitfast's
force-velocity Jacobian, so Bonesaw now maps non-RK contact reference
acceleration to explicit constraint-RHS ABI 0 while RK4 uses current-stage ABI
4. Historical mappers and evidence remain unchanged.

R240 freezes 128 sweeps from causal prediction refinement only: 128→256 is
1.980% of the useful-width gate, selected p99 is 3.34 ms, allocation is zero,
reruns are bitwise exact, and all 136 semantic arrays reproduce. The untouched
R241 holdout crosses the prior cone/integrator pairing. RK4/id 4 again covers
48/48 at 0.100 / 0.009 / 3.120 angular / linear / joint width and 3.56 ms p99.
The pyramidal implicitfast/id-0 row covers 47/48 at 0.124 / 0.025 / 17.138;
its lone failure predicts three unloaded foot points. Both meet deadline,
repeat, allocation, and 62-array replay gates, but the combined profile is
rejected. The live CPU complaint is now non-RK within-tick contact activation
and within-foot wrench distribution—not RK stage timing or enum mapping. No
R224 selection, plant non-regression, or authority follows this failed holdout.

Two opt-in diagnostics are also rejected. R242's ABI 6 predicted-gap activation
rule produces 19 extra unloaded contacts on the spent R241 replay and widens
the implicitfast row to 22.719 rad/s joint width. R243's pyramid-edge cone
coordinates (`N ± μT`) fail the causal convergence gate even at 256 sweeps.
Historical ABI 0 and Cartesian pyramid ABI 1 remain unchanged; neither
diagnostic is a production mapping.

R244 is the fresh edge-coordinate holdout: implicitfast/id 0 covers 47/48 at
0.220 / 0.048 / 16.030 width, while RK4/id 4 misses the 5 ms mechanism gate at
5.549 ms and needs 0.128 / 0.007 / 6.279. The candidate remains rejected.

Acceptance still requires all of the following independent witnesses:

- derive a label-independent non-RK within-tick activation/wrench construction
  without fitting R241 labels, then pass a new strict mixed-integrator holdout;
- demonstrate an R224-bounded state/trajectory-conditioned action with strict
  plant non-regression after, and only after, that transferable holdout passes;
- return the complete ordinary-process 200 Hz path to zero five-millisecond
  overruns and retain allocation/GC and exact-replay witnesses.

## Open robustness and hardware calibration

Still required: slopes, simultaneous/contact-coupled impulses, more material
application points and bodies, model perturbation, estimator/transport delay
and noise, actuator bandwidth, identified effort/power/thermal limits,
authenticated plant transport, and hardware validation. Simulator reset is not
recovery.

## Product direction retained from review

- Upkie remains the preferred interactive reference; larger humanoids are
  evaluation fixtures.
- Empty-space drag orbits; Shift-drag always pans; wheel zooms; green
  body/joint targets author ground-clamped guided queries while an orange
  collision wireframe, dashed rig, CoM, contacts, and constraint witnesses
  retain measured MuJoCo state. Ctrl+drag ray-picks any
  rendered body surface for a separately leased physical wrench. Explicit
  orange PUSH remains the touch-accessible equivalent.
- Keep Invariant, capture Viability, Intent, odom Preference, neutral
  posture/Style, physical resources, compute, map reporting, residual
  confidence, and plant consequence as separate authority rows.
- The public review surface stays on the single managed port 8777 pair; stale
  servers and tunnels must be stopped before replacement.
