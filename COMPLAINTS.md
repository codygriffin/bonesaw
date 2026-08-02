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
rejected.

R245 corrects the cross-profile loophole and freezes work independently from
causal prediction refinement: ABI 0 selects 64 sweeps and ABI 4 selects 32.
The model evaluates spherical friction motion at the instantaneous surface
material point (including `ω × r`), keeps centre-minus-radius gap geometry,
and evaluates contact `aref` from the current collision-boundary gap. R246 is
the untouched confirmation on two new pyramidal laws and offsets
230,000/240,000. Both rows cover 48/48 with exact active sets: implicitfast
fits 0.059 / 0.007 / 4.949 angular / linear / joint width at 0.871 ms p99;
RK4 fits 0.065 / 0.010 / 2.239 at 3.399 ms. Both allocate zero timed Rust
bytes, repeat bitwise, meet five milliseconds, and reproduce all 62 semantic
arrays in an independent process. The physical-transition evaluator profile
is promoted; no R224 selection, plant action, or authority follows yet.

R247 now exposes a Rust-owned, allocation-free selector over three R224
velocity-box candidates with atomic diagnostics. A simulator-free audit runs
zero-desired-acceleration WBC, velocity damping, and neutral recovery on all
96 spent R246 states. All 288 primary WBC queries admit, hot paths allocate
zero Rust bytes, 34/34 non-timing arrays repeat bitwise, and the conservative
chooser selects 85 / 8 / 3 candidates with zero component regression. The
uncertainty width was fit on spent R246 labels, so this freezes a candidate
family for new plant evidence rather than serving as a holdout. It emits no
torque, plant command, or authority.

ABI 6 predicted-gap activation remains rejected: it produces 19 extra unloaded
contacts on the spent R241 replay and widens the implicitfast row to 22.719
rad/s joint width. Historical ABI 0 and Cartesian pyramid ABI 1 remain
unchanged; edge ABI 2 is evaluator-only and is not an authority mapping.

Acceptance still requires all of the following independent witnesses:

- run the frozen R247 state-conditioned WBC family on fresh MuJoCo trajectories
  and require strict baseline/candidate plant non-regression without retuning;
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
