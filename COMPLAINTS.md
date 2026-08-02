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

Rust now has a distinct model-only current-stage-force RK4 ABI. It preserves the
historical full-tick RK4 path and R232/R233 evidence, but cancels that path's
extra complete gap advance before evaluating each stage derivative. Every
stage still refreshes support geometry, dynamics, mass factor, complete
Delassus response, point motion, and collision membership in caller-owned
storage. Scalar APIs reject both generalized variants.

R237 freezes 64 projected sweeps without completed labels: 64→128 changes the
prediction by 0.801% of the useful-width gate, the selected 64-sweep p99 is
4.06 ms (128 sweeps measures 4.18 ms), timed Rust allocation is zero, reruns
are bitwise exact, and an independent run reproduces all 122 semantic arrays.
Spent R233 labels are opened only afterward.

The untouched R238 cross-integrator holdout is the current boundary. Its new
RK4/id-4 row covers 48/48 samples at 0.122 / 0.018 / 5.191 fitted angular /
linear / joint width and 2.82 ms p99. Its new implicitfast/id-1 comparator
covers only 46/48 and needs 0.213 / 0.045 / 21.255 width, so the conjunctive
profile is rejected. Both rows repeat bitwise, allocate zero timed Rust bytes,
and meet the deadline; all 62 semantic arrays reproduce exactly. The original
stage-local-force complaint is resolved, but transferable accuracy across
integrators is not. No R224 selection, plant non-regression, or authority is
allowed from this failed combined holdout.

Acceptance still requires all of the following independent witnesses:

- isolate and correct the remaining implicitfast/contact-update transfer tail
  without fitting R238 labels, then pass a new strict mixed-integrator holdout;
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
