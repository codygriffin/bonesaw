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

True generalized RK4 is no longer the live complaint. Rust now owns four
allocation-free generalized state/contact stages per authored tick. Every stage
refreshes support geometry, floating bias/inverse dynamics, the factored mass
matrix, complete Delassus response, point velocity/acceleration, and collision
membership under a held generalized force. Analytic free-flight and mid-tick
contact-crossing regressions pass. On the spent R231 hard-RK4 rows, exact active
sets improve from 24/48 to 45/48 and missed actual points fall from 22 to 1.

R232 freezes 64 projected sweeps using prediction change only: the 64→128
refinement is 0.600% of the useful-width gate, p99 is 4.34 ms under the frozen
5 ms deadline, repeated outputs are bitwise exact, and the timed Rust boundary
allocates zero bytes. Completed R231 labels are opened only after selection.
The stateful score now compares Rust's returned final tangent with the reference
final tangent; it no longer projects a final impulse through the initial mass
response. On spent hard-RK4 rows this gives 46/48 coverage and
0.669/0.125/8.262 fitted angular/linear/joint width; the obsolete proxy had a
28.584 joint width.

The live complaint is now transferable final-tangent accuracy.
The untouched R233 RK4 laws retain the mechanism/deadline witnesses at
3.32–3.38 ms p99, but strict accuracy is rejected. Medium/elliptic has 46/48
frozen sample coverage with 47/48 exact active sets; hard/pyramidal has 44/48
coverage with 44/48 exact active sets. Their fitted angular /
linear / joint widths are 0.285 / 0.043 / 10.446 and 0.699 / 0.110 / 11.248.
The predictor commonly finds the right contacts and is now close to the 10
rad/s useful joint-width gate, but strict frozen coverage still fails. Do not
tune R233 labels into R232. Because the frozen
profile failed, it does not enter R224 terminal selection or plant
non-regression.

R236 removes the remaining localization ambiguity without changing a
coefficient or profile. On medium RK4, the full 10.446 rad/s joint width occurs
inside the 47 rows whose contact sets are already exact; on hard RK4 the exact-
set subset still needs 10.618 rad/s. Every worst coordinate is an ankle pitch
or roll joint. The fixture and predictor already share the same four 5 mm
spheres per foot. Foot-wrench errors, especially normal impulse and pitch
moment on the right foot, remain large. The live mechanism gap is therefore
stage-local force and within-foot wrench distribution/reference constraint-
solver semantics, with some hard-law activation tail—not primitive count,
radius, or another global activation threshold.

Acceptance still requires all of the following independent witnesses:

- derive a label-independent stage-local force/within-foot wrench construction
  that addresses the R236 ankle residual without fitting the spent R233 laws;
- pass a new strict cross-integrator coverage/width/deadline holdout, then
  demonstrate an R224-bounded state/trajectory-conditioned action with strict
  plant non-regression;
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
