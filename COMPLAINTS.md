# Bonesaw complaints and acceptance criteria

This is the live unresolved punch list. Completed work is removed and retained
in revisioned reports, the README, and `docs/IMPLEMENTATION_STATUS.md`.

## Open browser gate: viewport frame time

Scheduling, interpolation, cached geometry, disconnected ghost state, visible
green controls, orbit/pan/wheel zoom, coalesced drag, the filled z=0 plane,
exact visual-ground clearance, and simultaneous labelled preview/measured
plant state are implemented. The
remaining gate is measurement in the actual review browser with `?perf=1`:
retain frame/draw p50/p95/p99, missed-frame percentage, longest frame,
pointer-to-camera latency, snapshot jitter, command acknowledgement, and
controller solve time. Draw p95 must remain below 16.7 ms on a 60 Hz display.

This gate is **NOT RUN** in the current CLI session because no in-app browser
target is available. Transport or controller timing cannot substitute for it.

## Open CPU authority gate: useful physical transition tube

R224 removes robust terminal interval propagation from this punch list.
Allocation-free Rust now propagates every velocity in a declared componentwise
box through ballistic time, terminal tilt/rate, joint headroom/velocity, and
the separate pressure stack. The prior center score's 7/1 mid/hard false-safe
crossings become 0/0, and every contained completed state is bounded. The
query is 1.344/1.194 µs p99 with bitwise repeat and zero timed allocation.

The live complaint is the transition set feeding that mechanism. R221's frozen
profile covers only 44/48 mid-law and 31/48 hard-law source rows; the R224
contact-only terminal projection contains 44/48 and only 19/48 completed
states. The broad box also rejects 10/48 and 2/48 safe completed states. Do not
retune the 128 microsteps, 16× cap, or group widths from holdout labels. The
next candidate must have independent physical/statistical motivation, pass a
new untouched contact-law holdout at useful width and deadline, then feed the
retained R224 bound into a selector with strict plant non-regression.

Acceptance still requires all of the following independent witnesses:

- derive an independently motivated transferable contact-law residual or model
  parameterization before another untouched holdout; r221 already rejects the
  label-fit substepped-compliance profile despite useful width and deadline;
- type declared external-wrench/load provenance into the online command and
  keep it separate from impact impulse and unobserved-model reserves;
- freeze a tighter independently motivated transition set, pass a new untouched
  strict coverage/width/deadline holdout, then demonstrate an R224-bounded
  state/trajectory-conditioned action with strict plant non-regression;
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
  body/joint targets author ground-clamped guided queries while a dashed orange
  rig and contacts retain measured MuJoCo state. Ctrl+drag ray-picks any
  rendered body surface for a separately leased physical wrench. Explicit
  orange PUSH remains the touch-accessible equivalent.
- Keep Invariant, capture Viability, Intent, odom Preference, neutral
  posture/Style, physical resources, compute, map reporting, residual
  confidence, and plant consequence as separate authority rows.
- The public review surface stays on the single managed port 8777 pair; stale
  servers and tunnels must be stopped before replacement.
