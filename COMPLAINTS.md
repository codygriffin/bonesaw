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
query is 1.388/1.439 µs p99 with bitwise repeat and zero timed allocation.

R226 removes both the constant-impedance/reference-scaling shortcut and
independent-point contact distribution from this punch list. Generic Rust now
implements the documented positive reference law, position-dependent
impedance, refsafe clamp, declared friction-cone geometry, causal free point
acceleration, and a fixed-work complete-Delassus projected distribution. Six
coupled construction rows cover all 48 samples from both rejected R221 laws at
useful width and below a 5 ms CPU gate. The representative 32-step/free-
acceleration row fits angular/linear/joint width 0.095/0.030/6.131 and
0.449/0.065/9.320 at 0.463/0.634 ms p99. No empirical profile is promoted and
no fresh holdout is spent.

The live complaint is now independently freezing the coupled solver's
microstep/sweep/convergence rule before new evidence. Do not choose among the
32/64/128-step successes, retune impulse caps, or fit residual widths from
rejected R221 labels. The frozen candidate must pass a new untouched contact-
law holdout at useful width and deadline, then feed the retained R224 bound
into a selector with strict plant non-regression.

Acceptance still requires all of the following independent witnesses:

- freeze the coupled soft-constraint solver's microstep, sweep, cap, and
  residual contracts without selecting them from rejected R221 labels;
- freeze a tighter independently motivated transition set, pass a new untouched
  strict coverage/width/deadline holdout, then demonstrate an R224-bounded
  state/trajectory-conditioned action with strict plant non-regression;
- return the complete ordinary-process 200 Hz path to zero five-millisecond
  overruns and retain allocation/GC and exact-replay witnesses.

R225 removes external-load provenance from this punch list. Protocol 2 requires
an explicit source, executable load class, force frame, and application-point
frame; Rust rejects missing provenance and evidence-only impact/reserve classes
before worker mutation, while the MuJoCo worker independently revalidates the
contract. The browser operator and evaluation harness remain distinct sources.

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
