# Bonesaw complaints and acceptance criteria

This is the live unresolved punch list. Completed work is removed and retained
in revisioned reports, the README, and `docs/IMPLEMENTATION_STATUS.md`.

## Open browser gate: viewport frame time

Scheduling, interpolation, cached geometry, disconnected ghost state, visible
green controls, orbit/pan/wheel zoom, and coalesced drag are implemented. The
remaining gate is measurement in the actual review browser with `?perf=1`:
retain frame/draw p50/p95/p99, missed-frame percentage, longest frame,
pointer-to-camera latency, snapshot jitter, command acknowledgement, and
controller solve time. Draw p95 must remain below 16.7 ms on a 60 Hz display.

This gate is **NOT RUN** in the current CLI session because no in-app browser
target is available. Transport or controller timing cannot substitute for it.

## Open CPU authority gate: useful physical transition tube

R217 implements the requested coupled contact-law/CoP construction: a generic
allocation-free Rust solve couples all eight G1 foot points through the full
Delassus operator, causal end-of-step nonpenetration, nonnegative normal
impulse, and a circular Coulomb projection. R218 then freezes the mapping,
iteration count, and split-energy coefficients before generating 96 new reset
transitions under untouched soft/pyramidal/Euler and stiff/elliptic/RK4 laws.
Both laws retain 100% sample/component coverage and zero timed Rust allocation,
so the generic mechanism is knocked off the punch list.

The authority profile is not. Frozen p95 widths are
2.420/0.142/35.754 and 13.170/0.785/175.201 against 2.0/0.5/10.0 gates. Even a
perfectly fitted centered interval around the stiff-law predictor cannot beat
17.498 rad/s joint width because that is twice the observed causal residual.
No holdout miss was tuned back into the profile. Prior R203–R216 construction
details now live in revisioned reports rather than this unresolved list.

R219 knocks off the implementation half of typed estimator uncertainty. Rust
now envelopes an explicit finite contact-hypothesis set atomically and without
allocation. The construction result rejects scenario enumeration as the next
authority profile: 315 scenarios reach only 81.25% coverage at the useful
9.185 rad/s joint-width edge; larger rows reach at most 96.875%, remain near
22.5 rad/s, and take 6.383–7.784 ms p99. No row earns a fresh holdout. The live
complaint is now a higher-order compliant predictor, not another larger finite
scenario list.

R220 implements that predictor and passes its all-label construction gate at a
useful 1.748/0.299/9.802 width with a 55.705 µs p99 allocation-free query.
R221 rejects the frozen profile on genuinely new laws: mid compliance covers
44/48 samples and hard compliance only 31/48. Bitwise repeat, component
coverage, and timing remain good, but strict coverage is not negotiable. The
implementation item is knocked off; the remaining complaint is contact-law
transfer. Do not retune the 128 microsteps, 16× cap, or group box from R221.

Acceptance still requires all of the following independent witnesses:

- derive an independently motivated transferable contact-law residual or model
  parameterization before another untouched holdout; r221 already rejects the
  label-fit substepped-compliance profile despite useful width and deadline;
- type declared external-wrench/load provenance into the online command and
  keep it separate from impact impulse and unobserved-model reserves;
- demonstrate a state/trajectory-conditioned terminal action with strict plant
  non-regression; the current sensitivity result is construction evidence, not
  a consequence certificate;
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
  body/joint targets author ground-clamped guided queries; Ctrl+drag ray-picks
  any rendered body surface for a separately leased physical MuJoCo wrench.
  Explicit orange PUSH remains the touch-accessible equivalent.
- Keep Invariant, capture Viability, Intent, odom Preference, neutral
  posture/Style, physical resources, compute, map reporting, residual
  confidence, and plant consequence as separate authority rows.
- The public review surface stays on the single managed port 8777 pair; stale
  servers and tunnels must be stopped before replacement.
