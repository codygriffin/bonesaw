# G1 CoM-reference causal analysis — r32

## Outcome

Revision r32 locates the first sustained-walk divergence before precontact,
touchdown, or contact fallback. The existing `support-preview` experiment feeds
the centre of the future support patch directly to a 2 Hz CoM position servo.
Across the final 40 ticks of initial double support, that reference moves
23.7 cm laterally at `0.578 m/s`, then its finite-difference acceleration peaks
at `57.8 m/s²` at liftoff. This treats CoM position like a ZMP/centre-of-pressure
command. It is not a dynamically valid walking reference.

The consequence is causal and repeatable. While both feet are still locked,
the Viability-level CoM task drives shoulder coordinates to the session's
`8 rad/s` envelope. After left liftoff, the left foot shares that priority and
inherits a small acceleration residual on every tick; its tracking error grows
from zero at tick 198 to 1.1 cm at tick 210, 3.4 cm at tick 220, 6.6 cm when
precontact begins, and 18.1 cm by tick 249. The r31 touchdown gate later rejects
the resulting far/fast landing correctly.

Removing the CoM task from the otherwise identical 250-tick prefix holds the
left-foot error to 6 mm at tick 249, keeps root RMS at 4 mm, and avoids the
pre-liftoff upper-body velocity explosion. That control eventually loses
single-support balance in the full 600-tick trace, so “disable CoM” is evidence,
not a proposed policy. The next canonical controller must use a dynamically
consistent capture-point/ZMP preview or equivalent support-margin law.

## New authoritative observation

`bonesaw-py::FloatingWbcSession.run_trace` now writes measured world-frame CoM
position into a caller-provided fixed-shape NumPy array on every Rust tick. The
timed loop performs no Python callback and allocates no per-tick object. The
corpus archives `center_of_mass_tracked` beside its target jets and reports CoM
RMS and p95. This closes the prior diagnostic gap where only the CoM task's
acceleration residual was visible.

The eval boundary also exposes explicit, non-default experimental controls for
foot-task priority, CoM derivative suppression, cadence multiplier, protected
leg-yaw posture, and authored-joint braking. The joint-braking interval itself
lives in `bonesaw-core`, is allocation-free, and is unit tested. It remains
opt-in because the current long walking trace can enter its emergency braking
envelope and make the strict contact solve infeasible; it is not silently
declared canonical.

## 250-tick causal prefix

All cases use the official Unitree G1 23-DOF URDF, identical CMU 37/01 retarget,
four sole force points, r31 measured touchdown semantics, the r30 cached-Jacobi
solver, and one Rust batch call. The prefix ends before authored touchdown.

| Case | Root RMS | CoM RMS | Swing-foot RMS | Max root rotation | p99 |
|---|---:|---:|---:|---:|---:|
| r32 observed baseline: support-centre CoM, full jets, Viability | 2.323 cm | 3.550 cm | 8.921 cm | 2.117° | 4.737 ms |
| causal control: CoM task disabled, centroidal damping retained | 0.400 cm | 1.160 cm | 0.360 cm | 1.750° | 2.590 ms |
| position-only CoM, 0.3 Hz | 1.460 cm | 6.050 cm | 4.320 cm | 1.430° | 4.300 ms |

The no-CoM control is not a balance solution: its 600-tick trace reaches the
first contact contingency at tick 280 and then diverges. It is valuable because
it proves the precontact swing error is caused by the CoM policy rather than FK,
Jacobian sign, contact admission, target anchoring, or Web/Python transport.

## Default-policy regressions with measured CoM

No default controller or corpus policy changed in r32.

| Corpus | Functional | 5 ms p99 | Root / CoM / stance / swing RMS | Status |
|---|---:|---:|---:|---:|
| deterministic 1 cm G1 toe-step | PASS | PASS · 4.125 ms | 3.567 / 4.445 / 0.002 / 0.639 cm | 0 contingency, release, infeasible, or failed ticks |
| official-G1 moving liftoff | PASS | PASS · 3.852 ms | 0.748 / 1.561 / 0.000 / 0.173 cm | 0 contingency, release, infeasible, or failed ticks |
| 600-tick CMU support-preview stress | FAIL | PASS · 4.795 ms | 20.026 / 26.808 / 28.940 / 27.249 cm | 172 denied touchdown ticks; outgoing support remains locked; no fallback or rejected tick |

The accepted toe-step and liftoff prove the added observation channel is
inactive with respect to solve/integration behavior. The CMU trajectory and
non-timing status sequence remain the r31 result while its previously missing
actual-CoM error is now explicit.

## Rejected hypotheses and experiments

| Experiment | Evidence | Decision |
|---|---|---|
| Retry the identical assembled solve | repeated infeasibility did not recover | no hidden workspace nondeterminism; diagnostic retries removed |
| Promote foot tracking to Invariant | removes the earliest residual, then reaches 25.688/33.418/34.234 cm root/stance/swing RMS, 40.164° root rotation, and 77.049 ms p99 over 600 ticks | do not encode a reference defect as a priority rule |
| Extend support preview from 40 to 120/200 ticks | reduces reference speed but moves the root far outside its mocap intent and triggers contingency/large dense tails | simple averaging is still the wrong dynamics |
| Zero CoM velocity/acceleration jets | removes the 57.8 m/s² edge impulse but a 2 Hz position servo still distorts the swing | retained only as an eval control |
| Slow position-only CoM servo to 0.3 Hz | improves the prefix, then loses long-run balance and reaches 197 ms p99 under fallback work | useful gain bracket, not a balance policy |
| Disable centroidal damping | improves the first swing, then full-trace attitude/contact behavior collapses | damping is stabilizing later even though it competes early |
| Protect upper body and hip yaw at Viability | prevents arm/yaw flailing but pushes saturation into ankle pitch/roll and worsens swing reach | a nullspace symptom, not the root cause |
| Add discrete joint braking bounds | unit-level envelope is correct, but this already-invalid trace reaches emergency braking and strict contact infeasibility | keep opt-in until balance/reference policy stays inside the viability set |
| Shorten startup ramp / increase cadence | shortens single support but raises target speed and adds more failed transfers; cadence is not the first cause | retain cadence multiplier for sensitivity only |
| Project touchdown to a shorter root-to-foot reach | smooth geometric projection still worsens the active dynamic path | reach was not the first divergence |

## Next canonical experiment

Implement an online balance task in Rust from measured CoM position and velocity
plus the measured support polygon. The predeclared comparison should include:

1. the current support-centre position servo as a static control;
2. a bounded capture-point/DCM feedback law whose virtual ZMP is clipped to the
   measured support polygon;
3. the no-CoM causal control;
4. unchanged toe-step, moving-liftoff, and 600-tick transfer gates;
5. actual CoM, DCM, virtual/clipped ZMP, contact force, joint-limit margin,
   solver-work, latency, CPU, RSS, GC, and tracking arrays over time.

Acceptance must require a valid measured touchdown, no outgoing-support release
before replacement lock, no contingency/rejected ticks, the existing hard-row
`1e-8` contract, and the existing 5 ms p99 deadline. Widening the touchdown
envelope or lowering the tracking gate would hide the defect and is excluded.

## Retained artifacts

- accepted toe-step: `benchmarks/results/g1-synthetic-step-r32-com-observation`
- accepted moving liftoff:
  `benchmarks/results/floating-g1-liftoff-r32-com-observation`
- default CMU stress with actual CoM:
  `benchmarks/results/g1-cmu-transfer-r32-com-observation`
- 250-tick default causal prefix:
  `benchmarks/results/g1-transfer-r32-com-observation-prefix`
- 250-tick no-CoM control: `benchmarks/results/g1-transfer-r32-no-com-prefix`
- 250-tick slow position-only control:
  `benchmarks/results/g1-transfer-r32-com-slow-position-prefix`
- opt-in joint-braking stress:
  `benchmarks/results/g1-transfer-r32-joint-braking`
