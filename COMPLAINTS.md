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

## Open behavior gate: floating walking/contact transfer

R260 consolidates the pinned WBC behavior evidence into one Python manifest. The
fixed-base end-effector reach and bimanual-priority rows are retained alongside
the CMU subject 37/01 walking retarget, the measured-feedback floating
moving-liftoff prefix, the 600-tick floating support-transfer stress trace, and
the Upkie C++ WheelBalancer oracle. The moving-liftoff prefix and Upkie parity
pass; the walking row remains red at 5.549 cm foot RMS, and the full transfer
remains red with contingency/deadline/residual growth after the contact-mode
change. These are behavior/evaluation failures, not reset or authority
decisions. The next WBC slice is a causal support-transition/contact-mode
improvement with the same 250 Hz MuJoCo / 50 Hz measured-feedback contract.
See the [r260 WBC benchmark manifest](benchmarks/results/wbc-benchmark-manifest-r260/WBC_BENCHMARK_MANIFEST.md).

R262 now makes the timeout behavior explicit. The existing PyO3 floating WBC
session accepts a finite total Dykstra projection ceiling; the Python sweep
retains the unbounded baseline and 8/16/32/64-sweep traces. Unbounded work
misses the 20 ms 50 Hz deadline on 281/600 ticks at 276.8 ms p99. Every finite
profile has zero 20 ms misses, no failed/infeasible or contact-release tick,
and p99 below 5 ms, although generic-build maxima remain 16.44–16.70 ms. The
complete host-native profile pairs the minimum 8-sweep ceiling with two polish
iterations. Five CPU-4-pinned, 600-tick processes record 0/3,000 five- and
twenty-millisecond misses, 3.469 ms worst p99, 3.551 ms observed maximum,
72/72 exact non-timing arrays, and zero Python collections. This fixes the
"solver runs until the web view appears to stop" failure and demonstrates the
measured 200 Hz envelope for a fail-closed profile, but not floating transfer:
354/600 ticks are contingency and tracking/residual acceptance stays red. The
default remains unchanged; an unfinished solve never becomes executable and
does not reset state or grant authority. See the [r262 budget report](benchmarks/results/g1-floating-projection-budget-profile-r262/G1_FLOATING_PROJECTION_BUDGET_PROFILE.md).

R263 closes the separate interactive freeze/reset complaint. On the retained
600-tick cap8 transfer, the r262 state held an exact root-and-joint state for
171 ticks after contact contingency. Rust now treats `MaxIterations` and every
other unsolved contact result as non-executable, retries once with normal-only
rows, latches released targets out of hard rows until reset/schedule release,
and advances under a bounded free-body damping/gravity fallback when no
contact solve remains. The r263 trace has zero exact state stalls, one release
contingency, zero 20 ms misses, and 3.974 ms p99. The fallback is not a walking
or plant-authority claim: contact, tracking, residual, MuJoCo feedback, and
thermal gates remain open. See the [r263 recovery report](benchmarks/results/g1-floating-contact-release-r263/G1_FLOATING_CONTACT_RELEASE_R263.md).

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

R248 performs that fresh no-retuning plant A/B on the primitive two-probe G1
fixture, two new pyramidal laws, and offsets 250,000/260,000. The mechanism
remains clean—every candidate WBC query admits, hot paths allocate zero Rust
bytes, p99 stays below five milliseconds, and both 480-step branches produce
zero MuJoCo warnings—but the action profile is rejected. Only 14/48
implicitfast and 12/48 RK4 rows avoid every terminal component regression
against zero joint effort. Each branch uses five 4 ms physics steps per 20 ms
50 Hz WBC tick. Joint-position pressure remains the dominant miss, exposing
that effort magnitude alone is insufficient: bandwidth/slew realization must
enter the action boundary before another fresh holdout.

R249 now exposes a declared 20 ms Rust first-order realization sensitivity
family on those spent efforts. The boundary is bitwise exact, zero-allocation,
and sub-5 ms; its 25 Hz / 1,000 N·m/s row exercises three slew-limited
coordinates. R250's stricter spent-state action-freeze audit then runs realized
effort through fixed-effort WBC and MuJoCo and finds no useful safe profile:
there is no fresh action candidate or authority to promote.

R253 closes the R251 implementation seam without adding a policy. Rust now
selects from exact-three-candidate paired terminal-delta boxes over six
candidate-dependent deltas—tilt, angular rate, joint position, joint velocity,
actuator effort, and raw joint-headroom loss—with impact speed retained as a
candidate-invariant quantity and aggregate score separately gated. The
boundary is atomic, deterministic, and allocation-free. Fixed causal
closing-speed and tilt-sign groups freeze the neutral-recovery profile on
spent R250 evidence: three of 96 rows select the nonzero third candidate, all
three improve aggregate consequence, and none regresses a measured component.
This removes the missing Rust-bound complaint.

R254 has now spent the required one-shot no-refit holdout on new
elliptic/implicitfast and pyramidal/Euler laws at offsets 310,000/320,000. The
mechanism passes: all 96 causal groups are supported, WBC/realization/selector
hot paths allocate zero Rust bytes, p99 remains below five milliseconds, all
queries repeat, and 1,440 MuJoCo steps emit no warnings. The profile is
rejected. Frozen boxes miss both laws, led by joint-position pressure and its
headroom transform; four selected rows regress a component, two regress
aggregate consequence, and only one of five nonzero selections is both
strictly nonregressing and improving. No widening or refit follows.

R255 now supplies the missing complete-state interval composition boundary in
allocation-free Rust. It bounds clearance, vertical speed, root attitude/rate,
joint position, and joint velocity before terminal scoring; 2,048 independent
point checks are contained and a 64-row batch remains below 0.1 ms p99 with no
policy or physics. This does not resolve transfer by itself: independently
bounding candidate and baseline loses their shared uncertainty and cannot
certify a paired improvement. No profile or authority follows.

R256 now closes the correlation-preserving Rust mechanism. Candidate and
baseline share the same uncertain terminal state and only the candidate delta
varies; independent consequence uppers are never subtracted. A 3×4 G1 batch is
exact, allocation-free, below 0.6 ms p99, and contains 384/384 paired samples.
An authored contact/estimator construction also contains all 288 stored R254
terminal candidates using 96 kinematic forwards and zero policy/physics steps.
This removes the missing paired-state implementation complaint, but not the
calibration gate: the current absolute coverage can be broad and has not frozen
a useful paired action profile.

R257 attempts that freeze from immutable endpoint-velocity evidence with zero
policy or physics. The result is rejected: source component/aggregate coverage
is 71.875%/38.542%, the spent R254 rehearsal reaches 72.454%/37.153%, and all
192 evaluation selectors retain zero. Angular-rate, joint-velocity, and effort
are fully covered; tilt, joint position, and raw headroom are not, with a
34.853 maximum joint-position-pressure miss. This removes endpoint-velocity
residual fitting as an acceptable next step. A terminal-state corpus or a
certified within-step position/attitude trajectory bound is required.

R258 closes the missing-data seam without widening authority. The already-spent
R250 reset-every-sample traces now retain terminal clearance/vertical speed,
roll/pitch, roll/pitch angular rates, every joint position, and every
generalized velocity for all 96 × 3 branches. The extraction executes the
historical 480 WBC queries and 1,440 MuJoCo steps, emits no warnings, and an
independent allocation-free Rust state rescore reproduces all 4,896 diagnostics
exactly. This is spent design evidence only: R259 must fit a causal paired tube
from these immutable states and still demonstrate a useful nonzero profile;
profile and authority remain closed.

R259 performs that zero-policy/zero-physics fit over the complete retained
state. Component and aggregate coverage both reach 100% on the source corpus,
and the paired Rust query remains exact, allocation-free, and below 0.41 ms p99.
The smallest nonzero worst-component and aggregate uppers remain 18.486 and
9.451. The profile is still rejected: every one of 96 selectors chooses the exact-zero
baseline, so there is no useful nonzero action to transfer. The unchanged
spent R254 rehearsal reaches 98.553% component and 97.917% aggregate coverage
and also selects zero throughout. This closes the data-retention and complete
state-composition complaints, but not the physical transition gate; a causal
nonzero construction or a certified within-step action profile is still needed.

R261 supplies that source-local nonzero construction without policy, physics,
or plant actions in the evaluator. Rust keeps 3–12 complete state residual
exemplars correlated, including candidate-dependent clearance and vertical
speed, then performs support-free propagation, consequence scoring, hypothesis
enveloping, and conservative selection allocation-free at 28.10 µs p99. Source
component/aggregate coverage is 100%/100%, and all five nonzero selections are
strictly nonregressing and improving. Transfer is still rejected: the unchanged
spent R254 rehearsal reaches only 85.301%/80.208% coverage, with two selected
component regressions and one aggregate regression. No fresh holdout or
authority follows; the next profile must cover multiple spent law families
without destroying those useful selections.

R264 knocks off that multi-law profile complaint on spent evidence. One frozen
55-coordinate observed-state profile now uses same-cell nearest residual
prototypes across all four R248/R254 contact-law families, with asymmetric
causal-cell calibration and an explicit distance gate. The bounded Rust query
checks at most 192 caller-owned prototypes, allocates zero timed bytes, repeats
all non-timing outputs exactly, and agrees with Python on every nearest identity
(maximum squared-distance error 2.84e-14). Leave-one-law-out centers plus the
frozen calibration cover 192/192 component and aggregate rows; three nonzero
candidate-2 choices survive and all three are actually strictly nonregressing
and improving. The broad 38.317 component / 19.137 aggregate worst extensions
explain the deliberately low action rate. Model and source hashes are pinned,
and the evaluator executes zero policy steps, physics steps, or plant actions.
This profile was frozen for exactly one untouched new-law/offset holdout;
R265 has now consumed it, and no authority follows from spent rehearsal. See the
[r264 residual-prototype report](benchmarks/results/g1-residual-prototype-profile-r264/G1_RESIDUAL_PROTOTYPE_PROFILE.md).

R265 consumes that holdout exactly once without refit or widening. The new
medium elliptic/Euler and stiff elliptic/RK4 laws use offsets 330,000/340,000
and execute 1,440 fresh MuJoCo steps. Provenance, WBC/realization, allocation,
repeat, warning, and timing mechanisms pass: 88/96 rows remain inside the
frozen distance envelope, the Rust profile query is 0.996 µs p99, WBC is
3.253 ms p99, allocation is zero, and warnings are zero. The profile is
rejected. Supported component/aggregate coverage is only 92.045%/85.985%.
Joint-position pressure and headroom dominate at 27.992/2.834 maximum miss;
aggregate miss reaches 13.964. The sole nonzero action is medium-Euler row 20,
candidate 1: aggregate improves by 0.732, but tilt/angular-rate pressure regress
by 0.0119/0.00349, outside boxes that predicted improvement. Candidate 1 had
never been selected in the spent rehearsal but was still selectable on the
fresh state. The holdout is consumed and will not be rerun. The next profile
must freeze action-support evidence as well as state distance/calibration, then
be designed on now-spent R265 labels before any later holdout is declared. See
the [r265 one-shot report](benchmarks/results/g1-residual-prototype-plant-holdout-r265/G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT.md).

R252 also makes actuator lag failure tolerant at a lower layer: Rust can cap
positive observed mechanical power after bandwidth/slew realization, reports
every clamp, replaces the persistent lag state with the applied effort, and
preflights a complete trace before mutation. The mechanism is exact-replay,
zero-allocation, and useful on most spent rows, but strict terminal
non-regression still rejects every tested damping gain; it has no authority.

ABI 6 predicted-gap activation remains rejected: it produces 19 extra unloaded
contacts on the spent R241 replay and widens the implicitfast row to 22.719
rad/s joint width. Historical ABI 0 and Cartesian pyramid ABI 1 remain
unchanged; edge ABI 2 is evaluator-only and is not an authority mapping.

Acceptance still requires all of the following independent witnesses:

- freeze a new correlated profile on spent evidence with explicit per-action
  selection support, not only same-cell state distance, and require complete
  consequence coverage before declaring any later one-shot holdout;
- preserve R262's zero-overrun host-native envelope, allocation/GC contract,
  and exact replay when the useful floating behavior profile is promoted to
  the ordinary path; R262's fail-closed timing profile alone is not functional
  admission.

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
