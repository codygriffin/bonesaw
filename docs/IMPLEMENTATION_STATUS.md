# Implementation status against architecture specification v0.1

This file separates demonstrated behavior from architectural intent.

## Current CPU checkpoint — r288 support-transfer motion headroom

R288 adds a default-off Rust witness for the support-feasible transfer tube.
For each authored joint it reports the directional finite-limit margin after
one reaction interval plus ideal stopping distance, and the remaining authored
velocity-limit fraction. The support-tube opt-in can take the minimum of this
motion witness and the legacy position-only witness; a negative margin remains
evidence of an unrecoverable observation rather than a command or authority
grant. Five CPU-4-pinned Upkie process repeats execute 20,000 warmed calls per
case with bitwise-repeatable outputs and zero allocations, bytes, or
deallocations in the measured loop (median 77.3 ns/call across cases). This
closes the typed joint-headroom input seam, but does not admit walking,
contact, timing, thermal, plant, or hardware authority. See
[`JOINT_MOTION_HEADROOM_R288.md`](../benchmarks/results/joint-motion-headroom-r288/JOINT_MOTION_HEADROOM_R288.md).

## Current CPU WBC timing experiment — r290 post-transfer Style budget

R290 arms the existing terminal Style projected-solve ceiling only after the
support-reachable tube has clipped an intent request. The clipping tick stays
on the nominal unbounded path; the next solve is the first eligible capped
solve. On the frozen 2,317-tick G1 replay, the first clip is tick 819 and the
inclusive prefix through that tick is bit-exact. Style-2 exhausts once at tick
1152, keeps dynamics/contact residuals below `1e-8`, but raises root/CoM/foot
RMS by 10.0%/10.3%/10.5% and still misses the 5 ms p99 gate across five
CPU-4-pinned repeats (mean `5.177 ms`). The gate is retained as a typed,
default-off research primitive; no walking or authority promotion follows.
See [`G1_POST_TRANSFER_STYLE_BUDGET_R290.md`](../benchmarks/results/g1-post-transfer-style-budget-r290/G1_POST_TRANSFER_STYLE_BUDGET_R290.md).

The supplemental six-profile screen confirms there is no missed static
ceiling. Style-1 repeats one non-timing digest but spans 4.852–5.185 ms p99,
moves first release 1234→875, and worsens root RMS 41.35%. Style-2 remains
above 5 ms and worsens root RMS 10.05%; Style-3/4/6/8 never exhaust and are
byte-exact on every common non-timing array. See
[`G1_POST_TRANSFER_STYLE_BUDGET_SCREEN_R290.md`](../benchmarks/results/g1-post-transfer-style-budget-screen-r290/G1_POST_TRANSFER_STYLE_BUDGET_SCREEN_R290.md).

## Current CPU optimization audit — r289 direct-offset Jacobi pointer

R289 tests direct base-pointer offsets around the arithmetic-identical R284
Jacobi loops. Four CPU-4-pinned, 2,317-tick pairs preserve all 112 current
non-timing arrays byte-for-byte. Three pinned complete-process counter pairs,
however, show +0.243% retired instructions in every candidate run. The
candidate is rejected and remains available only through
`jacobi-column-offset-pointer-experiment`; R284 stays the production default.
No timing, authority, policy, physics, contact-transfer, or CUDA gate changes.
See
[`G1_JACOBI_COLUMN_OFFSET_POINTER_R289.md`](../benchmarks/results/g1-jacobi-column-offset-pointer-r289/G1_JACOBI_COLUMN_OFFSET_POINTER_R289.md).

## Current CPU architecture slice — r287 allocation-stable historical queries

R287 closes R286's legacy reconstruction-allocation caveat with
`RobotHistory::reconstruct_into`, `ExternalFrameHistory::reconstruct_into`, and
an explicit `HistoricalFrameQueryWorkspace`. Strict scalar/batch calls own the
robot state, external samples/provenance, model cache, atlas inputs/snapshot,
and retained outputs; undersized storage is a typed error. Twenty-one
independent Upkie process runs execute 2,048 warmed queries each with zero
measured allocations, bytes, or deallocations, bitwise repeatability, stable
2→2 capacities, and a CPU-4-pinned 1.380 µs/query median. This is a query/dataflow result
only and grants no WBC, physics, plant, or command authority. See
[`FRAME_QUERY_ALLOCATION_R287.md`](../benchmarks/results/frame-query-allocation-r287/FRAME_QUERY_ALLOCATION_R287.md).

## Prior CPU architecture slice — r286 historical frame-query batching

R286 adds caller-owned `CompiledFrameAtlas::query_history_into` and
`query_history_batch` workspaces. The batch surface preserves scalar query
semantics, returns an indexed typed error on failure, and reuses each output's
external-provenance capacity across repeated batches. The Upkie audit runs 64
queries per batch for 32 measured batches with bitwise-stable results and
stable provenance capacity; it does not claim the legacy state reconstruction
path is allocation-free and makes no WBC, physics, or authority claim. See
[`FRAME_QUERY_BATCH_R286.md`](../benchmarks/results/frame-query-batch-r286/FRAME_QUERY_BATCH_R286.md).

## Current CPU checkpoint — r285 typed low-authority degradation

R285 adds independent, default-off Preference and Style projected-solve
ceilings. No ceiling applies to Invariant, Viability, Intent, equality, bound,
or named hard-row work. Exhaustion returns the current hard-feasible partial
optimum, preserves completed higher authority, skips lower levels, and is
retained across retries as a fixed-capacity typed mask. The unbounded default
reproduces the established 89-array digest. A Style-2 profile exhausts only at
ticks 155/158 and reaches 4.431 ms p99 with zero hard failures, but changes the
later contact path and worsens root RMS by 81.81%; it is rejected. The
mechanism is retained as a degraded-mode primitive, not walking or command
authority. See [`G1_LOW_AUTHORITY_BUDGET_R285.md`](../benchmarks/results/g1-low-authority-budget-r285/G1_LOW_AUTHORITY_BUDGET_R285.md).

R284 promotes an arithmetic-identical Jacobi column-pointer kernel. Five
CPU-4-pinned, policy-free/physics-free repeats preserve all 89 established
non-timing arrays and reduce the pinned three-repeat average by 2.26% retired
instructions, 1.17% cycles, and 3.06% cache misses. The timing p99 mean moves
5299.6→5264.8 µs but remains above the 5 ms gate; no authority or physics
claim follows. `jacobi-column-pointer-control` restores the pre-R284 kernel.
See [`G1_JACOBI_COLUMN_POINTER_R284.md`](../benchmarks/results/g1-jacobi-column-pointer-r284/G1_JACOBI_COLUMN_POINTER_R284.md).

R279 adds an exact discrete backward preimage of the authored support schedule.
The fixed-size Rust fold produces a time-varying DCM box; a moving-boundary
barrier includes measured CoM velocity, reference-phase boundary velocity, and
an explicit acceleration envelope. Observer telemetry, intent projection, soft
task authority, and four-face hard admission are independently default-off.
Python retains the immutable policy-free/physics-free scenarios, metrics, and
report.

R283 qualifies a corpus-specific seven-iteration active-set accelerator while
retaining the exact eight-sweep Dykstra boundary. Five CPU-4-pinned runs match
all 89 established non-timing arrays. The 1694/2296 release tails fall from
195/184 ms to 7.2–7.9 ms and dense polish inverses fall 126→12, but every p99
still misses 5 ms at 5.204–5.517 ms. Budgets below seven changed behavior, so
the universal default remains 64 and no authority is admitted. Ordinary
Preference/Style task inversions are the next exact CPU target. See the
[`G1_FEASIBILITY_POLISH_BUDGET_R283.md`](../benchmarks/results/g1-feasibility-polish-budget-r283/G1_FEASIBILITY_POLISH_BUDGET_R283.md)
report.

R282 adds a fixed-capacity, allocation-free cumulative witness for every
bounded floating-WBC solve attempt. It records stable stage masks and
per-priority/per-stage task pseudoinverse, Jacobi, clipping, feasibility, and
polish work. The regenerated 2,317-tick trace preserves all pre-existing
semantic arrays, makes retry work visible on release tails, and remains
policy-free/physics-free with authority closed. The measured replay is
5.283 ms p99 with 0 Python collections and 3.891 MiB RSS growth; this is
telemetry, not an optimization or admission result. See the
[`G1_CUMULATIVE_SOLVE_DIAGNOSTICS_R282.md`](../benchmarks/results/g1-cumulative-solve-diagnostics-r282/G1_CUMULATIVE_SOLVE_DIAGNOSTICS_R282.md)
report.

R281 rejects and removes a bit-exact two-column Jacobi specialization: all 89
non-timing arrays match, but p99 worsens 5.200→5.364 ms. Preference/Style own
67.3% of task Jacobi sweeps, and deterministic status-5 tails at ticks
1694/2296 overwrite the failed solve/retry counters with the final fallback's
zeros. The next implementation seam is allocation-free cumulative per-attempt
work telemetry, followed by an exact reduction in repeated projected-task
inverse or dense-product work.

R280 qualifies the full two-axis soft construction on five CPU-4-pinned,
policy-free/physics-free processes. Every semantic trace is bit-exact. Release
moves 1108→1234 and root/CoM/foot RMS improves by 14.35%/14.90%/16.87%, but all
five p99 values miss the 5 ms gate (5.226–5.730 ms). Hard enforcement admits
662/1832 active ticks and leaves 1170 explicitly unresolved. A world-Y-only
specialization was removed because its 4.956 ms p99 came with worse-than-dormant
tracking. No authority is granted. The immediate slice is exact-trace CPU
optimization and deterministic release-tail localization. See the
[`G1_SUPPORT_REACHABLE_TUBE_R280.md`](../benchmarks/results/g1-support-reachable-tube-r280/G1_SUPPORT_REACHABLE_TUBE_R280.md)
report.

## Prior CPU checkpoint — r278 local support-transfer tube

R278 introduced the fixed-capacity, bias-corrected hard CoM-acceleration
halfspaces and independent preview request used by R279. Its best five-tick
local-intersection DCM barrier admitted 233/936 active requests but moved
conflict/release to tick 324 and missed the 5 ms p99 gate. R279 retains that
typed-boundary baseline and replaces its local geometry with schedule reachability.

## Prior CPU checkpoint — r276 automatic contact localization

R276 completes the default-off per-target contact-localization seam predicted by
R275. After an unfinished full-lock solve with at least two represented
targets, Rust probes each target in stable order with one normal-only solve;
failed candidates restore the exact rank-minimal contact pattern, while only a
solved candidate marks that target `NormalFallback` and integrates its output.
Single-target failures are deliberately not probed. The PyO3 trace exposes
attempt count, admitted target, and typed status 13 in caller-owned arrays.

The dormant replay is exact against the retained R275 control on all 81 shared
non-timing arrays. In the policy-free, physics-free G1 replay, the enabled
profile spends two probes at tick 875, rejects left, admits right, and keeps the
left foot locked. It still reaches left-only fallback/release at 885/886 versus
control release 1108; root/foot RMS becomes 16.782/16.376 m, p99 is 5.216 ms.
The mechanism passes, the walking profile and
authority are rejected, and the feature remains default-off. The remaining WBC
slice is pre-liftoff coupled root/CoM/support shaping or a support-feasible
trajectory tube, not another failure-time selector. See the
[`G1_AUTOMATIC_CONTACT_LOCALIZATION_R276.md`](../benchmarks/results/g1-automatic-contact-localization-r276/G1_AUTOMATIC_CONTACT_LOCALIZATION_R276.md)
report.

## Prior CPU checkpoint — r274 joint-position stopping-headroom capture

R274 adds `joint_position_capture_acceleration`, an allocation-free scalar
request based on directional stopping distance plus a bounded reaction-time
guard. The PyO3 profile defaults its weight to zero, selects lower-body joints
from the pinned URDF names, composes into the existing preallocated Viability
joint-task slot, and leaves hard stopping intervals unchanged. A dedicated raw
array reports active coordinate count separately from velocity-envelope work.
The dormant trace matches every shared R273 non-timing array and reports zero
activity; every candidate is exact through its first activation at tick
859–863. Five retained profiles activate for 13–17 ticks on up to three
coordinates, but all reach the right-knee lower limit at tick 875. Fallback is
876 or 888 and the best release is 1020 versus baseline 1108. Root RMS remains
14.886–18.989 m; several profiles also exceed the 5 ms p99 gate. The mechanism
passes, while walking profile and authority are rejected. The next slice must
address coupled root/foot/support feasibility before the limit rather than add
another local braking gain. See the
[`G1_JOINT_POSITION_CAPTURE_R274.md`](../benchmarks/results/g1-joint-position-capture-r274/G1_JOINT_POSITION_CAPTURE_R274.md)
report.

## Prior CPU checkpoint — r273 bounded NormalFallback relock probe

R273 makes `NormalFallback` recoverable without making a schedule bit or an
unfinished solve authoritative. `normal_fallback_relock_probe_interval_ticks`
defaults to zero. On an eligible tick, Rust emits the ordinary full locked
contact modes and spends one bounded solve. A solved result is the only event
that promotes the target back to `Locked`; an unsolved result is discarded,
the normal-only modes and viability task are restored, and the controller
spends one bounded retry. Admission, rejection, and rejection-with-release have
typed statuses 10/11/12. Interval zero reproduces all 72 R272 non-timing arrays
bit-for-bit, and every enabled trace is identical through fallback tick 875.
Intervals 1/4/8/16/32 produce full-lock episodes lasting up to ten ticks,
proving the former absorbing state is gone, but release earlier at ticks
916/926/976/957/1011 versus baseline 1108. Interval 64 rejects all three probes and preserves tick 1108
while adding work. The mechanism passes; all tested walking profiles and
authority remain rejected. The next slice is pre-limit feasibility and
compositional tracking recovery, not another post-event gain. See the
[`G1_NORMAL_FALLBACK_RELOCK_PROBE_R273.md`](../benchmarks/results/g1-normal-fallback-relock-probe-r273/G1_NORMAL_FALLBACK_RELOCK_PROBE_R273.md)
report.

## Prior CPU checkpoint — r272 NormalFallback task-scale causal split

R272 makes the existing NormalFallback viability point-task weight an explicit
scale (`normal_fallback_task_weight_scale`, default `1.0`). Scale `0.0`–`0.5`
reduces only that soft task after measured fallback; hard contact rows, force
cones, acceleration bounds, and release cleanup are unchanged. The dormant
scale-1 replay is bitwise equal to R270 on all 72 non-timing arrays. In the
policy-free, physics-free G1 sweep, every reduced scale is also bitwise equal
to baseline through fallback tick 875. The knee reaches its lower limit at tick
874 with the same −4.097 rad/s critical-window minimum in every profile; later
full-trace velocity differences cannot be attributed as prevention. Scale 0.25 stays below 5 ms p99
but worsens root RMS to 17.030 m; scale 0.5 improves RMS to 14.691 m but
crosses p99 at 5.162 ms. No scale is admitted; the next slice remains
continuous position-limit/support-transition recovery. See the
[`G1_NORMAL_FALLBACK_TASK_SCALE_R272.md`](../benchmarks/results/g1-normal-fallback-task-scale-r272/G1_NORMAL_FALLBACK_TASK_SCALE_R272.md)
report.

## Current CPU checkpoint — r271 aggregate support-load floor falsifier

R271 adds an opt-in hard aggregate normal-load row to each finite support patch.
The row constrains the sum of normal-force slots, allowing a four-point sole to
redistribute load instead of imposing equal per-slot floors. The Python shell
exposes `minimum_support_load_fraction`, validated in `[0, 1]`, and leaves its
default at zero. The requested fraction of supported weight is split evenly
over active patches. A zero-floor A/B preserves all 72 retained non-timing
arrays bit-for-bit, and exact compact-slot audits find zero violations in every
enabled row. A policy-free, physics-free G1 sweep at 0.5%, 10%, and 25% still
shows the global walking profile is not admissible: first fallback moves
875→505→424→308 and release follows immediately for the floor profiles, while
full root RMS reaches 24.382–28.960 m. This makes the previous 42–58 N
underloaded-support episodes explicit rather than silently trading them against
intent, but it does not provide useful walking authority. The generic mechanism
remains default-off and the global profile is rejected; the next slice remains continuous position-limit and
support-transition recovery. See the
[`G1_SUPPORT_LOAD_FLOOR_R271.md`](../benchmarks/results/g1-support-load-floor-r271/G1_SUPPORT_LOAD_FLOOR_R271.md)
report.

## Current CPU checkpoint — r270 lower-body velocity-envelope causal split

R270 adds an opt-in safety profile around the persistent floating WBC's
joint-velocity envelope. The allowlist is derived once from the pinned URDF
joint names (`hip`, `knee`, `ankle`, `wheel`, or `leg`), so upper-body joints
cannot be activated by the protective layer. The soft viability task and hard
directional braking bound are independently switchable, enabling a true
hard-only control with zero soft weight. Conflicting hard intervals fail closed
at the solver boundary. The profile is default-off, and its tick path reuses
preallocated coordinate/acceleration storage.

On the policy-free, physics-free low-gain morphology replay, early
multi-support envelope activity is zero. Hard-only holds the critical
right-knee coordinate above -8 rad/s (minimum -4.097 rad/s) and moves first
NormalFallback 863→888; soft-only moves it only to 864. Soft+hard reaches the
next support transition at tick 1108, but composition with r269 releases at
tick 897 and exceeds the 5 ms p99 gate. The standalone combined distribution
is under that timing gate. The complete trace remains rejected: full root RMS
is 15.514 m and the knee reaches its lower position limit at tick 874. This is
a causal mechanism result, not a walking, composition, or authority result;
the next slice is continuous position-limit/support-transition recovery. See the
[`G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md`](../benchmarks/results/g1-lower-body-velocity-envelope-r270/G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md)
report.

## Current CPU checkpoint — r269 bounds continuation and localizes handoff

R269 makes finite-cap contact failure continuous across controller calls without
turning unfinished work into an executable command. An opt-in hard-problem cache
can resume the exact Dykstra point and row multipliers for an identical problem;
each solver query still spends at most eight feasibility sweeps. The current
primary-plus-retry path can issue two queries, making the observable WBC-tick
ceiling 16 sweeps. A session-level hold budget (12 ticks in this evaluation)
emits typed status 8 and integrates no output. Ticks 876–887 preserve q, v,
root pose, tracked points, and CoM bit-for-bit while cumulative work advances
16, 32, …, 192 sweeps.

At tick 869, typed status 9 removes only the failed right support and retains
the locked left support with 336.5 N and hard residuals below 1e-8. First global
release moves from tick 869 to 888. A separate centroidal control exercises
the same localized handoff at tick 529: only the failed right
support is removed, the incoming left support remains active with 227.3 N, and
the accepted partial-support solution has hard residuals below 1e-8. Status 5
remains global free-body release and its rejected residual witnesses are zero.
Target 1/right-foot fallback is predeclared by this diagnostic profile; R269
does not infer which support failed, so automatic fault localization remains
an open controller boundary.

The mechanism passes, but the low-gain walking profile remains rejected. First
NormalFallback is unchanged at tick 863; full-run root RMS is 17.026 m and
attitude reaches 120.38 degrees. The candidate remains below the 5 ms p99 gate,
while the separate localized-handoff control exceeds it. Defaults and
authority are unchanged. The next controller slice is continuous recovery from
persistent NormalFallback with post-touchdown root/attitude tracking, not more
per-call solver work or weaker contact constraints. See
[`G1_BOUNDED_CONTACT_CONTINUATION.md`](../benchmarks/results/g1-bounded-contact-continuation-r269/G1_BOUNDED_CONTACT_CONTINUATION.md).

## Prior CPU checkpoint — r268 bridges the native reference into integration

R268 exercises the retained four-step Rust LIPM reference through the bounded
integrated floating controller with no policy and no physics simulator. The
standalone boundary now consumes authored root position, velocity, and
acceleration jets independently instead of substituting the CoM derivatives.
An optional retained morphology witness initializes q/v, root translation, and
root twist after exact reference correlation plus the existing 10 mm foot / 30
mm CoM certificate. Future oracle WBC outputs are neither loaded nor executed.

Initialization-only extends the earlier native-reference clean prefix from 265
to 501 ticks and reaches the authored liftoff at tick 300 with 0.003 mm root
error. Its latest replay p99 is 4.884 ms and accepted hard dynamics/contact residuals remain
below 1e-8, but support still releases before touchdown tick 529. The exact R54
oracle task stack fails at tick 188. A new allocation-free Rust scalar-jet path
can sample and time-warp the morphology q/v/qdd trace under the common cursor,
but that default-weight Preference profile fails at tick 439. A low-gain
`0.05` morphology profile carries the first support transfer through touchdown
to tick 863 with 1.84 cm root RMS, then fails later; it is still a causal
negative control, not an admitted default. All profiles are rejected;
defaults and authority remain unchanged. Release/free-body fallback now also
clears rejected contact residuals, force/effort margins, collision witnesses,
and task residual slots before integrating, so status-5 telemetry is explicitly
fail-closed rather than stale. The next slice is attitude/support
retention across the last 90 single-support ticks, not another solver-budget or
contact-threshold change. See
[`G1_NATIVE_REFERENCE_INTEGRATION.md`](../benchmarks/results/g1-native-reference-integration-r268/G1_NATIVE_REFERENCE_INTEGRATION.md).

## Prior CPU checkpoint — r267 localizes reacquisition without controller promotion

R267 makes contact release ownership per target: an unsuppressed requested
target can leave session-level free-body mode even if a different failed target
remains latched out. The retained trace exercises target 0 Precontact at tick
428 while target 1 stays suppressed. Target 0 never locks and releases at tick
460 after 116 unsupported ticks, so the mechanism closes a global-latch bug but
does not claim recovery.

Artifact-only cap and active-set sweeps reject both obvious shortcuts. Cap 768
first retains locked contact at tick 300 but reaches 22.9 ms. Equality-first
repair needs 16 active-set iterations and retains contact only through tick
329. Its native 600-tick row has 270 contingency ticks, 23.264 ms p99,
33.291 ms maximum, and 51 twenty-millisecond misses. R165 independently moved
11 fall boundaries earlier with this seed rule, so the default remains off.
The next functional slice is causal single-support reference compatibility,
not more solver work or a late-fall reset. See
[`G1_FLOATING_REACQUISITION_LOCALIZATION.md`](../benchmarks/results/g1-floating-reacquisition-localization-r267/G1_FLOATING_REACQUISITION_LOCALIZATION.md).

## Prior CPU checkpoint — r265 rejects the frozen profile on its one-shot holdout

R265 consumes R264's single authorized holdout with no refit, widening, retry,
or authority path. New medium elliptic/Euler and stiff elliptic/RK4 laws use
predeclared offsets 330,000/340,000 across 96 states and 1,440 MuJoCo steps.
The mechanism passes: the pinned model/profile hashes match, 88 rows are inside
the frozen distance gate, unsupported rows retain baseline, semantic replay is
exact, timed Rust allocation and MuJoCo warnings are zero, and profile/WBC p99
are 0.996 µs/3.253 ms.

The profile is rejected. Supported consequence coverage drops to
92.045% component and 85.985% aggregate. Joint-position pressure has a 27.992
worst miss, headroom 2.834, and aggregate 13.964. The sole nonzero selection is
medium-Euler row 20 candidate 1: its aggregate improves 0.732 but tilt and
angular-rate pressure regress 0.0119/0.00349. That candidate was part of the
frozen family but never selected in the spent rehearsal. The next construction
must therefore freeze per-action selection support in addition to state
distance/calibration. R265 is consumed and cannot be rerun; authority remains
closed. See
[`G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT.md`](../benchmarks/results/g1-residual-prototype-plant-holdout-r265/G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT.md).

## Prior CPU checkpoint — r263 floating contact-release recovery

The floating session now treats every unsolved contact result, including
bounded `MaxIterations`, as non-executable. It retries once with normal-only
contact rows, then records a fixed-capacity per-target release latch and clears
the measured support phase/landing anchors before retrying without contacts.
The latch prevents the same failed contact from being rebuilt on every 50 Hz
tick. If no contact solve is executable, Rust advances with a bounded
free-body fallback (angular/joint damping plus gravity, clamped to the current
acceleration envelope) and keeps contingency status visible; it never turns an
unfinished contact solve into authority. The latch clears only on reset or a
schedule edge that actually releases the target. A 600-tick r262→r263
comparison removes the 171-tick exact state stall, records one release
contingency, zero 20 ms misses, and 4.757 ms p99 in the latest replay. This closes the interactive
freeze/reset failure mode, not the functional floating-walk, MuJoCo tracking,
thermal, or authority gates. See the [r263 recovery report](../benchmarks/results/g1-floating-contact-release-r263/G1_FLOATING_CONTACT_RELEASE_R263.md).

## Current live feedback checkpoint — r235 measured-state lifecycle

The live plant now has an explicit ownership contract. Green `TARGET` remains a
state-local WBC preview query; orange Ctrl-drag is a bounded, provenance-carrying
world-frame wrench; and the orange wireframe/dashed rig are measured MuJoCo
state. The persistent Rust WBC consumes the latest MuJoCo qpos/qvel/root state
at 50 Hz while MuJoCo advances five 4 ms physics steps between commands. It
does not integrate a private plant proxy or overwrite measured state.

`plant_pause` releases the wrench, freezes MuJoCo time and state, skips WBC and
physics, and keeps a heartbeat. `plant_resume` starts from the frozen measured
state without resurrecting a wrench. `plant_reset` clears the wrench, rebuilds
balanced standing, increments `reset_epoch`, and preserves paused ownership
when requested while paused. Worker, protocol, and UI-contract tests cover
freeze/heartbeat, resume, reset-while-paused, and paused-load rejection; the
same lifecycle passes localhost and the single public Cloudflare WebSocket
end-to-end smoke. This admits state ownership and fail-safe lifecycle semantics, not green-target
plant realization, policy recovery, authentication, thermal calibration, or
hardware authority. See
[`docs/LIVE_PLANT_INTENT_WRENCH_CONTRACT.md`](LIVE_PLANT_INTENT_WRENCH_CONTRACT.md)
and the hosted [r235 lifecycle report](/LIVE_MUJOCO_FEEDBACK_LIFECYCLE_R235.html).

## Current live editor checkpoint — r234 measured MuJoCo geometry/state

The ground grid now comes from MuJoCo's streamed plane point and normal rather
than an implicit browser constant. TARGET overlays the measured collision body
as an orange wireframe, plus the dashed measured rig, contacts, physical CoM,
and its projection onto the simulator plane. Green remains the authored WBC
query. Measured collision vertices below the plane turn red.

The 50 Hz plant record now includes actual actuator force, generalized
actuator/passive/bias force, scalar constraint force/position/velocity, solver
forward/inverse residual, exact constraint-row count, and summed ground-normal
load in addition to the existing 250 Hz MuJoCo state. Worker, UI-contract, and
live WebSocket regressions pass. A one-metre-down preview still retains
0.250000 mm collision and 0.162025 mm exact visual clearance. This admits
ground/state observability, not green-target plant realization, and the browser
frame-time gate remains unrun because no in-app browser is attached.

## Current CPU checkpoint — r238 stage-force RK4 passes; mixed transfer rejects

Rust now implements a genuine four-stage generalized RK4 path inside the
model-owned contact query. The initial free acceleration is converted to a
held generalized force. Each RK stage owns its floating pose/tangent, rebuilds
sphere support geometry, floating bias and inverse dynamics, the factored mass
matrix and complete Delassus operator, point velocity/free acceleration, and
stage-local collision membership. The final pose, tangent, and contact impulse
use classical RK4 weights. Scratch is construction-owned; the hot query does
not grow storage. Analytic constant-force motion and a mid-tick crossing that
Euler intentionally misses both have direct Rust regressions.

On spent R231 data, the equation-driven change improves hard-RK4 exact active
sets from 24/48 to 45/48 and missed actual points from 22 to 1. R232 then uses
prediction change only: 32 sweeps fail the frozen 2% refinement gate at
2.496%, while 64→128 refinement is 0.600%. It freezes 64 sweeps at 4.34 ms p99,
bitwise repeat, and zero timed Rust allocation. Completed R231 labels are
opened only after selection and cannot tune the result. Direct final-tangent
scoring gives the spent hard law 46/48 coverage and 0.669/0.125/8.262 fitted
width; the old final-impulse-through-initial-response proxy is retained only as
a named diagnostic.

R233 spends two new RK4 laws at disjoint offsets 130,000/140,000. The
medium/elliptic law has 46/48 frozen sample coverage and 47/48 exact active
sets; the hard/pyramidal law has 44/48 coverage and 44/48 exact active sets.
Both meet the 5 ms deadline at 3.32–3.38 ms p99 with repeat and allocation
gates intact, but fitted angular/linear/joint widths remain
0.285/0.043/10.446 and 0.699/0.110/11.248. The generic RK4 mechanism stays;
the accuracy profile and authority are rejected without fitting the spent
R233 labels.

R236 performs that spent-label localization without changing any profile.
Medium's 10.446 rad/s joint width is unchanged on its exact-active-set subset;
hard's exact-set subset still requires 10.618 rad/s. Worst coordinates are all
ankle pitch/roll. Both implementations already share four authored 5 mm sphere
contacts per foot, while the right-foot normal-impulse RMSE is 0.730/1.957 N·s
and pitch-moment RMSE is 0.0311/0.0802 N·m·s. The next construction is
stage-local force and within-foot wrench distribution/reference solver
semantics, not primitive geometry or a scalar activation mask. R236 takes zero
new physics, policy, controller, selector, or plant steps and is ineligible for
selection or authority.

R237 preserves that historical model ABI as id 3 and adds a distinct
model-only current-stage-force ABI id 4. The new path cancels the legacy local
solver's extra full-tick gap advance, so each RK derivative evaluates the
positive-reference law at its current stage state. Scalar APIs reject both
generalized ids, and caller-owned scratch still covers every stage refresh.
Prediction-only 64→128 refinement is 0.801% of the useful-width gate; 64 sweeps
freeze at 4.06 ms selected p99 (4.18 ms at 128 sweeps) with bitwise repeat, zero timed Rust allocation,
and 122/122 semantic arrays exact on an independent rerun. Completed R233
labels remain inaccessible until after selection.

R238 then creates two new law families at disjoint offsets 150,000/160,000.
The RK4/id-4 row covers 48/48 under the frozen box at
0.122/0.018/5.191 angular/linear/joint width and 2.82 ms p99. The
implicitfast/id-1 comparator covers 46/48 and needs
0.213/0.045/21.255, so the conjunctive mixed-integrator profile is rejected.
Both rows repeat bitwise, allocate zero timed Rust bytes, meet the 5 ms
deadline, and reproduce all 62 semantic arrays. The stage-force construction
is retained; label-independent implicitfast/contact-update transfer remains
the CPU accuracy gate. No R224 selection, plant non-regression, or authority
follows the failed combined holdout.

R239 isolates a second integration-category error using spent R238 labels only.
MuJoCo's documented implicit/implicitfast velocity Jacobian excludes constraint
forces `Jᵀf(v)`; contact friction remains a constraint-space reference
acceleration. Mapping implicitfast onto Bonesaw's local implicit contact damping
was therefore incorrect. Diagnostic ABI 0 removes sample 26's predicted-only
point and cuts joint width from 21.255 to 5.077 rad/s, while sample 10 remains
an exact-active-set distribution tail. R239 is ineligible for selection and
replays 33 semantic arrays exactly.

R240 preserves every historical mapper and adds the equation-level constraint-
RHS mapper: non-RK uses ABI 0; RK4 uses current-stage ABI 4. Prediction-only
128→256 refinement is 1.980% of the useful-width gate, freezing 128 sweeps at
3.34 ms selected p99 with repeat, zero allocation, and 136 exact semantic
arrays. Only afterward do spent R238 scores reopen.

R241 spends new crossed cone/integrator laws at offsets 170,000/180,000. The
elliptic RK4/id-4 row transfers strictly at 48/48 with
0.100/0.009/3.120 angular/linear/joint width and 3.56 ms p99. The pyramidal
implicitfast/id-0 row covers 47/48 and needs 0.124/0.025/17.138; its lone miss
predicts three unloaded points. Both rows meet deadline, repeat, allocation,
and 62-array replay gates, but the combined profile remains rejected. The next
CPU mechanism is label-independent non-RK within-tick activation and
within-foot wrench distribution—not RK stage timing or integrator-enum mapping.

The model-only `generalized-implicit-stage-force` ABI id 5 is retained as an
opt-in diagnostic hypothesis. It performs one local implicit contact update at
each refreshed generalized-RK stage while preserving historical implicitfast
id 1 and every prior mapper. Its atomicity and positive-impulse regressions
pass, but it has not been promoted or evaluated on a fresh holdout.

R242 rejects the next opt-in non-RK activation hypothesis: ABI 6 admits a
positive-gap point only when its one-state-step predicted gap crosses while the
current normal velocity closes. On spent R241 labels it predicts 19 unloaded
points, covers 77.083%, and fits 1.478/0.275/22.719 angular/linear/joint width;
the historical ABI 0 boundary guard remains unchanged. R243 evaluates an
edge-coordinate pyramid cone (`N ± μT`) against the causal R241 replay, while
retaining Cartesian ABI 1, and freezes its first causal profile at 64 sweeps.

R244 is the historical untouched edge-coordinate follow-up at offsets
190,000/200,000. Its implicitfast row covers 47/48 and its RK4 row exceeds the
five-millisecond gate. It remains immutable failed evidence.

R245 corrects the cross-profile loophole by forcing cone ABI 2 for both
integrators and freezing them independently before opening completed labels:
ABI 0 selects 64 sweeps and ABI 4 selects 32. Rust now evaluates spherical
friction motion at the instantaneous surface material point (including
`ω × r`), retains centre-minus-radius gap geometry, and computes reference
acceleration from the current collision-boundary gap. Incremental `D·λ`
updates remain caller-owned and allocation-free.

R246 generates two new pyramidal laws at disjoint offsets 230,000/240,000 only
after that freeze. Both implicitfast/id-0 and RK4/id-4 rows cover 48/48 with
exact active sets. Widths are 0.059/0.007/4.949 and 0.065/0.010/2.239
angular/linear/joint; p99 is 0.871/3.399 ms. Both repeat bitwise, allocate zero
timed Rust bytes, meet five milliseconds, and reproduce all 62 semantic arrays
in an independent process. The evaluator profile is promoted. R224-bounded
plant action, non-regression, and authority remain closed.

## Current CPU checkpoint — r229 localizes stiff activation without promotion

R229 reads the completed immutable R228 labels explicitly, so it is a
diagnostic only and cannot construct a threshold, select a profile, or admit
authority. All six rejected stiff rows predict at least one foot point that
MuJoCo leaves unloaded, and no actually loaded point is missed. Predicted-only
points are not sufficient evidence, however: they also occur in 8/42 covered
stiff rows and 13/48 covered soft rows.

Summed predicted-only impulse strictly separates this spent corpus (0.176 N·s
maximum covered versus 0.241 N·s minimum uncovered), but it is not an eligible
threshold. A label-oracle mask that deletes those impulses without re-solving
coupling repairs four of six misses and makes states 100009 and 100037 much
worse, for only 46/48 coverage. A separately constructed label-free first
ballistic cohort uses the authored five 1 ms ticks; it is a subset of eventual
MuJoCo load in 6/6 misses and exact in 5/6. It identifies the first event but
cannot predict later contact growth. Ten semantic arrays, normalized metrics,
and the report replay exactly. The next construction must causally advance
contact activation, geometry, velocity, and full Delassus response and re-solve
the coupled set; R228 remains rejected.

## Current live boundary checkpoint — r225 typed external-load provenance

The `/plant-ws` protocol now requires every executable external wrench to carry
an explicit source, `declared_continuous_wrench` class, world force frame, and
world application-point frame. Allocation-free Rust core validation keeps that
class distinct from `measured_impact_impulse` and `unobserved_model_reserve`;
the server permits only browser-operator and evaluation-harness sources, then
the Python MuJoCo worker independently revalidates the same contract. Plant
state echoes the accepted provenance and reports impact/reserve as separate,
explicitly unavailable records rather than silently treating either as zero.

The public Cloudflare audit rejects missing provenance, impact-as-command, and
reserve-as-command without mutating the worker command or interrupting the
stream. Both accepted sources echo exactly, release normally, and leave zero
MuJoCo warnings. This is a transport/provenance admission, not authenticated
operator identity, impact estimation, or model-error calibration.

## Current live editor checkpoint — r223 explicit preview/plant ground state

The editor now retains two explicitly labelled states while TARGET is active:
green is the authored state-local WBC preview and a dashed orange rig is the
measured 250 Hz MuJoCo plant under its independent 50 Hz Rust command stream.
The z=0 ground is a filled plane, contacts render in either mode, and the
preview reserves 0.25 mm collision clearance. An end-to-end local/public gate
transforms all 41 authored visuals and 25 STL instances from streamed frame
records. Reachable and one-metre-down/clamped base requests retain exactly
0.250000 mm collision clearance and at least 0.135636 mm visual clearance.

The plant record now includes root pose/twist, joint state, actuator effort,
generalized acceleration, generalized constraint force, contact/penetration,
energy, solver iterations, and warning count. Worker and WebSocket tests retain
250 Hz physics, 50 Hz WBC/stream, five substeps, finite vectors, zero warnings,
lease expiry, fall report, next-step simulator reset, and fresh reconnect. This
does not execute green base intent in MuJoCo and does not replace the unrun
browser frame-time gate.

## Current CPU checkpoint — r228 rejects the frozen profile on fresh laws

R228 generates two new reset-every-transition MuJoCo law/state corpora at
offsets 90,000 and 100,000 after R227 froze 32 microsteps, 32 sweeps, the
label-free momentum cap, and 0.449/0.065/9.320 angular/linear/joint width. The
soft/pyramidal/Euler law passes 48/48. The stiff/elliptic/implicit-fast law
passes only 42/48 and 98.9943% of components. Both laws retain deadline,
bitwise-repeat, zero-allocation, and exact 46-array replay witnesses; query p99
is 0.612/0.418 ms. The six stiff misses all predict additional near-
simultaneous foot-point activation absent from measured reference load, and
fitting them would require 1.926/0.338/46.976 width.

The frozen profile is therefore rejected and not eligible for terminal
selection. Retain the generic coupled mechanism, do not tune the six fresh
misses back into R227, and independently derive the next stiff-contact
activation/order formulation with state-dependent geometry/Delassus evolution
before another holdout. No selector, plant action, or authority is admitted.

## Current CPU checkpoint — r227 freezes coupled work from causal convergence

R227 fixes a label-free momentum cap and selects the coupled construction
profile using only differences between causal predictions. The least-work row
must change by at most 2% of each useful-width gate when projection sweeps
double, at most 20% when microsteps double, and remain below a 5 ms CPU
deadline. This freezes 32 microsteps × 32 forward/reverse sweeps: 1.200% sweep
refinement, 16.858% substep refinement, and 0.637 ms worst-law p99. Completed
impulse and generalized-residual labels are accessed only after selection.

The already-rejected R221 labels then show 48/48 coverage under each law at
useful width, but they cannot promote the profile. All timed Rust calls allocate
nothing and two complete runs reproduce 192 semantic arrays exactly. R228
preserves the frozen substeps/sweeps/cap/residual contract on new laws and
rejects it. No selector, plant action, or authority is admitted.

## Current CPU checkpoint — r226 uses the documented positive reference law

R226 replaces the construction-only constant-impedance/reference scaling in
R220 with the documented positive time-constant/damping-ratio law, complete
position-dependent impedance spline, explicit refsafe clamp, first-order
tangent decay, and declared circular/pyramidal friction sections. Generic Rust
types causal free point acceleration separately from contact velocity and
offers both independent effective-normal-mass response and a fixed-work
complete-Delassus projected distribution. Both queries are allocation-free;
PyO3 preflights the complete batch before output mutation. Core/Python
evolution, invalid-input atomicity, bitwise repeat, and zero timed allocation
pass.

The construction audit reuses the already-rejected immutable R221 labels, so
it cannot admit authority. It makes 96 causal prestate forward-dynamics queries
to construct the free-acceleration witness and takes zero physics, integration,
policy, or controller steps. Six complete-Delassus construction rows cover
48/48 samples under both laws at useful width and the explicit 5 ms CPU gate.
A representative 32-step/free-acceleration row fits angular/linear/joint width
0.095/0.030/6.131 and 0.449/0.065/9.320 at 0.463/0.634 ms p99. The reduced
integration schemes preserve the authored law's explicit/implicit character
but do not reproduce MuJoCo's generalized RK4 or implicitfast integrators.
Retain both generic mechanisms and the causal boundary, but freeze no empirical
profile and spend no untouched holdout. The next transition checkpoint must
derive a convergence/profile rule independently of these rejected labels,
then pass a new holdout before any selector or authority claim.

## Current CPU checkpoint — r224 propagates the full velocity box to terminal pressure

R224 adds a generic allocation-free Rust outer bound over the velocity
coordinates consumed by terminal consequence. Ballistic time is monotone-
bounded from the supplied vertical-speed interval. Root tilt/rate and joint
position/velocity are then propagated across the whole impact-time interval;
all pressure and aggregate fields are upper bounds, while joint headroom is a
lower bound. The componentwise box carries neither probability nor a claim
that every Cartesian corner is reachable. A 729-point core oracle, generic
PyO3 containment/atomicity tests, bitwise repeat, and zero timed allocation
pass.

On immutable R221 contact-only states, center scoring's 7/1 mid/hard false-safe
crossings become 0/0 under box scoring, with zero bound violations or
false-safe rows among completed states inside the terminal projection. Query
p99 is 1.388/1.439 µs. This does not restore authority: source sample coverage
is still only 91.667%/64.583%, contact-only terminal-projection coverage is
91.667%/39.583%, and the broad box rejects 10/48 and 2/48 safe completed
states. The generic consequence bound is retained; the R220/R221 profile,
selector, and command authority remain rejected. The remaining CPU authority
work is a tighter independently motivated transition set, an untouched
holdout, and only then plant non-regression for a state-conditioned action.

## Current CPU checkpoint — r247 keeps terminal candidate selection diagnostic

R247 adds an allocation-free PyO3 boundary that scores and conservatively
selects exactly three R224 componentwise velocity-box candidates. All rows are
preflighted before diagnostics or selection outputs are written; the timed
repeat reports zero Rust allocation, and the Python contract covers both the
improving candidate and a late invalid-row atomic rejection.

The policy-free, simulator-free action audit runs three fixed desired-
acceleration laws through exact G1 floating WBC on all 96 spent R246 states.
All 288 primary solves admit, the WBC and selector stay below five milliseconds
p99 and allocate zero Rust bytes, the selector chooses 85 zero-acceleration / 8
velocity-damping / 3 neutral-recovery candidates with zero component
regression, and 34/34 non-timing arrays reproduce bitwise in a fresh session.
Per-sample access is limited to root height and model-predicted contact
activation. The uncertainty width was already fit on spent R246 labels, so
this freezes the candidate family for a fresh plant A/B; it is not holdout or
authority evidence. No selected acceleration or torque is applied. The next
gate is a new MuJoCo baseline/candidate non-regression matrix without retuning.

## Current CPU checkpoint — r248 rejects held-torque plant realization

R248 executes the frozen R247 family without retuning on the primitive
two-probe G1 fixture, two new pyramidal laws, and offsets 250,000/260,000. Every
state forks before execution into a zero-generalized-joint-effort baseline and
a selected-torque candidate held for five 4 ms MuJoCo steps (one 20 ms / 50 Hz
WBC update). The mechanism passes: 288 WBC queries admit, WBC and selector
allocate zero Rust bytes, WBC p99 remains below five milliseconds, 960 MuJoCo
steps complete, and neither branch emits a warning. Strict plant non-regression
fails at 14/48 implicitfast and 12/48 RK4 rows. Joint-position pressure remains
dominant. R248 therefore rejects the action
profile and keeps authority closed. A bounded actuator bandwidth/slew
realization must be designed on this now-spent evidence and frozen before a
new no-retuning plant holdout.

## Prior CPU checkpoint — r264 freezes a useful cross-law residual profile

R264 adds a fixed-capacity residual-prototype boundary to `bonesaw-core` and a
batched caller-owned PyO3 adapter. A validated profile contains at most 64
features, 256 prototypes, and 16 causal cells. Each query performs a bounded
same-cell nearest search, rejects out-of-envelope distance, composes asymmetric
component/aggregate calibration around the predicted plus prototype residual,
and invokes the existing conservative three-candidate selector. Candidate zero
must remain the exact available zero box; validation rejects any baseline
residual or widening. The hot path allocates no memory.

The Python freeze consumes checksum-pinned R258/R254 state and diagnostic
artifacts only. Its 55-coordinate full observed-state profile covers 192/192
leave-one-law-out component and aggregate rows and retains three nonzero
candidate-2 selections, all actually strictly nonregressing and improving.
Rust matches every Python nearest identity with 2.84e-14 maximum squared-
distance error, repeats all non-timing output exactly, allocates zero timed
bytes, and records 1.23 µs p99. Broad 38.317/19.137 worst component/aggregate
extensions keep 189 rows at baseline. The profile was frozen for exactly one
untouched new-law/offset holdout, now consumed by R265; no policy, plant action,
default change, or authority is admitted. See
[`G1_RESIDUAL_PROTOTYPE_PROFILE.md`](../benchmarks/results/g1-residual-prototype-profile-r264/G1_RESIDUAL_PROTOTYPE_PROFILE.md).

## Prior CPU checkpoint — r262 bounds floating feasibility work without functional admission

R262 adds an explicit finite `FloatingWbcSession` feasibility policy and a
reproducible Python sweep over 8/16/32/64 total Dykstra projection sweeps. The
unbounded 600-tick floating support-transfer trace misses the 20 ms 50 Hz
budget on 281 ticks at 276.8 ms p99. Every finite profile has zero 20 ms
misses, p99 below 5 ms, no failed/infeasible tick, and no contact-release
reset, but generic-build maxima remain above 16 ms. A five-process host-native
profile pins logical CPU 4 and combines the minimum eight projection sweeps
with two feasibility-polish iterations. It records 0/3,000 five- and
twenty-millisecond misses, 3.469 ms worst p99, 3.551 ms observed maximum,
72/72 exact non-timing arrays, and zero Python collections. This is fail-closed
budgeting: the unfinished hard solve never becomes executable and the existing
normal-contact contingency keeps state flow alive. The strict row remains
functionally red (354/600 contingency ticks plus large tracking/residual
errors), so the default, transfer behavior, profile admission, and authority
remain unchanged. See
[`G1_FLOATING_PROJECTION_BUDGET_PROFILE.md`](../benchmarks/results/g1-floating-projection-budget-profile-r262/G1_FLOATING_PROJECTION_BUDGET_PROFILE.md).

## Prior CPU checkpoint — r261 finds a useful source profile but rejects transfer

R261 replaces independent terminal-state coordinate boxes with 3–12 complete
residual exemplars per causal state group. A new Rust/PyO3 boundary keeps each
baseline/candidate pair correlated, retains candidate-dependent clearance and
vertical velocity, performs support-free propagation and exact consequence
scoring, envelopes at most 16 hypotheses, and invokes the conservative selector
without allocation. The Python evaluator takes zero policy steps, physics steps,
or plant actions. Immutable R258 source component/aggregate coverage reaches
100%/100%; five nonzero selections are all strictly nonregressing and improving,
and the recorded p99 is 28.10 µs.

The source-local candidate is not transferable. The unchanged spent R254
rehearsal reaches 85.301% component and 80.208% aggregate coverage; two selected
rows regress a component and one regresses aggregate score. The profile is not
frozen for a new holdout and authority remains closed. The next construction
must cover multiple spent law families while retaining useful nonzero actions.

## Prior evaluation checkpoint — r260 consolidates WBC behavior evidence

R260 adds one pinned Python manifest over the existing end-effector reach,
bimanual priority conflict, CMU 37/01 walking retarget, measured-feedback
floating moving-liftoff, 600-tick floating support-transfer stress, and Upkie
C++ WheelBalancer artifacts. It retains tracking RMS, p50/p99/p99.9 latency,
jitter, deadline misses, process memory/CPU/GC, solver work, contact/dynamics
residuals, status transitions, and ten execution-time windows. Fixed-base
reach/conflict, the moving-liftoff prefix, and Upkie canonical parity pass;
walking remains red at 5.549 cm foot RMS and full transfer remains red after
contact-mode change with contingency, deadline, and residual growth. This is
evaluation evidence only and admits no policy, plant action, or authority.
See [`WBC_BENCHMARK_MANIFEST.md`](../benchmarks/results/wbc-benchmark-manifest-r260/WBC_BENCHMARK_MANIFEST.md).

## Prior CPU checkpoint — r259 completes state composition but rejects the profile

R259 consumes the immutable R258 terminal-state corpus and fits baseline plus
candidate-minus-baseline residual boxes over complete terminal root
attitude/angular-rate, joint-position, and joint-velocity coordinates. The
R256 Rust boundary preserves shared baseline correlation; the evaluator runs
zero policy steps, physics steps, and plant actions. Source component and
aggregate coverage are both 100%, the paired selector is exact and
allocation-free at 0.409 ms p99. The smallest nonzero worst-component and
aggregate uppers remain 18.486 and 9.451, so every source row remains selected
as the exact-zero baseline. The unchanged spent R254 rehearsal reaches 98.553%
component and 97.917% aggregate coverage, with no nonzero selections.

The complete-state data seam is therefore closed, but the profile is rejected:
coverage alone cannot certify a useful action. No authority or new-law holdout
follows. The next construction must derive a causal nonzero paired profile (or
certify a complete within-step action tube) before spending one untouched law
and offset set.

## Prior CPU checkpoint — r258 retains the spent terminal-state corpus

R258 extends the already-spent R250 reset-every-sample replay so the terminal
state needed by the paired consequence boundary is retained rather than
reconstructed from endpoint velocity. For all 96 × 3 branches it stores root
clearance/vertical speed, roll/pitch and roll/pitch angular rates, every joint
position, and every generalized velocity. The extraction executes the frozen
480 WBC queries and 1,440 MuJoCo steps, emits zero warnings, and an independent
allocation-free Rust state rescore reproduces all 4,896 diagnostics exactly
(p99 0.524 ms in the recorded host run).

This is spent design evidence, not a holdout: no policy step, plant action, or
authority is emitted. R259's complete-state fit is recorded above; it has
complete source coverage but no useful nonzero selection, so the profile and
authority remain closed.

## Prior CPU checkpoint — r257 rejects endpoint-only state-tube freeze

R257 consumes only immutable R250 arrays and fits baseline plus
candidate-minus-baseline generalized-velocity residual boxes inside fixed
causal closing-speed/tilt groups and two declared source-law hypotheses. An
unknown-impulse-time interval lifts endpoint velocity uncertainty across the
20 ms transition. The direct R256 paired scorer stays allocation-free, exact,
and below 0.4 ms p99, with zero policy steps, physics steps, or plant actions.

The source profile is rejected. Component/aggregate coverage is
71.875%/38.542%, all 96 selectors retain zero, and the unchanged rehearsal on
already-spent R254 arrays reaches 72.454%/37.153%. Angular-rate,
joint-velocity, and effort coverage are 100%, but tilt, joint-position, and raw
headroom reach only 49.7%, 33.3%, and 48.3%; joint-position pressure misses by
as much as 34.853. Endpoint velocity labels do not identify within-step
terminal attitude and joint-position drift. No profile is frozen and authority
remains closed. The next data seam must retain terminal position/attitude state
or a certified within-step trajectory bound, rather than fitting consequence
residuals after the fact.

## Prior CPU checkpoint — r256 preserves paired state uncertainty

R256 adds an allocation-free Rust delta boundary over a shared terminal-state
tube. Every candidate is represented as the same uncertain baseline plus a
candidate delta, so consequence differences are bounded directly instead of
subtracting two independent score uppers. A fixed three-candidate,
four-hypothesis G1 batch repeats 500 times at 380.52/431.98 microsecond p50/p99
with exact replay, zero measured Rust allocation, an exact zero baseline, and
384/384 independently sampled paired points contained.

A separate replay composes R255 with authored directional contact and estimator
hypotheses on immutable R254 states. It performs 96 MuJoCo kinematic forwards
but zero physics or policy steps, 96 evaluation-only selector calls, and zero
plant actions. The
absolute tube contains all 288 stored terminal candidates with zero timed Rust
allocation. That 100% coverage does not freeze a useful profile: reserve width,
the q-drift lift, and paired lower/upper deltas must be fixed on spent evidence
before a new-law holdout. No action or authority is admitted.

## Prior CPU checkpoint — r255 composes complete terminal-state boxes

R255 adds an atomic, allocation-free Rust interval boundary over the complete
state consumed by the ballistic terminal-impact proxy: clearance, vertical
speed, roll/pitch, angular rates, joint positions, and joint velocities.
Pressure and aggregate consequence fields are upper bounds; raw joint headroom
is a lower bound. A G1-shaped 64-row PyO3 batch repeats 500 times at
50.72/75.50 microsecond p50/p99 with exact output and zero measured Rust
allocation. Dense Rust tests and 2,048 independently scored interior points
remain inside the reported bounds to floating-point roundoff.

The checkpoint executes zero policy steps, physics steps, selector calls, or
plant actions. It closes the interval-composition mechanism, not action
selection: independent candidate boxes erase shared candidate/baseline
correlation and therefore cannot establish paired improvement. No profile is
frozen and authority remains closed. The next CPU seam is a causal
shared-hypothesis or paired state tube built on this complete-state boundary.

## Prior CPU checkpoint — r254 rejects state-only paired transfer

R254 spends the exact no-refit plant gate declared by R253. Two new laws—
elliptic/implicitfast and pyramidal/Euler—start at untouched deterministic
offsets 310,000 and 320,000. Each of 96 states runs zero effort, realized
zero-WBC effort, and realized neutral-recovery effort for five 4 ms MuJoCo
steps, totaling 1,440 fresh physics steps and zero policy steps. The immutable
R253 residual boxes, group map, zero-regression gate, and 0.01 guaranteed-
improvement threshold are consumed without widening or fitting.

The mechanism passes with complete causal-group support, admitted WBC, exact
selector replay, zero timed Rust allocation, no MuJoCo warnings, and sub-5 ms
p99. The action profile does not transfer. Boxes miss both laws, with the
largest errors in joint-position pressure and raw headroom loss. Five rows
select neutral recovery; three improve aggregate consequence, but four regress
at least one component, two regress aggregate score, and only one is both
strictly component-nonregressing and aggregate-improving. R254 therefore
rejects the profile and keeps authority closed. The next CPU seam is a causal
contact-law/estimator-uncertainty terminal tube, not an enlarged empirical box.

## Prior CPU checkpoint — r253 freezes a paired consequence profile

R253 adds an atomic, allocation-free Rust selector over exactly three
candidate-minus-baseline terminal consequence boxes. Six candidate-dependent
deltas remain separate—tilt, angular rate, joint position, joint velocity,
actuator effort, and raw joint-headroom loss—while ballistic impact speed is
candidate-invariant and aggregate score has its own non-regression gate. A
second Rust boundary forms those boxes directly from four shared
candidate/baseline terminal hypotheses, preserving paired correlation instead
of subtracting independent envelopes.

Python reopens immutable R250 arrays without policy or physics, predicts the
post-20 ms fixed-effort state, and fits residual boxes only within fixed causal
closing-speed/tilt-sign groups. The neutral-recovery family freezes three
nonzero third-candidate rows: all three improve aggregate consequence and have
zero measured component regression. The velocity-damping family selects zero
on every row. This spent evidence freezes only an action profile for one fresh
no-refit contact-law/offset holdout; it does not admit authority.

## Prior CPU checkpoint — r252 adds passivity without claiming safety

R252 adds an atomic, allocation-free Rust trace that combines declared
bandwidth/slew response with a pointwise positive mechanical-power cap. On 96
spent states, four actuator-coordinate damping gains run twice over 4,320
MuJoCo steps: all applied power is non-positive, semantic traces repeat
bitwise, allocation stays zero, and warnings stay zero. Aggregate consequence
and kinetic energy usually improve, but every gain regresses at least one
terminal component on some rows. The mechanism is retained; no damping gain,
fresh action, or authority is selected.

## Prior CPU checkpoint — r251 isolates shared plant uncertainty

R251 performs no policy, physics, or selector query. It reopens only R250's
spent arrays and measures candidate-vs-zero paired velocity residuals under a
fixed causal partition of observed height, tilt, and root angular-rate signs.
The paired construction materially narrows the independent candidate/baseline
span while covering every source row by construction. That is design evidence,
not transferable coverage: Rust has no paired terminal-delta bound yet, no
action is frozen, and authority remains closed.

## Prior CPU checkpoint — r250 rejects the spent actuator action family

R249 replays the 96 selected R248 effort rows through Rust's persistent
first-order bandwidth/slew session at 20 ms / 50 Hz. Four declared profiles
include ideal, 50 Hz no-slew, 25 Hz / 1,000 N·m/s, and 12 Hz / 500 N·m/s. Each
row atomically resets realized effort; all four repeat bitwise, report zero
timed Rust allocation, and remain below the 5 ms query gate. The 25 Hz row
exercises three slew-limited coordinates. R250's stricter spent-state action freeze
then re-solves fixed realized effort and rolls out all profiles/families in
MuJoCo: no profile has zero component regression plus a useful nonzero action.
R250 therefore selects no fresh action and keeps authority closed; a
state-conditioned residual/action family is required before another holdout.

## Prior CPU checkpoint — r222 terminal consequence exposes point-score optimism

R222 adds a generic arbitrary-joint-count PyO3 batch over Rust's existing
ballistic terminal-impact proxy. Caller-supplied prediction/oracle state rows
are scored independently for impact time/vertical energy, 60 ms zero-
acceleration terminal tilt/rate, joint headroom/velocity, and separate harm
pressures. Every row validates before output mutation; paired queries repeat
bitwise and allocate nothing. The evaluator takes zero physics, policy,
controller, or integration steps and explicitly remains a proxy rather than
collision impulse, injury, recovery, or hardware safety.

On immutable R221 labels, the contact-only predicted/oracle ablation is
optimistic on 33/48 mid-law and 34/48 hard-law samples. It produces 7 and 1
false-safe terminal threshold crossings and 5/2 limiting-pressure class
changes. Harm-pressure absolute error is 18.246/24.135 p95/max for the mid law
and 108.500/204.899 for the hard law. All eight false-safe samples still lie
inside R220's componentwise velocity tube, demonstrating that scoring its
center cannot stand in for propagating the full interval through nonlinear
terminal consequence. Pair scoring is 0.976/0.816 µs p99, zero-allocation,
and all 12 non-timing arrays reproduce exactly. The rejected R221 point
predictor remains non-authoritative.

## Prior CPU checkpoint — r221 fresh laws reject the frozen compliant profile

R220 adds a fixed-substep compliant predictor in generic Rust. It carries
signed point gap and contact velocity through each microstep, computes explicit
Kelvin–Voigt normal impulse, projects tangent impulse to the circular Coulomb
disk, updates all contacts through the full state-local Delassus operator, and
advances gap with post-impulse normal velocity. Stiffness and damping are
explicitly mapped from effective normal mass, declared relaxation, and
impedance. Inputs validate before output mutation; core/Python evolution,
atomicity, determinism, and zero-allocation tests pass.

On immutable R218 construction labels, the selected 128-microstep profile and
its label-fit group residual box cover all 96 samples at
1.748/0.299/9.802 root-angular/root-linear/joint width. The retained query is
55.705 µs p99 with zero Rust allocation, and all 52 non-timing arrays reproduce
exactly. The 16× cap scale, microstep count, and residual box are explicitly
construction calibration, not hardware physics or authority.

R221 freezes every one of those choices before generating two new reset law/
state sequences at offsets 70,000 and 80,000. The mid/elliptic/implicit-fast
law covers 44/48 samples (91.667%, 99.7126% of components); the hard/pyramidal/
RK4 law covers 31/48 (64.583%, 98.1322% of components). Queries repeat bitwise,
allocate nothing, and remain at 52.646/53.573 µs p99, but strict coverage
fails. All 56 non-timing holdout arrays reproduce exactly. The mechanism stays;
the profile and authority are rejected without retuning from R221.

## Prior CPU checkpoint — r219 types finite estimator hypotheses and rejects scenario explosion

R219 adds a generic Rust envelope over an explicitly enumerated finite contact-
estimator hypothesis set. Every hypothesis owns contact velocity, impulse caps,
friction, restitution, and compliance while sharing the exact state-local
Delassus operator and generalized impulse response. The caller owns all output
and scratch buffers. Rust validates every scenario before touching output, so
a malformed late hypothesis is atomic; core and Python independent-solve
oracles pass with zero timed allocation. The API explicitly does not claim
coverage between the enumerated scenarios.

The zero-plant construction replay evaluates eight profiles on immutable R218
labels. The 59-scenario family includes nominal, global gap/normal/tangent,
per-contact signed-axis, and law-error rows. The 315-scenario family additionally
enumerates all `2⁸` signed normal-velocity patterns. Its useful-width edge
reaches only 81.25% strict sample coverage at 0.864/0.129/9.185 p95 width;
larger uncertainty reaches 92.708% at 1.923/0.339/23.306. Raising impulse caps
reaches 96.875% but stays near 22.5 rad/s joint width, still misses, and every
315-scenario p99 query exceeds the 5 ms control period (6.383–7.784 ms in the
retained run). All 48 non-timing replay arrays reproduce exactly. The finite-
hypothesis mechanism stays, but no profile advances to a fresh holdout. The
next predictor must model higher-order compliant evolution rather than expand
a discrete scenario list until it memorizes completed labels.

## Prior CPU checkpoint — r218 fresh-law holdout rejects the frozen coupled profile

R217 adds a generic PyO3 boundary over the existing allocation-free Rust
projected Delassus solve. All eight prospective G1 foot points are solved
together. The input is the causal end-of-step violation
`v_n + max(gap, 0)/dt`; tangential velocity remains physical, normal impulse
is nonnegative, and a circular Coulomb section is enforced. Declared contact
relaxation maps continuously to restitution and diagonal regularization. The
construction replay takes no physics, policy, controller, or integration step.
It improves compliant-law joint residual to 1.521 rad/s p95, but the rigid-law
residual remains 11.277 rad/s and therefore needs at least 22.554 rad/s of
centered joint width. The all-label R217 profile is construction evidence only.

R218 freezes that time-step law, its 16 sweeps, and R217's root/articulated
energy coefficients before generating 96 new reset transitions. The untouched
soft/pyramidal/Euler law and stiff/elliptic/RK4 law both obtain 100% sample and
component coverage with deterministic, zero-allocation Rust queries. The
frozen widths are nevertheless 2.420/0.142/35.754 and
13.170/0.785/175.201 rad/s-or-m/s at p95, against 2.0/0.5/10.0 gates. More
fundamentally, the stiff fresh-law predictor has a 17.498 rad/s unavoidable
joint-width floor. No holdout result was tuned back into the profile. The
generic coupled solver and split support stay; the calibration and authority
are rejected. The next CPU authority experiment must type contact-estimator
uncertainty or use a higher-order compliant contact model before freezing
another holdout.

## Prior CPU checkpoint — r216 causal center + split kinetic residual is still too wide

R216 replays the immutable r215 G1 corpus without policy, controller,
integration, or physics steps. One resultant point per foot is predicted only
from prospective four-corner gap/velocity, Rust model effective mass, declared
friction, and declared contact relaxation. Completed point impulses remain
scoring-only labels. Rust adds an allocation-free exact support function for
the Minkowski sum of independently budgeted root `[0,6)` and articulated
`[6,n)` kinetic impulse ellipsoids. Its block-matrix algebra matches an
independent inverse oracle; invalid energy is atomic; the generic PyO3 query
is symmetric and allocation-free.

Python fits a deterministic affine energy-fraction envelope on all 96 r215
labels for construction diagnosis. It covers 96/96, but p95 interval width is
15.890 rad/s root angular, 0.994 m/s root linear, and 141.487 rad/s joints.
The failure precedes residual geometry: the causal predictor itself has an
11.075 rad/s rigid-law p95 joint residual, so any centered component interval
covering it needs at least 22.151 rad/s p95 width against the 10 rad/s gate.
The exact completed-point oracle has only 1.775 rad/s joint p95 on that law,
localizing the remaining gap to causal contact-law realization/CoP evolution.
The split primitive stays; the all-label predictor/profile and authority are
rejected. A typed estimator uncertainty set is the other admissible route.

## Prior CPU checkpoint — r215 finite-patch coupling is physical but still unusable

R215 adds an allocation-free Rust support function for a finite-patch spatial
wrench. Each foot owns one resultant nonnegative normal impulse; passive-slip/
Coulomb tangential impulse, CoP moment from authored 85×30 mm half-extents, and
zero unmodeled torsion remain coupled to it. A generic PyO3 query composes that
set with the model-owned spatial `M⁻¹Jᵀ` response and r212's 50/10/50 continuous
acceleration reserve. Core/Python atomicity, exact-support, shape, determinism,
and zero-allocation tests pass. The support evaluates both passive-slip
saturation breakpoints as well as zero and maximum normal impulse; an explicit
regression rejects the tempting but non-conservative endpoint-only shortcut.

The profile is frozen before two new G1 state/law sequences. The compliant/
elliptic/RK4 law covers 48/48; rigid/elliptic/implicit covers 40/48 and 98.9943%
of components. P95 root-angular/root-linear/joint width is
3.529/0.655/878.035 and 3.180/0.634/923.910, versus 2.0/0.5/10.0 gates. The
corrected bound is 3.410 µs p99 at worst with zero Rust allocation. Coupling four
points removes one Cartesian product but full corner-of-patch moment
uncertainty still amplifies through ankle inertia. Mechanism stays; profile and
authority are rejected without holdout feedback. The next useful form needs a
causal center contact prediction plus a state-conditioned kinetic residual.

## Prior CPU checkpoint — r214 kinetic impulse geometry is much narrower but still misses

R214 adds an allocation-free Rust primitive for the zero-centered mass-metric
impulse set `pᵀM⁻¹p ≤ E₂`. Its exact component support is
`sqrt(E₂ (M⁻¹)ᵢᵢ)`, avoiding the coordinate box's sum of every absolute
inverse-mass column. Core tests compare every component with an independently
inverted floating mass matrix, reject negative energy atomically, and the
generic NumPy query remains symmetric and zero-allocation.

Before executing two fresh G1 laws and state sequences, the energy radius was
frozen analytically as `E₂ = f²m(gΔt)²` with `f=0.50`; no r212/r213 residual or
quantile determines it. On 48 medium/pyramidal/implicit reset transitions it
reaches 100% coverage. On 48 hard/pyramidal/Euler transitions it reaches
47/48 complete samples and 99.9282% component coverage; state 20,020 misses
`left_ankle_roll_joint` (16.164 rad/s residual versus 7.659 half-width). P95
width is 1.092 rad/s root angular, 0.075 m/s root linear and 19.966 rad/s
joints—roughly 69× narrower at the joints than r213's independent-point tube,
but still above the 10 rad/s gate and not strict. Kinetic machinery stays;
the profile and authority are rejected without tuning the miss back into it.

## Prior CPU checkpoint — r213 rejects independent point boxes across G1 contact laws

R213 applies r212's frozen 50/10/50 grouped continuous-acceleration reserve and
unchanged passive directional profile to the pinned 23-DOF G1. Each of 96
five-millisecond pre-impact samples resets independently. MuJoCo supplies 48
soft/pyramidal/RK4 and 48 stiff/elliptic/implicit transitions over eight
primitive foot points; no controller, policy, or successful-rollout selection
is present. Rust now exposes the directional transition bound on the generic
arbitrary-morphology session in addition to point/spatial/momentum queries.

Both laws reach 100% sample/component coverage, but useful width fails
conjunctively: p95 group widths are 6.660 rad/s root angular, 1.043 m/s root
linear, and 1383.925 rad/s joints versus frozen 2.0/0.5/10.0 gates. The eight
independent impulse boxes compound through G1's leg Jacobians. A separate
momentum sensitivity also covers every sample only at
34.881/0.917/336.739. All timed Rust calls allocate zero; the directional bound
is 2.486 µs p99 in the slower law. Generic machinery stays. Both profiles and
authority are rejected without feeding a holdout quantile back into either.
The next construction must couple the finite foot patch or carry a causal
spatial-wrench set; this is not a second simulator engine or hardware contact
calibration.

## Prior CPU checkpoint — r212 freezes a narrow Upkie grouped reachable set

R212 reruns only the allocation-free Rust directional bound over the immutable
274-sample r207 replay, taking zero physics, policy, or controller steps. The
original 5/5/50 reserve replays bitwise-identically at 98.120% retained and
100% fresh coverage. Raising only root angular to 50 closes four of five misses
but leaves an invariant 0.004526 m/s root-linear miss. The round 50/10/50
root-angular/root-linear/joint profile closes 266/266 retained and 8/8 fresh
samples at p95 width 10.137/1.041/58.020. Its Rust p99 is 0.491 µs with zero
allocation. Because the profile was assembled with knowledge of Upkie
calibration, this is construction evidence and was frozen—not authority—before
the r213 G1/contact-law holdout.

## Prior CPU checkpoint — r211 momentum model query crosses morphology

R211 moves the contact-transition response boundary out of the Upkie example
surface into generic `ContactTransitionModelSession`. Construction resolves a
fixed list of URDF frames and allocates point, spatial, dynamics and solve
scratch once. Point/spatial Delassus, generalized-momentum residual and full-
inverse-mass box queries write only to caller-owned arrays.

A policy-, controller- and physics-free Python oracle evaluates 64 deterministic
states each on pinned Upkie (6 joint / 12 generalized DOF) and pinned Unitree
G1 (23 / 29). Pinocchio 4.0 independently assembles each floating mass matrix,
reorders it to `[root angular; root linear; joints]`, and maps singleton
covectors, signed boxes and three-candidate residual batches. Maximum absolute
error is 2.984e-13 against a 1e-9 D1 gate. Upkie/G1 box p99 are 8.781/77.813 µs
and residual p99 are 6.500/52.119 µs, with zero timed Rust allocation. Generic
model machinery is admitted. R204/R210 residual calibration, a second contact
law, consequence, composed deadlines and hardware authority are not.

## Prior CPU checkpoint — r210 perfect contact accounting still needs a conditioned residual

R210 is the optimistic complement to R209. It replays the same immutable R206
arrays with zero physics and zero policy/controller steps, but first subtracts
the exact completed spatial-wrench response. Rust then maps leave-one-named-
case-out residual boxes through the full inverse mass. The completed wrench and
nearest support candidate remain disclosed labels; this cannot authorize an
online command.

The signed coordinate box reaches 99.624% retained and 100% fresh coverage,
but misses one `handle_forward_4n/drop10` right-wheel component by 0.58282
rad/s. Its p95 root-angular/root-linear/joint width is
28.683/3.370/50.823, failing strict coverage and both R204 root-width ceilings.
A group-symmetric root-angular/root-linear/joint box reaches 100%/100% only at
45.463/5.384/584.787, failing every R204 width ceiling. Residual query and
coordinate/group projection p99 are 6.717 and 14.198/8.181 µs with zero
allocation. The mechanism passes, but no profile or authority is promoted.

## Prior CPU checkpoint — r209 generalized-momentum tube, mechanism only

R209 adds an allocation-free Rust map from a signed generalized-momentum
covector box through the exact full inverse mass matrix into a generalized-
velocity interval. The paired Rust residual query computes
`M(q)(Δv_observed − Δv_predicted)`. Atomic validation, caller-owned storage,
determinism, and zero timed Rust allocation pass through the PyO3 boundary.

The evaluator performs zero physics and zero policy steps over the immutable
R206 replay. Signed leave-one-named-case-out boxes are fit either to the
nearest support hypothesis (an optimistic oracle diagnostic) or to all
declared candidates, then projected by Rust. All four tested profiles cover
100% of the eight fresh samples but only 98.872% of 266 retained samples. The
narrowest profile is still 49.056 rad/s root-angular, 5.691 m/s root-linear,
and 179.104 rad/s joint width at p95, versus R204's much tighter
9.687/0.991/58.020. Scaling by 1.25× does not change sample coverage and the
maximum miss remains above 55/s. Rust residual/projection p99 are
5.228/8.423 µs in the narrow profile. The covector→tangent mechanism stays;
the empirical tube is not causal online calibration, fails strict retained
coverage, has no second-morphology witness, and admits no authority.

## Prior CPU checkpoint — r208 physics-free compliance replay, no scalar winner

R208 adds an explicit dimensionless diagonal-compliance ratio to the Rust
coupled projected impulse solve. The physical post-contact velocity output is
still evaluated with the unmodified Delassus operator. Atomic validation,
determinism, fixed work and zero allocation are retained.

The 7-ratio × 5-sweep grid runs entirely over the immutable R207 NPZ: zero
MuJoCo steps and zero policy/controller steps. Ratio 10 gives the lowest
impulse RMSE (0.04374 versus 0.05087 N·s unregularized) but worsens LOCO
coverage to 94.526% and joint width to 143.833 rad/s. Ratio 0.1 slightly
improves retained raw coverage 8.647→9.023% and root residual width, while
maximum miss grows 62.118→63.675. Fresh raw coverage is unchanged at 62.5%.
The best maximum miss remains the one-sweep unregularized solve; the best LOCO
coverage remains an unregularized eight-sweep row. Compliance machinery passes,
but no scalar profile is promoted and authority remains off.

## Prior CPU checkpoint — r207 coupled Delassus response, convergence is not authority

R207 extends the model-owned contact response from six independent effective
masses to the complete symmetric two-wheel `J M⁻¹Jᵀ` operator. Rust performs a
fixed forward/reverse projected impulse solve with nonnegative bounded normal
impulse, directional passive caps, restitution, and a circular Coulomb section.
Core, PyO3, symmetry/diagonal, cross-contact, atomic-failure, determinism and
zero-allocation tests pass.

Across the frozen 274-sample matrix, cross-wheel coupling is 3.25% p95 but the
operator condition number is 4.73e9 p95. A point solve plus the unchanged
support/acceleration envelope covers only 10.219%. Increasing one→sixteen
sweeps raises impulse RMSE 0.05087→0.05445 N·s and worst generalized miss
62.118→156.499; the near-null contact modes are being fit more exactly while
the physical prediction degrades. One/16-sweep p99 are 0.444/3.478 µs; full
response p99 is 9.721 µs, all with zero Rust allocation. Leave-one-named-case-
out residual coverage peaks at 95.985% and remains label-calibrated diagnostic
evidence. The full replay is frozen for subsequent policy/physics-free
regularization sweeps. Machinery passes; authority remains off.

## Prior CPU checkpoint — r206 spatial wrench preserved, moment is not the missing residual

R206 integrates each wheel's complete signed force impulse and spatial moment
about world origin across the 1 kHz substeps, including contact free torque,
then translates it exactly to the causal prospective point. Rust emits the
twelve-axis `[moment XYZ; force XYZ]` response and complete spatial Delassus
operator. A separate allocation-free Rust query reports
`M(q)(Δv_observed − Δv_predicted)` as a generalized-momentum covector.

On 274 samples, exact force at the prospective point covers 83.835% retained;
adding exact spatial moment falls to 81.579%. Fresh coverage remains 62.5% and
maximum miss rises 2.598→5.317/s. Moment magnitude is 0.001003 N·m·s p95 and
minimum-candidate momentum residual is effectively unchanged
0.06636→0.06645 p95. Spatial response/momentum timing p99 are 12.426/6.773 µs
with zero allocation in the first isolated run; the augmented canonical repeat
measures 11.595/5.899 µs. Mapping both responses back through `M(q_pre)` and
comparing with MuJoCo's independently accumulated generalized constraint
impulse lowers p95 reconstruction residual 0.010120→0.009407 for the spatial
wrench. Thus spatial accounting is physically more faithful, while the point
approximation's better pose coverage is error cancellation. The mechanism and
replay are retained; completed wrench is label-only and no authority is
admitted.

## Prior CPU checkpoint — r205 contact-response localization, force-point answer rejected

R205 adds label-only contact geometry needed to localize R204's five misses.
The plant trace retains the signed world impulse and normal-impulse-weighted
contact position over every 1 kHz substep. The evaluator compares the same
support-hypothesis acceleration interval with no impulse, exact completed
impulse through the causal prospective-point response, and exact impulse
through a completed oracle-centroid response.

Support acceleration alone covers 10.526% of retained samples. Exact impulse
at the prospective point raises coverage to 83.835%; the oracle centroid is
worse at 81.579%. Fresh coverage is 62.5% for all three narrow decompositions.
Centroid displacement is 20.001 mm p95 and 21.702 mm maximum, while its maximum
component miss grows from 2.598 to 5.317/s. Response p99 is 8.178/7.766 µs and
zero-allocation. A single force point—even completed and oracle-selected—does
not represent the distributed wheel impulse. The next CPU layer must retain a
spatial impulse moment/contact distribution, while root/generalized-momentum
residual stays independent. No label-only field enters authority.

## Prior CPU checkpoint — r204 passive directional impulse tube, rejected for authority

R204 adds an allocation-free directional contact witness: tangential impulse
is limited by both Coulomb capacity and the declared passive slip-arrest demand
`m_eff |v_slip| + F_t Δt`. The plant trace now retains signed world-frame
wheel impulse components for label-only scoring. With R203's structured
acceleration reserve unchanged, the primary covers every measured impulse and
all fresh μ=0.02 velocity samples while shrinking p95 root-angular/root-linear/
joint width from 29.779/2.255/1154.415 to 9.687/0.991/58.020. Bound p99 is
1.237 µs with zero allocation.

Strict retained velocity coverage is only 261/266 (98.120%); worst miss is
0.13226 rad/s pitch in backward/drop10. Larger slip/load profiles widen the
tube but do not close the same root-momentum gap. The directional mechanism is
implemented and useful as a diagnostic projection, not admitted authority.
Next work is a coupled multi-contact/Delassus impulse response or separately
calibrated root-momentum residual, then another morphology/contact-law holdout.

## Prior CPU checkpoint — r203 continuous acceleration tube, authority gated

R203 adds an allocation-free componentwise generalized-acceleration interval
to the physical transition contract. For each coordinate Rust validates lower
and upper acceleration, evaluates all four products with the uncertain
nonnegative transition-time endpoints, then adds the unchanged contact impulse
box through the R202 model-owned response. Invalid intervals leave every output
unchanged; the NumPy boundary retains zero timed allocation.

The narrow primary ±5 m/s² root-linear reserve is intentionally retained as a
failure: it covers 265/266 original samples and all eight samples in a newly
introduced μ=0.02 row, but one backward/drop10 root-angular-y component misses
by 0.01254 rad/s. A separately predeclared structured ±5 rad/s² root-angular,
±5 m/s² root-linear and ±50 rad/s² joint row covers all 274 samples. Optimized
model/interval p99 are 8.046/0.851 µs with zero allocation. The structured row
adds only 0.05/0.05/0.5 units of five-millisecond angular/linear/joint width;
the much larger remaining 29.779/2.255/1154.415 p95 widths come from the
independent per-wheel friction box. Acceleration interval mechanics are now
implemented. Calibration on another morphology/contact law, coupled impulse
geometry, typed external load, terminal consequence, and end-to-end timing are
still required before authority.

## Prior CPU checkpoint — r202 model-owned contact response, authority gated

R202 implements the missing model-owned half of the physical transition
contract in `bonesaw-core`. `ContactTransitionResponseScratch` preallocates FK,
floating inertia, factor, point-Jacobian, right-hand side, and solution storage.
One query accepts prospective world points and orthonormal contact bases,
factors the floating mass matrix once, writes every `M⁻¹Jᵀ` column into a
caller-owned `[dof, contact, 3]` buffer, and reports directional effective mass.
The Upkie NumPy boundary performs the complete query with zero timed allocation.

The retained 266-sample plant construction audit evaluates all 12 generalized
velocity components and unions the four support hypotheses for the selected
terminal action. The primary total-mass-floor profile covers 264/266 complete
samples (99.248%), 99.937% of components, and every measured impulse. Both
misses are root-linear x in the 4 N low-friction case, maximum 0.01665 m/s.
Response/projection p99 are 7.697/0.771 µs. The interval is also deliberately
reported as too wide for authority: root-angular/root-linear/joint p95 widths
are 86.738 rad/s, 6.467 m/s, and 3294.670 rad/s. Model-owned response is now
implemented; continuous acceleration/external-load uncertainty, a fresh
morphology/friction/timing holdout, useful width, plant consequence, and the
independent deadline gate remain open.

## Prior CPU checkpoint — r201 physical contact-transition interval mechanism

Implemented and admitted state-locally:

- generic allocation-free Rust action generation for exact double-, left-,
  right-, and zero-support masks;
- model-derived current CoM and active support centroid, supported braking and
  inertial-tilt coordination, joint damping, and exact ballistic flight without
  a fictitious contact impulse;
- a caller-owned PyO3 boundary with explicit mode, mask, saturation, and timing
  evidence;
- a separate floating-WBC solve for every action, preserving typed dynamics,
  contact, friction, joint, acceleration, and effort admission;
- a 256-state policy- and physics-free Upkie corpus with 256/256 admissions,
  exact reordered/fresh-session replay, mirrored single-support action,
  `2.05e-11` maximum hard violation, `124.453 µs` WBC p99, and zero Rust
  allocation/Python GC.

R176 then exercises that action in the complete 20-case plant matrix with four
authority arms: retained r137, measured-contact control, non-executing shadow,
and causal selection, plus a fresh selected replay. The mechanism passes every
gate: the shadow is physically exact, selection occurs only after exact
double-support arming, exact observed support loss, and typed WBC admission,
replay is exact, all traces are finite, and timed Rust allocation/Python GC are
zero. The selector executes 1,703 ticks without a timeout extension, blend,
cached-command resurrection, reset, or rejected-row execution.

Controller promotion is rejected. Nine of ten retained green cases physically
fall and all ten lose qualification once 33 loop overruns are included; the
sole measured-contact green row is lost, and eight existing fall boundaries
move earlier. The next action must
preserve those mechanism contracts while changing the support-transition action
or its selection conditions until all green rows survive, no boundary moves
earlier, and the loop has zero overruns. Delay/noise/dropout transition coverage
comes after that non-regression gate. The r176 selector remains Python-owned
evaluation orchestration; its state transition must move into generic Rust only
after its semantics are fit for promotion. Hardware realization remains
separate.

R177 isolates why measured-contact control itself destroyed most of the r137
green envelope. The r139 filter correctly starts without hard-contact authority
and requires three positive samples for activation; r176 allowed the primary
WBC to execute the resulting startup/reacquisition no-contact program. An
evaluation-only preservation arm keeps the established double-support primary
separate while the exact observed mask drives only the contingency query and
selector. Its non-executing shadow is physically bit-exact to r137 in all 20
cases, and the selected candidate preserves all 10/10 retained green rows with
exact replay and zero timed Rust allocation/Python GC.

This is not promoted: preserved primary support has no current observation
authority. The selected action still advances seven r137 red boundaries by as
much as 3.295 s. The 200 Hz gate also remains red: r137, preserved shadow, and
selected arms record 642/645/202 misses respectively, with unequal red-case
trace lengths. The next implementation is an explicit generic Rust
enable/reacquisition and bounded retained-command state, followed by a
directionally conditioned support-loss action. Only then should observation
delay/noise/dropout be added.

R178 implements the replacement mechanism in `bonesaw-core`. Generic
`ContactProgramAuthority` state combines already-admitted primary and
current-support commands with the existing bounded command lease. A fresh
primary command can enter only when exact raw, stable, and hard masks agree.
An explicitly configured, independently admitted current-hard-support command
may enter during debounce only after prior authority exists. Otherwise a
pending transition can use only the bounded prior-command lease; startup
without a prior command withholds. Evidence loss, inconsistent masks, invalid
commands, expiry, and reordered ticks are non-executable. The caller-owned
PyO3 boundary uses fixed NumPy arrays and an allocation guard. Five focused
Rust tests and all 267 workspace tests pass. R180 wires this mechanism into the
plant adapter and supplies its first causal non-regression matrix.

R179 closes the orthogonal action-conditioning gate in r177's exact
preserved-primary scaffold. Only the first exact mask-0 transition is queried;
the evaluation lease is consumed on either verdict. Selection requires typed
current-support WBC admission, at least `0.10` improvement from the Rust
eight-knot forecast over inertial continuation, and achieved root angular
acceleration within `40 rad/s²`. The 20-case five-arm replay records 17
queries, 11 transfers, and six rejections. All 10 retained green rows survive,
no r137 fall boundary moves earlier, and diagonal, low-friction forward, and
handle-forward boundaries move later by `0.200/0.055/0.035 s`. Shadow and
selected replay are exact; timed Rust allocation and Python GC are zero; the
one-query WBC/forecast maxima are `143.902/0.521 µs`; candidate loop overruns
fall `642→592` and every green-row loop maximum remains below 1.113 ms. This
promotes the conditioned action only inside the scaffold. Plant-controller
promotion remains rejected until r178 current-observation authority replaces
the unauthorized primary-preservation arm without regressing these gates.

R180 performs that composition with a fixed-primary-program-effort realization
against the exact current hard-support mask. The retained plant profile uses
three explicit prestart contact observations, establishes primary authority
first, and permits fresh current-support authority during debounce only after
prior authority exists. A two-tick retained-command experiment changed red-row
consequences and was rejected; the evaluated profile sets the lease to zero
ticks and continuously re-admits the actual-support realization instead. Its
five-arm 20-case matrix exercises 1,227 fresh current-support selections,
including 381 transition ticks, with zero retained ticks. Candidate execution
is bit-exact to r137 in all rows, preserving all 10 green rows and all 10 fall
boundaries; replay, sparse-row semantics, allocation, and GC gates pass. The
mechanism is admitted, not the timing or hardware profile: the retained run has
nine 5 ms synchronous misses and a 5.036 ms worst controller call, while
delay/noise/dropout, mirrored transition, higher-fidelity plant, and hardware
evidence remain open.

R181 supplies the causal negative control and makes the supervisor-health
boundary explicit. Directly selecting a separately optimized current-support
action changes the terminal boundary or qualification in 16/20 rows. In the
admitted arm, the independent WBC instead fixes the already-effective r137
actuator effort, proves its achieved acceleration and hard feasibility under
the current support mask, and cannot change that effort. Rust then selects
16,982 primary and 1,227 current-support proofs across 18,209 ticks with zero
withheld, retained, or fallback ticks. Executed torque error is exactly zero;
every plant/command trace and fall boundary matches r137; maximum hard
violation is `2.77e-9`; proof-WBC p99/max are `108.011/128.713 µs`; and timed
allocation/GC remain zero. A current-support proof is not counted as a fresh
primary solve, so it cannot increase fall-safe freshness confidence. Semantic
composition is admitted. The synchronous and hardware profiles remain
rejected by 10 red-tail 5 ms misses, a 5.639 ms controller-call maximum, and
missing delay/noise/dropout, mirrored-transition, higher-fidelity, and hardware
evidence.

R182 supplies that delay/dropout/mirrored execution matrix. Python deterministically
delays acquisition by 5 or 20 ms, drops one 5 ms sample every 250 ms, drops
10 ms every 500 ms, and flips either contact bit for 5 ms every 250 ms. Seven
plant cases include exact-green nominal and sagittal recovery, mirrored ±1 N
lateral failures, handle application, and low friction. Rust receives sample
age and availability through the same observation contract used by authority.
Across 49 paired/replayed runs and 41,247 ticks, 234 unavailable samples always
withhold and emit zero torque; 847 physical/observed mask mismatches exercise
delay/chatter; exact replay, finiteness, allocation, and GC gates pass. All
three exact-green cases remain green. Robustness is rejected because seven
existing failure boundaries move earlier, worst `−1.625 s`, and 56 synchronous
loops exceed 5 ms. A calibrated age/dropout contingency with non-regressing
terminal consequence is now required before hardware authority.

R183 corrects the causal classification of the delay rows without weakening the
fault test. R182 primed zero-age samples and then introduced 20 ms-old runtime
samples, so Rust correctly rejected four backwards timestamps and withheld for
four or five startup ticks. The new age-aware prestart path advances the receiver
clock and primes three already-aged samples. Across seven cases and exact replay,
the monotonic stream is 20 ms old from its first runtime tick, produces zero
timestamp faults and zero withheld ticks, and is bit-exact to exact observation
for every plant, command, and terminal field. This is a property of r181's fixed-
effective-effort realization, not a generic claim about delayed contact. The two
r182 delay regressions are therefore latency-onset artifacts; five periodic-
dropout regressions remain real, worst `−0.920 s`. The synchronous profile also
remains rejected by 17 loop misses and a 5.569 ms controller-call maximum.

R184 then removes dropout phase as the remaining confound. Exact observation is
compared with 5 ms-per-250 ms and 10 ms-per-500 ms loss beginning either on
actuator tick zero or after one complete period of established authority. Every
unavailable tick withholds and emits exactly zero torque, all exact-green cases
recover, replay is exact, and allocation/GC remain zero. Consequence is still
rejected: ten case/profile boundaries move earlier. Established 5 ms gaps move
left/right/handle failures by `−0.685/−0.350/−1.245 s`; established 10 ms gaps
move mirrored/low-friction rows by `−0.155/−0.015 s`. One handle profile is
unsettled at six seconds. This proves zero torque is not a safe unavailable-
observation action even after normal enable; a separately admitted bounded
command is required. The synchronous run has 28 loop misses and a 6.087 ms
controller-call maximum.

R185 introduces a separate typed command path for that unavailable-evidence
case. `maximum_inexact_observation_hold_ticks` is independent of the existing
contact-transition lease; selection code 4, lease provenance, age, remaining
budget, and status all remain observable through the allocation-guarded PyO3
surface. A one/two-tick hold replays only the last already-admitted effective
effort, emits no contact-force witness, and expires to withheld zero torque.
The seven-case, eight-profile paired-replay matrix passes every mechanism gate:
dormant exact hold-0/1/2 streams are identical, all 274 holds reproduce their
preceding effort, and Rust allocation/Python GC remain zero. It promotes no
physical policy: the one-tick 5 ms hold leaves one left fall `0.040 s` earlier,
despite improving right and handle by `0.515/0.570 s`; 10 ms bursts regress by
as much as `1.655 s`. Timing is also rejected by 81 loop misses and a 6.845 ms
controller maximum.

R186 extends that exact mechanism with a non-compounding Q15 authority fraction
in Rust. Zero disables retention and exactly reproduces withheld semantics;
quarter fractions scale the prior admitted effort bit-for-bit. The evaluator
uses the same first-loss fraction for established 5 ms and 10 ms dropouts, so
it cannot select using future burst duration. The seven-case, 13-profile paired
matrix passes mechanism, replay, provenance, finiteness, zero-allocation, and
zero-GC gates. It promotes no global fraction: all of
`0/0.25/0.5/0.75/1` advance an existing fall or create a new one, and `0.5`
turns the recovering forward-4 N reference/5 ms row into a fall at 2.495 s.
The next CPU controller experiment must condition withhold/hold/brake/impact
authority on current state and a short no-contact forecast. The retained rerun
records 131 loop misses and a 6.955 ms controller maximum.

R187 tests whether a denser scalar grid merely missed a safe constant. It adds
Q15 gains `0.45` and `0.95` and, as an explicit negative control, lets the
evaluator classify 5 ms and 10 ms bursts separately. Both values pass the 10 ms
slice, but no value passes 5 ms and no value passes both classes. Because those
classes are indistinguishable on their shared first unavailable tick, this is
a future-duration oracle rather than a causal selector. Exact-observation
dormancy, Q15 scaling, withheld zero semantics, replay, finite execution, Rust
allocation, and Python GC gates pass. Global policy and synchronous admission
remain false; timing records 196 loop misses and a 7.182 ms controller maximum.

R188 removes the future-duration oracle. The allocation-free Rust selector
forces support unknown and scores Q15 authority `0/0.25/0.5/0.75/1` from the
current reduced root state and the last admitted command's achieved
acceleration, torque utilization, and joint headroom. Exact ties choose less
authority. Every unavailable tick is queried, exact streams are dormant,
matched 5/10 ms first-loss prefixes are identical, replay is exact, and the
selector maximum is `3.015–6.192 µs` across two independent full-process
repetitions, with zero Rust allocation/Python GC. Mechanism
passes; plant consequence does not. The selector creates a new backward-4 N
fall at 3.000 s and advances six existing fall boundaries, worst `−1.995 s` at
the handle. The synchronous profile also rejects 100–104 loop misses,
6.122–6.247 ms worst loops, and 5.697–5.866 ms controller-call maxima.
Reduced-score argmin is therefore a useful
diagnostic baseline, not executable authority.

R189 adds a minimum predicted improvement over zero authority. Rust owns the
finite nonnegative margin, exact argmin, lower-authority tie break, and final
withhold decision; the existing one-tick authority budget remains independent.
Eight points from zero through a withheld-equivalent endpoint pass all
mechanism gates. None passes strict 5/10 ms consequence. The best small margin
removes the new green fall but advances the left boundary by 0.620 s. Timing
records 192–235 misses and 5.699–6.645 ms controller maxima across three
repetitions.

R190 adds typed `FreshSupportFreeInexactObservation` authority. Current robot
state authors a flight-mode ballistic/damping request and an independent
zero-contact WBC admits it before execution. The candidate does not refresh
Primary age/health, emit contact force, or consume future dropout duration;
invalid input withholds without silently falling back. Mechanism passes with
`3.486–9.778/154.372–178.638 µs` author/WBC maxima across three repetitions.
Consequence rejects the action: left
boundaries improve 0.450/0.885 s, but two reference-green sagittal rows newly
fall and right/handle/low-friction rows regress, worst −1.935 s. The ordinary
runs record 113–132 misses and 5.779–6.261 ms controller maxima.

R191 adds a policy- and physics-free terminal consequence module in
`bonesaw-core`. Given a declared root-impact plane, observed root/joint state,
and already-admitted candidate accelerations, Rust computes ballistic
time/vertical specific energy plus separate terminal tilt, angular-rate, joint
position, joint-velocity, effort, and admission pressures. The conservative
three-way chooser uses withhold as baseline and admits retained or support-free
authority only when at least one component improves without any component
regression. PyO3 exposes fixed caller-owned buffers; Python owns Upkie model
adaptation and the plant corpus. Three complete runs each cover 62,817
first-run ticks and 481 terminal queries; mechanism, exact replay,
causal-prefix, typed execution, independent re-audit, allocation, and GC gates
pass with zero audit disagreement. The retained artifact selects
withhold/retained/support-free 337/20/124 times; observed online-selector and
re-audit maxima span 0.992–8.106/0.972–1.102 µs. Strict consequence
still rejects promotion: both
green sagittal rows survive and handle/5 ms improves 1.340 s, but left/right
boundaries move earlier by up to 0.680 s. The runs record 115–126 misses,
6.620–6.777 ms controller maxima, and 6.950–7.117 ms loop maxima.

R192 adds measured post-control root angular and joint velocities to the Python
plant trace and audits the r191 selected candidate qdd against the resulting
5 ms interval acceleration. All 481 samples are finite; the
337/20/124 withhold/retained/support-free samples each span all seven cases,
with zero Rust allocation and Python GC. Leave-one-case-out componentwise
training maxima cover 96.881% of complete samples and 99.142% of scalar
components. A 5% reserve reaches only 97.089% complete coverage and retains a
25.234× worst exceedance, so the global realization envelope is rejected and
does not enter authority. The same diagnostic pass localizes synchronous
timing to the authoritative 2,688-sweep Dykstra fallback rather than the
selector or speculative planner.

R193 models typed withhold through a no-contact fixed-zero-effort floating WBC
instead of assigning it zero generalized acceleration. The Rust dynamics
witness is admitted, nontrivial, replay-exact, and allocation-free; Python only
owns the causal A/B corpus and report. Across 40,308 first-run ticks and 259
independently re-audited queries, every mechanism gate passes and all nominal
and sagittal green rows survive. Plant consequence rejects the unchanged
componentwise chooser: handle/5 ms falls at 3.055 s versus 4.630 s exact and
5.970 s under r191. Its first changed decision is support-free→withhold because
support-free tilt pressure 0.862 exceeds the corrected baseline's 0.781 even
though joint-velocity pressure improves 2.472→0.000. The semantic correction
is retained; the universal Pareto ordering is not promoted. The run records 50
loop misses and 6.938/6.616 ms loop/controller maxima.

R194 measures that zero-effort prediction on the 179 intervals where typed
withhold actually executes. Unavailable contact observation does not imply
flight: the plant retains double support on 169 samples, has no physical
support on six, and has one-sided support on four. Prediction-to-realization
qdd error reaches 1643.7 rad-or-m/s² and the corresponding terminal-pressure
error norm reaches 28.200. Leave-one-case-out componentwise bounds cover only
93.3% of mirrored-right and 86.4% of handle samples, with a 20.831 maximum
pressure-bound exceedance. Offline Rust rescoring remains finite,
allocation-free, and at most 6.562 µs. The calibration is rejected; no-contact
is retained as one named hypothesis, while the fixed four-support-hypothesis
aggregator remains diagnostic pending plant-bracketing and consequence gates.

R195 implements and evaluates the fixed four-support-hypothesis alternative.
Each withhold/retained/support-free effort is realized under
none/left/right/double support, then the Rust componentwise envelope and typed
chooser run without allocation. Attempt, valid-selection, invalid-baseline
fail-closed, exact replay/re-audit, and first-loss causal-prefix gates pass
across 40,982 first-run ticks, 270 attempts, and 266 valid envelope selections. Direct
following-5 ms realization rejects the model: 23/266 selections (8.65%) are
componentwise bracketed; maximum envelope exceedance is 267.523 and maximum
measured-physical-hypothesis pressure-error norm is 271.247. Two low-friction
profiles safely withhold on four ticks with invalid baseline hypotheses. Plant
consequence remains red (handle/5 ms is 1.245 s earlier than exact), as does
synchronous timing (70 misses; 6.578/5.793 ms loop/controller maxima). The
hypothesis set is retained as diagnostic structure only.

R197 adds opt-in plant-only impulse witnesses without changing controller
semantics. Normal/tangential wheel impulse and generalized constraint impulse
are accumulated over each complete five-substep control interval; disabled
traces remain exact zero and enabled traces replay bit-for-bit. Across the same
266 valid terminal selections, the measured physical support mask is the
lowest-qdd-error mask 91.35% of the time, but no realized qdd vector lies inside
the componentwise range of all four rigid hypotheses. Best-mask qdd-error norm
has p95/max 223.257/14,718.472, while tangential contact impulse has `+0.907`
correlation with that error. This is source localization, not a causal bound:
the impulse is known only after the plant interval and remains outside
authority. The next CPU experiment must use pre-step slip/load/contact state or
an explicitly bounded momentum/contact-impulse transition and validate it on
fresh held-out cases.

R196 evaluates six realization-residual models under strict
leave-one-named-case-out splits over the same 266 r195 selections. Deployable
features are causal current-state/model witnesses only; an oracle
physical-support/action model is labelled separately. No model passes strict
bracketing. Global maximum coverage is 99.25% with a 267.523 pressure bound; a
fixed 25% reserve reaches 99.62%. Action and kNN conditioning cover 98.12% and
85.34%; the Lipschitz model covers 91.35% despite a 37,977.373 p95 bound; oracle
support/action reaches 94.36%. The global worst miss is
left/drop5-matched/tick-700 typed withhold across flight→double→right support:
the largest smooth-qdd error is 14,732.8 rad/s². Rust rescoring remains
allocation-free with a 0.952 µs maximum. No residual enters authority; the next
model boundary is impulse/momentum aware.

R198 tests whether R197's strongest measured source can become a causal sensor
feature by delaying it one complete control interval. Previous normal,
tangential and generalized constraint impulses are appended with frozen linear
and log scales; current-interval impulse, next support, disturbance identity and
future outage duration remain excluded. The unmodified R196 85.34%/91.35%
kNN/Lipschitz baselines reproduce exactly. Both lagged encodings retain the
same coverage, while Lipschitz maximum exceedance worsens to 16.151 and its p95
bound stays near 38,000. No residual bound or authority is admitted. The next
CPU boundary is pre-impact relative velocity/load/penetration with explicit
timing uncertainty, or a bounded generalized momentum jump.

R199 evaluates that broader existing pre-step state/model family directly
against completed-interval normal/tangential impulse and two velocity-jump
residual targets. The fixed feature excludes physical support, post-step
state/impulse, case identity, and future dropout duration. No strict
leave-one-named-case-out row is admitted over 266 samples. Global maxima cover
99.624% of complete samples but require roughly 72 rad/s of five-millisecond
velocity-jump error and still miss left/drop5-matched by 1.536 rad/s; a fixed
25% reserve still misses normal impulse. Action, kNN, Lipschitz, and oracle
physical-support/action rows cover 97.744%, 75.564%, 90.602%, and 93.609%.
Plant runs remain finite and allocation/GC-free. This rejects empirical
current-state conditioning, not a physically authored momentum/impulse tube.

R200 then instruments the exact current contact boundary before each scored
interval: per-wheel availability, signed MuJoCo contact distance, and three
signed constraint-coordinate relative velocities. Frozen linear and signed-log
encodings are appended to the unchanged R196 feature under the same strict
six-case-train/seventh-case-holdout protocol. Across 266 samples, 247 queries
have current contact and 19 are in flight. kNN regresses from 85.338% to
84.962%; Lipschitz remains 91.353%, misses by 15.086 and expands its p95
maximum-component bound from 37,977 to 39,752. The worst kNN transition remains
flight→double→right inside the next 5 ms. The complete result tree repeats
exactly. Exact simulator contact prestate remains a candidate-sensor oracle,
not a typed online observation or authority. The next model must bound impact
time and generalized momentum jump directly.

R201 implements that next boundary in `bonesaw-core`. The allocation-free
function accepts a bounded transition-time interval, per-contact closing speed,
effective normal mass, sustained normal load, restitution and friction, plus a
caller-owned `M⁻¹Jᵀ`. It emits contact impulse and componentwise generalized
velocity-jump intervals into caller buffers. Five Rust tests cover analytic
projection, signs, uncertain time, contact permutation, failure atomicity and
realized in-box values; three PyO3 tests cover shape/error semantics and zero
allocation. Prospective wheel-bottom kinematics are sampled before every plant
interval, including flight. All six declared physical profiles cover every
measured normal/tangential impulse over 266 selections. The tight row has a
0.703 N·s p95 normal bound and 18.0% p95 utilization; the primary uncertainty
row has 6.042 N·s and 1.7%, with 0.148 µs Rust p99. Only the mechanism is
admitted. The response map is not yet sourced from Bonesaw's model, full
velocity-jump coverage and usefulness are unproven, and no candidate ordering,
plant consequence, timing or hardware authority changes.

R189 adds a Rust-owned minimum score-improvement gate over zero retained
authority. The selector remains fixed-size and allocation-free; insufficient
improvement installs Q15 zero without renewing a lease or primary health. All
mechanism and exact-replay gates pass across 127,865 first-run ticks, 154
profiles, and 1,792 queries. None of eight margins passes both outage classes.
The 0.001 row removes the r188 backward green fall but advances one 5 ms and
three 10 ms boundaries; the withheld endpoint advances five boundaries, so
more scalar conservatism is not a monotone physical ordering. Two runs retain
192–210 deadline misses, 8.847–9.448 µs selector maxima, 6.134–7.039 ms loop
maxima, and 5.699–6.645 ms controller maxima. Policy and synchronous admission
remain false.

## Gate 1 — canonical model and frame atlas

Implemented:

- canonical URDF skeleton and IDs;
- fixed-link inertia folding with semantic frames preserved;
- mass/inertia validation;
- current FK, Jacobians, CoM, mass matrix, energy, and gravity;
- recursive fixed-base inverse dynamics, bias forces, and forward dynamics;
- bounded robot history and deterministic ingest policy;
- continuous-joint interpolation and bounded prediction;
- constant-time pairwise current-frame queries;
- a compiled rooted frame atlas with `control_world`, `odom`, `map`, and every
  canonical robot frame;
- topologically ordered midpoint, origin/orientation, and ground-projection
  derived-frame operations;
- fixed-capacity external-frame histories with deterministic ingest, SE(3)
  interpolation, hold, and bounded constant-twist prediction;
- timestamped historical atlas queries that reconstruct minimal state, run FK,
  evaluate the atlas, and return source provenance/intervals;
- narrow PyO3/NumPy model-query and fixed-shape trace wrapper;
- deterministic canonical `MotionProgram` archive with schema, SHA-256 payload
  checksum, content fingerprint validation, and rebuilt runtime indexes;
- optional collision policy serialized into and covered by the canonical
  program fingerprint;
- explicit robot-root pose in `control_world`;
- tested isolation of global `map` jumps from WBC-root robot poses.

Still required:

- free-flyer, planar, and spherical manifolds;
- SE(3) history with covariance/error propagation;
- batched historical `query_frames` output and per-query quality/error limits;
- compiler-calculated fixed workspace layout.

The immutable `MotionProgram` does carry a stable SHA-256 fingerprint and
program epoch; archive serialization and compatibility policy remain.

## Gate 2 — pure-Rust kinematic rig

Implemented:

- point, orientation, CoM, and posture row emission;
- fixed five-level priorities;
- stable task ordering;
- strict recursive null-space solve;
- joint position/velocity bounds with deterministic activation,
  same-priority re-optimization, and stable tie order;
- general two-sided linear hard inequalities with deterministic stable-ID
  ordering, bounded phase-I feasibility, active-row projection, residual
  diagnostics, and typed contradiction detection;
- independent Pinocchio 4.0 differential fixtures for FK, all body frames,
  frame Jacobians, mass matrix, gravity, and inverse dynamics;
- isolated PlaCo reference harness using an exact shared corpus and raw
  per-step latency/tracking/memory traces;
- a checksum-pinned CMU subject-37/trial-1 walking corpus owned by Python:
  deterministic ASF/AMC reconstruction, named foot/hand retargeting,
  six-harmonic periodic closure, independent leg/arm morphology scale,
  0.75×/1.0×/1.25× cadence blocks, finite-difference target-velocity jets,
  stance/swing labels, and predeclared tracking/clearance gates. Rust consumes
  fixed-shape targets and owns every measured tick. The first grounded
  5,000-tick run has 0% reference flight and 25.66% double support; Bonesaw
  completes without contingency/rejection but remains red at `5.549 cm` foot
  RMS against the `5 cm` gate, while PlaCo remains red on `3.301 cm`
  swing-clearance RMS against its `3 cm` gate;
- a pinned official Upkie C++ controller-law worker and 100,000-step shared
  corpus. With upstream parameters both Rust wheel commands are canonically
  bitwise equal; the live tuning delta, per-region behavior, ten timing
  windows, p99.99 latency, jitter, RSS, CPU, faults, context switches, raw
  traces, and zero Rust loop allocations are retained separately;
- typed solve diagnostics;
- twelve fixed floating-task diagnostic slots (eight semantic base slots plus
  four Cartesian point slots) with stable IDs, task kind,
  priority, active-row count, physical-unit L2/RMS residual, and clipped-level
  membership, serialized by the browser adapter without adding hot-loop
  allocation;
- a moving-root CMU 37/01 floating-WBC corpus with a one-second phase ramp,
  moving-trajectory stance classification, Rust-latched measured touchdown
  anchors, locked/normal-only/released contact status, one-step velocity
  viability bounds, and separate nominal-prefix/full-run metrics. The first
  119 ticks are nominal with `0.430 cm` root and `0.878 cm` stance-foot RMS;
  the three-second run remains red after contact contingency and terminal
  infeasibility;
- a topologically compiled scalar/vector/rotation signal graph with value,
  velocity, and acceleration jets (world angular derivatives for rotation);
- input, constant, add, scale, analytic blend, clamp, deadband, low-pass, and
  critically damped spring operations;
- stable signal/output IDs, compile-time type and dependency validation, and
  explicit double-buffered stateful-node memory;
- signal topology and canonical memory layout frozen into `MotionProgram`
  archive schema 6 and its SHA-256 content fingerprint, including explicit
  actuator coordinates, transmission maps, actuator limits, and optional
  electrical/thermal resource profiles;
- caller-owned signal scratch/output buffers with zero measured heap
  allocations per evaluation;
- compiled point, CoM, and orientation task specifications that resolve
  signal-output stable IDs to fixed output slots and robot frames at authoring
  time;
- task stable-ID, frame, response-profile, output-slot, and signal-type
  validation;
- resolved task operations frozen into the program archive and fingerprint;
- signal evaluation, explicit memory transition, resolved task emission,
  strict solve, and quintic synthesis integrated into the primary Controller;
- a 5,000-tick two-stream signal-to-task controller sentinel with `0.9214 cm`
  point tracking RMS, `0.1929°` orientation tracking RMS, bitwise
  repeatability, zero allocations, and release p99 `139.9 µs` in the full
  reference run;
- compiled floating root-orientation, root-translation, and CoM acceleration
  tasks with canonical priorities, weights, damping ratios, acceleration
  bounds, and stable signal-output slots;
- one shared four-node/three-task Upkie floating policy compiler used by the
  browser and native endurance evaluator, including a 1 Hz critically damped
  root-translation command spring;
- fixed-capacity point/orientation/CoM/posture/repeller task slots;
- a compiled direction-aim task whose vector signal is normalized on S² and
  whose angular Jacobian is projected into the controlled axis tangent plane,
  leaving roll exactly unconstrained;
- fixed-capacity hard-constraint slots;
- caller-owned flat hierarchical-solver and pseudoinverse workspaces;
- zero measured heap allocations in standard and collision-enabled controller
  transitions after construction.

Still required:

- direction, pose, and frame-transform jets;
- compiled signal bindings for pole-vector, posture, and dynamic-WBC
  acceleration/wrench tasks;
- compiler-assigned named numerical row ranges beyond the resolved task slots;
- pole-vector, manipulability, and base tasks;
- warm-start state;
- archive-compiler-assigned named row ranges (the current capacities are
  calculated when controller scratch is constructed);
- exact-command differential adapter for a full WBC formulation with matching strict
  hierarchy semantics.

## Gate 3 — trajectory replacement

Implemented:

- fixed-horizon quintic construction;
- position/velocity/acceleration endpoint continuity;
- deterministic polynomial root isolation for derivative extrema;
- limit validation;
- exact 1 kHz sampling into four flat structure-of-arrays buffers;
- contingency deceleration segment.

Also implemented:

- explicit `ControllerState` input/output double buffering;
- caller-owned `ControllerScratch` and reusable `ControllerOutputBuffer`;
- borrowed reusable `ControllerInput` for native and language-binding loops;
- commanded-state splice independent of lagging observed state;
- immutable-program epoch validation;
- allocation-counting sentinel around the controller call;
- preallocated task, constraint, solver, trajectory, model, collision, and
  output workspaces;
- zero allocation calls and bytes per transition in all native sentinels.

Still required:

- configurable observed/commanded divergence policy;
- retry solve with segment-derived bounds;

## Gate 4 — collision and wheelbase

Implemented:

- retained sphere, cylinder, box, and mesh collision descriptions;
- deterministic conservative sphere proxies and compiled self-pairs;
- signed distance, normal, closest points, normal velocity, and Jacobian rows;
- deterministic first-order hard barrier inequalities and soft repeller tasks
  with stable pair-derived row IDs;
- dense servo-grid segment clearance validation with first-violation and
  minimum-distance provenance;
- opt-in controller integration that emits avoidance rows before the
  hierarchical solve and selects the contingency segment when the primary
  segment violates configured clearance;
- immutable finite dense world SDFs with analytic trilinear gradients,
  explicit Reject/OccupiedBoundary policy, conservative body probes, and
  allocation-free floating acceleration barriers;
- exact primary/brake command-world sampling plus conservative Lipschitz
  clearance certification, bounded adaptive midpoint refinement, and stable
  probe/body/time/unknown provenance;
- explicit local-SE(3) floating-root prediction segments paired independently
  with primary and brake, including analytic root-twist bounds and compiled
  conservative probe reach for angular motion;
- differential-drive forward/inverse wheel mapping;
- successful Upkie import including both wheel coordinates.

Still required:

- adjacency/filter configuration beyond immediate parents;
- exact mesh CCD and mutable/versioned dynamic-scene sweeps;
- prediction covariance/error growth and command-history or forward-dynamics
  root predictors beyond the current state-local short-horizon witness;
- wheel limit propagation.

## Gate 5 — dynamics

The CPU reference provides a Jacobian-assembled joint-space mass matrix,
potential/kinetic energy, gravity generalized force, composite fixed-link
inertia, recursive inverse dynamics, velocity/gravity bias force, and a
mass-solve forward-dynamics oracle. Randomized evaluations check positive mass
eigenvalues and inverse/forward round-trip error.

Also implemented:

- reusable allocation-free mass, gravity, bias, and inverse-dynamics outputs;
- reusable allocation-free damped planar point IK with joint-limit projection
  and an Upkie dual-contact squat sentinel;
- allocation-free fixed and floating point/angular Jacobians, mass matrices,
  bias forces, inverse dynamics, centroidal maps, and centroidal momentum;
- a unified fixed-base decision vector `[q̈, τ, contact force]`;
- a unified floating-base decision vector
  `[root q̈, joint q̈, actuator τ, contact force]`;
- hard rigid-body dynamics equality;
- six unactuated free-body equilibrium rows with no root-torque variables;
- hard point-lock and wheel rolling/lateral contact-acceleration rows including
  `dJ·v`;
- unilateral normal-force ranges;
- four-sided friction pyramids;
- acceleration and actuator-torque bounds;
- strict acceleration, nominal-load, and torque objectives;
- typed fixed-capacity scratch and result storage;
- a 5,000-tick native sentinel with zero infeasible ticks, zero allocations,
  bitwise repeatability, sub-`1e-10` dynamics residual, sub-`1e-11` contact
  residual, and release p99 below 3 ms on the baseline host.
- an independent Pinocchio 4.0 oracle over 50 Upkie states for fixed and
  floating mass, bias, inverse dynamics, CoM, centroidal map, and momentum,
  with maximum floating error `2.132e-14`;
- a 5,000-tick Upkie floating-contact sentinel with zero infeasible ticks,
  bitwise repeatability, zero allocations, dynamics residual `5.14e-12`,
  contact residual `1.82e-14`, and release p99 `132.4 µs`.
- a canonical `RollingWheel` row coupling wheel-center and wheel angular
  acceleration, with bounded velocity-residual stabilization and a unit-level
  coefficient/sign/bias contract;
- a 5,000-step raw-integrated Upkie balance/squat sentinel using Upkie's PI
  structure with an articulation-aware axle-to-CoM virtual pitch: full `12 cm`
  lowering, `5.93e-6 m` constrained slip, 1.73 cm permitted wheel travel, zero
  infeasible ticks, zero allocations, dynamics residual `2.19e-11`, and
  isolated-endurance release p99 `277.8 µs`;
- an r129 allocation-free rooted Upkie capture/station session. The full
  sagittal DCM is a Viability pressure witness, a C1 fade subordinates the odom
  station Preference, and `map ← odom` remains reporting-only. A 10,000-call
  policy-component/physics-free corpus has exact replay, zero command change
  under a 10 m map jump, validated quaternion faults, zero GC, and a Rust
  allocation check on every call;
- an r130 seven-value, 10-second MuJoCo fraction sweep. Fractions `0.05–0.40`
  recover and return to station, `0.00` loses station, and `0.60/1.00` fall.
  The frozen `0.20` value is the fastest qualified station re-entry rather
  than the maximum passing feedback fraction;
- an r130 retained 10-second closed-loop consequence gate. The 4 N × 100 ms
  forward torso probe recovers in `1.450 s`, re-enters station authority in
  `1.870 s`, finishes at `−6.9 µm`, admits every tick after a fail-closed 25 ms
  startup, and records controller p50/p99 `125.2/166.0 µs`, full-loop p99
  `446.9 µs`, zero 5 ms overruns, zero timed Rust allocations, and zero Python
  GC. The 6 N overload falls; broader recovery axes remain open;
- an r131 typed live plant gateway. TARGET stays on the guided `/ws` protocol;
  PUSH lazily opens `/plant-ws`. Rust owns finite-vector validation, the `8 N`
  limit, request correlation, a `140 ms` dead-man expiry, one isolated worker
  per client, and child lifetime. Python/MuJoCo owns contact and integration,
  feeds measured state into the persistent Rust capture/WBC sessions, and
  streams named transforms plus capture/station/actuator/compute evidence at
  `50 Hz`. Five fresh-session trials pass response, recovery, expiry,
  invalid-command survival, correlated reset, and timing: Rust controller p99
  at most `458.3 µs`, four-tick worker p99 `4.236 ms`, and stream p99 `41.5 ms`.
  A supplemental local/public trace reports a sustained-push fall, resets on
  the next stream step, settles at `0.024 mm` station error, and reconnects
  with a fresh worker;
- an r132 bounded application-point wrench path. Rust checks latest-snapshot
  body identity and a `750 mm` body-origin lease; Python independently checks
  the exact MuJoCo COM lever and streams `tau=(point−COM)×force`. Five repeats
  of fresh −200/0/+200 mm sessions observe
  `−0.400765/0.000289/+0.401712 N·m`, the `0.802477 N·m` differential matches
  the `0.800000 N·m` prediction, signed pitch/rate are strictly ordered, and
  physical fields replay bit-exactly. A 751 mm request is rejected and cleared
  without disconnect; controller/worker maxima are `325.5 µs`/`2.071 ms`;
- a Python-owned rolling corpus with nine train/held-out cases, physically
  admissible initial ground velocities from -5 to +5 cm/s, two isolated
  repeats per case, zero infeasible ticks, and exact repeatability outside
  timing fields;
- default raw WebSocket integration using the corpus-equivalent wheel-center
  contact rows, axle-to-CoM observation, PI wheel task, Intent posture/CoM
  tasks, and SE(3) transition. An 18-second browser hold completed without
  transport or solver errors; the Hello contract declares the execution mode.
  Revision r16 adds clipped-level RMS and the dominant named task residual to
  live telemetry, and serves review assets with `no-store` cache semantics;
- a 5,000-tick 18-DOF floating-contact sentinel with zero infeasible ticks,
  bitwise repeatability, zero allocations, acceleration-tracking RMS `1.06e-2`,
  dynamics residual `5.14e-10`, contact residual `5.19e-11`, and release p99
  `0.669 ms`;
- official-G1 moving-liftoff and full-transfer profiles with raw per-step state,
  task, contact, residual, and latency arrays. The r30 liftoff passes every
  behavior and CPU gate at `3.904 ms` p99 (`2.320 ms` median). Its cached-energy
  Jacobi kernel preserves the bounded hybrid feasibility seed; active-set
  cycling now restores exact Dykstra state and resumes the remaining bounded
  projection budget. The support-preview transfer trace has no contingency or
  rejected ticks and passes the CPU gate, but deliberately retains large
  tracking error and remains behaviorally red;
- measured touchdown admission in `bonesaw-core`: an authored contact edge is
  a request until the sole material point is within `2.5 cm` of its anchor and
  below `0.20 m/s` tangential and normal velocity. `bonesaw-py` evaluates the
  same finite-patch FK/Jacobian state used by the WBC, retains the outgoing
  support until the replacement locks, and streams delayed-admission telemetry.
  The accepted toe-step remains combined green; the CMU transfer now fails
  honestly with 172 denied-contact ticks instead of creating mid-air contact;
- allocation-free actual-CoM trace output: `bonesaw-py` writes measured
  world-frame CoM positions into a caller-owned `[ticks, 3]` NumPy array from
  the Rust tracking cache, and the Python corpus reports/archives CoM RMS and
  p95 beside the authored reference. The r32 causal A/B rejects the current
  support-centre position servo; r33 implements its capture-point/DCM and
  clipped-virtual-ZMP replacement as the next opt-in experiment;
- allocation-free DCM/virtual-ZMP control in `bonesaw-core`: measured-height
  LIPM frequency, measured DCM feedback, deterministic fixed-capacity convex
  hull construction, metric support erosion, nearest-polygon ZMP projection,
  horizontal-only CoM task emission, acceleration cap, and explicit height-
  floor failure telemetry. `bonesaw-py` streams measured/target DCM, raw and
  clipped ZMP, CoM velocity/command, natural frequency, measured height,
  clipping flags, and hull size. The r33 short-horizon improvement is proven,
  but the full transfer rejects this first policy, so it remains opt-in;
- an allocation-free joint acceleration viability interval in `bonesaw-core`
  combines position, velocity, acceleration, and stopping-distance bounds.
  The Python boundary exposes it only as an explicit experiment because the
  invalid long walking trace can enter emergency braking and make strict
  contact infeasible; default controller behavior is unchanged;
- an allocation-free smooth joint-velocity-envelope acceleration law plus
  preallocated active-coordinate assembly in the Rust Python boundary. It is
  exactly inactive below its configured utilization, activates with a cubic
  smoothstep, and remains opt-in: r34 improves DCM/swing behavior and delays
  contingency, but does not preserve root attitude over the full transfer;
- typed measured contact-phase authority in `bonesaw-core`, with validated
  nonnegative per-phase scales and bounded non-overshooting release. The Rust
  batch boundary retains the scalar state, assembles active coordinates in
  fixed storage, and streams phase/scale/count arrays. r35 rejects phase-only
  promotion but preserves the mechanism for the next feedback schedule;
- signed measured DCM margin to the exact eroded support polygon and a
  cubic-smooth feedback authority target over capture margin, root-attitude
  error, and precontact age. Target/applied scales are fixed-array telemetry.
  r36 improves contingency timing but proves that the fixed landing target is
  far outside support;
- capture-aware sole-center landing retargeting in `bonesaw-core`, with an
  analytic authored-disk/future-root-reach intersection, fixed landing height,
  bounded anchor slew, an explicit commitment horizon, and fixed-shape
  anchor/reach/touchdown-viability telemetry. r37 delays contingency from tick
  472 to 482 in its stable case without rejected solves. It remains opt-in:
  measured position and tangential speed never satisfy touchdown together, so
  whole root/CoM/swing trajectory retiming remains next;
- causal fixed-horizon walking-reference construction in Python eval land. The
  DCM oracle now honors `support_preview_ticks` as a receding horizon, and
  lateral root registration is pinned to one canonical CMU cycle rather than
  output-buffer length. Exact prefix regressions cover the full authored jet
  and contact schedule. The selected 200-tick corrected trace is repeatable and
  rejection-free but remains touchdown-red, so it is evidence rather than a
  promoted controller default;
- persistent touchdown phase/rate state in the Rust floating controller, backed
  by a pure no-allocation conservative-rate policy, bounded recovery, quintic
  vector-jet sampling, and the full time-warp chain rule. Root, CoM, four
  endpoints, and contact intent share one cursor; fixed-shape phase/rate/
  required-time/limiter telemetry crosses PyO3. A stressed deterministic G1
  touchdown goes from a failing 10-tick transition to a passing 3-tick
  transition across three bitwise-exact behavior repeats. The progress gate
  rejects pre-touchdown stalling on the still-red CMU transfer;
- an optional allocation-free DCM-margin phase-rate law in `bonesaw-core`. It
  uses a C1 support-margin taper and min-combines with touchdown rate while the
  same cursor continues to sample root, CoM, endpoints, and contact intent.
  r40 proves exact disabled-path parity across 71 arrays and improves the best
  clean prefix from 482 to 490 ticks, but rejects promotion: the edge state is
  still 0.639 m / 6.044 m/s versus the immutable 0.025 m / 0.20 m/s touchdown
  envelope, and the 800-tick extension diverges;
- a policy-free, physics-free Python reference contract that consumes only
  authored root/CoM/foot jets, stance schedule, and sole geometry. It derives
  positive-normal-force, friction, zero-angular-momentum CoP/DCM support,
  contact-continuity, and reach checks while explicitly excluding effective
  references, tracked state, controller/solver output, force telemetry, and
  timing. The current CMU references fail before WBC execution;
- a pinned PlaCo 0.9.23 `WalkPatternGenerator` oracle adapted to the official
  G1 sole/trunk frame conventions without modifying the source model. It emits
  standalone CoM/foot/contact arrays under matched first-step geometry and
  timing, with no WalkTasks, IK, WBC, integration, or simulator. Eight of nine
  reference gates pass; two CoP boundary ticks keep it strictly red and locate
  the next planner defect before controller execution;
- an allocation-free Rust two-stage LIPM boundary planner plus a thin Python
  NPZ/report adapter. The planner joins exact constant-CoP arcs with a `100 ms`
  analytic linear-CoP transfer, rejects every switch candidate outside the
  fixed opening/future support polygons, and bounds the G1 landing reach at
  `0.800 m`. The r43 trace passes all nine policy-free, physics-free gates with
  `0.880 cm` minimum CoP margin, `31.8 m/s³` peak sampled jerk, bitwise repeat,
  and zero allocation calls in the measured plan/sample regions;
- immutable standalone-reference admission in the floating G1 corpus. Exact
  root/CoM/foot/contact arrays are consumed without reconstruction, projection,
  or retiming. The first WBC run retains a 265-tick nominal prefix and zero
  primal-infeasible ticks but tips later, proving that a morphology-consistent
  joint/posture witness is still missing downstream of centroidal feasibility;
- an allocation-free Rust whole-body morphology certificate and stateless
  oracle-state WBC admission. The r44 reference separates pelvis/root motion
  from CoM motion, preserves both flat-foot orientations, projects joint
  position/velocity/acceleration analytically, and evaluates 600 independent
  floating inverse-dynamics solves without policy, integration, or physics.
  The position witness converges on every tick, all three measured Rust loops
  allocate zero bytes, hard dynamics/contact residuals are 1.22e-9/4.88e-11,
  physical outputs are bitwise repeatable, and all 26 declared gates pass;
- allocation-free hard finite-support CoP inequalities in the floating WBC.
  Revision r45 derives each patch hull from current contact geometry, erodes it
  by a declared margin, exposes the achieved geometric margin, and extends the
  oracle-state corpus to 27 gates. The 5 mm contract is green across all 600
  states; the 10 mm sensitivity remains feasible but exposes a swing-tracking
  limit;
- typed continuous authority evidence and capability curves. Revision r49
  keeps joint-position headroom and actuator-effort utilization threshold-free
  in `bonesaw-core`, intersects floating-WBC torque bounds with finite URDF
  effort limits, and fills fixed-shape joint/actuator/hard-row/solver/task-layer
  traces inside the Rust PyO3 hot loop. The 600-tick oracle passes 30/30 gates
  without policy, state integration, or physics: peak effort is 57.47%, minimum
  joint headroom is 11.48°, hard residual is 1.22e-9, support reaches the
  declared 5 mm boundary, and the retained WBC timing run is 4.02/4.60 ms
  p99/max with zero 5 ms misses. All compared outputs
  are bit-for-bit identical to the retained loose-2000-Nm control. Python owns
  absolute threshold sweeps and reporting; thermal, electrical power, speed,
  reliability, and task-specific physical normalization remain unmodeled;
- explicit generalized-to-actuator semantics and persistent actuator resource
  authority. Revision r50 replaces the implicit one-joint/one-actuator
  assumption with a schema-6 `CompiledActuation`: stable actuator IDs,
  actuator-space velocity/acceleration/effort/jerk limits, a dense forward
  transmission, optional validated reverse map, and a power-consistent effort
  transpose. The URDF path emits an explicit identity fallback. A coupled 2×2
  fixture proves velocity round-trip and power preservation;
- exact coupled actuator-space effort inequalities in both fixed and floating
  dynamic WBC. Revision r51 accepts square invertible transmissions, emits one
  stable hard row per finite actuator bound using `Gᵀ τ_generalized`, minimizes
  the same physical actuator effort at Style priority, and reports exact
  actuator-space headroom plus the limiting actuator without allocations. The
  600-state G1 oracle adds an explicitly synthetic ankle differential at ±14
  Nm. Its r54 four-step rerun passes all 45/45 gates: all 2,317 states solve or
  use typed lower-layer slack, 119 ticks reach at least 99% pair
  utilization, power duality closes to 4.44e-15 W, Rust allocates nothing, and
  the repeat is bitwise identical. This retracts the earlier 32/32 tracking
  claim while preserving the exact coupled-effort evidence. Passive,
  underactuated, and overactuated force subspaces remain a declared gap until
  the program carries an explicit generalized-force map;
- sustained multi-step oracle admission without a policy, state integration,
  simulator, or physics rollout. Revision r54 uses the unchanged reference of
  four alternating 62 mm steps from Rust support-constrained LIPM segments, then
  projects and scores 2,317 independent G1 states in the allocation-free Rust
  path. All eight contact-edge windows and every hard dynamics, contact,
  friction, finite-CoP, effort, repeatability, allocation, and timing gate pass:
  870 states solve exactly, 1,447 use typed lower-task slack, none fail, support
  keeps 5 mm, effort peaks at 53.13%, and p50/p99/max WBC time is
  3.511/5.876/7.832 ms. Root attitude and height now occupy Invariant, matching
  the integrated floating profile, while horizontal transfer remains Viability.
  All 43/43 gates pass: root angular/height residuals fall to
  0.001849 rad/s² / 0.000775 m/s² without weakening hard constraints or moving
  thresholds. The architecture authority stack still exposes Preference/Style
  clipping and per-step/contact-window evidence rather than collapsing full
  physical admission into an all-objectives-exact claim;
- an r55 report-only CPU reference audit that keeps comparison semantics
  explicit. The current four-step G1 trace contributes full per-tick WBC,
  task-residual, CPU/wall, RSS, allocation, deadline, jitter, headroom, and
  one-second execution-window evidence. Checksum-retained r38 artifacts add the
  identical-corpus PlaCo fixed-base task comparison, Pinocchio 4.0 fixed and
  floating rigid-body product oracle, and the pinned upstream Upkie C++ rolling
  law. The aligned Rust law has 0/100,000 canonical command mismatches; the G1
  Pinocchio maximum is 1.71e-13. Timing rows are host/run observations and no
  speed ratio is claimed across different semantic boundaries or collection
  revisions;
- an r89 fresh same-session reference audit over three 5,000-tick fixed-base
  corpora, 100,000 aligned Upkie controller-law samples, fresh Pinocchio
  products, and the current G1 profiles. It retains raw per-step NPZ arrays,
  full CPU/memory/jitter/latency/tracking distributions, seven responsive
  execution-over-time charts, and 600 lossless one-second window rows. Bonesaw
  records 1.51–2.75× lower p50 fixed-base boundary latency and roughly half
  PlaCo's process RSS, while PlaCo records lower RMS tracking error on the
  bimanual and walking cases. Each walking candidate fails a different authored
  gate. Upkie aligned commands have 0/100,000 canonical bit mismatches. These
  are separately scoped witnesses rather than an aggregate reference verdict;
- an r123 policy-, physics-, and integration-free Upkie rolling-WBC admission.
  `FloatingWbcSession.run_oracle_trace` now accepts fixed-shape per-target
  contact modes plus explicit rolling coordinate, velocity coefficient,
  stabilization gain, and bounded correction. The 256-state corpus composes
  two `RollingWheel` contacts with the floating decision vector, URDF effort
  limits, unilateral force, square friction pyramids, and strict task layers.
  Pinocchio independently rebuilds `M`, `h`, both wheel-center Jacobians, and
  `Jdot·v` from the URDF, then checks the returned qdd/torque/force at
  `1.97e-11` dynamics and `4.61e-12` contact L∞. All states solve or use typed
  lower-layer slack; repeat, reverse order, and two-chunk outputs are bitwise
  exact; measured Rust queries allocate zero bytes. The report retains the
  large wheel/posture tracking loss and 250 rad/s² acceleration-cap saturation
  as exhausted authority rather than treating hard-equation admission as pose
  realization. It also includes slip stabilization, invalid-descriptor fault
  typing, friction/effort sweeps, task/nullspace residuals, memory, CPU,
  latency/jitter, work correlation, execution-order windows, and the pinned
  official Upkie scalar-law comparison;
- an r124 long contact-transition replay over the same policy-, estimator-,
  integration-, and physics-free boundary. The PyO3 trace accepts an optional
  fixed-shape `[ticks, targets]` mode array while keeping activity and rolling
  constants separate. A seeded 20,000-state Upkie corpus covers all four
  point-contact modes, double/single/mixed/no support, bounded rolling slip,
  and 288 authored edges. Pinocchio independently selects XYZ, Z, YZ, or
  wheel-coupled-X-plus-YZ rows per target and reconstructs packed contact-force
  dynamics. All 20,000 states are `Solved` or `SolvedWithSlack`; independent
  dynamics/contact L∞ is `3.02e-11` / `6.13e-12`, inactive force tails are
  exactly zero, and repeat/reverse/four-chunk outputs are bitwise exact. The
  measured call runs at `110.40/172.94/220.37 µs` p50/p99/max with zero
  0.5 ms misses, zero Rust allocations, zero GC, and no RSS growth. Whole-call
  prevalidation makes nonfinite state, invalid mode/activity, and zero rolling
  coefficient errors atomic: each produces a typed `ValueError` before any
  caller output changes. Contact estimation, impact dynamics, and hybrid
  closed-loop stability remain explicitly outside this state-local gate;
- an r125 state-local resource-authority boundary over the same Upkie corpus.
  PyO3 accepts optional `[ticks, generalized_dof]` acceleration and
  `[ticks, dof]` actuator-effort availability in `[0,1]`; Rust scales the exact
  solver/telemetry bounds in place and restores nominal authority between
  calls. A 10,000-state nominal/acceleration/effort/combined replay remains
  entirely policy-, estimator-, integration-, and physics-free. Every query is
  `Solved` or `SolvedWithSlack`; the combined case reaches acceleration and
  effort bounds 6,201 and 1,368 times, including 526 simultaneous contacts,
  while independent Pinocchio dynamics/contact L∞ remains `6.11e-11` /
  `1.19e-11`. All-ones scales are bitwise transparent; repeat, reverse, and
  four-chunk results are exact; hot-loop allocations remain zero. Negative,
  above-one, and nonfinite scale probes are typed and atomic. Availability is
  declared external state and does not claim a thermal cause or plant
  realization;
- an r90 live timing correction that distinguishes the maximum end-to-end
  single query from the sum of four 5 ms queries in each streamed 50 Hz frame.
  The unchanged flat-foot physical trace passes both clocks: maximum-query
  p50/p99/max `1.923/3.076/3.223 ms` with 0/60 over 5 ms; four-query batch
  `7.349/9.148/9.373 ms` with 0/60 over 20 ms. Query count and batch time are
  carried explicitly in the WebSocket state. Work counters and a perf capture
  identify dense pseudoinverse work as the next optimization target without
  changing an iteration limit or semantic gate;
- an r91 real CUDA StateInput stage behind a dynamically loaded Driver API.
  Embedded, checksum-retained PTX uses one thread per agent, fixed SoA buffers,
  device-side finite checking, deterministic invalid/padding zeroes, and no
  atomics. Construction owns a context, module, function, and five device
  allocations; execution has no CPU fallback. Its CpuMirrorF32 reference
  passes byte replay, permutation, neighbor isolation, padding, and zero host
  allocation. The current host reports `no_device`, so all actual device
  execution gates remain NOT RUN and the full CUDA mirror/Graph capabilities
  remain false;
- an r56 continuous capability evaluator over the unchanged policy-,
  integration-, simulator-, and physics-free four-step states. Eleven separate
  authority rows retain raw per-tick pressure, warning/critical episodes,
  5–500 ms dwell curves, and 0.25/1/5 s leaky exposure. There is no aggregate
  score and no dwell allowance for a hard violation. Current evidence has zero
  hard/20 ms critical ticks, one isolated 5 ms Viability-transfer warning, 76
  nominal-5 ms overruns with a 40 ms longest episode, and persistent legal
  Preference/Style clipping of 990/575 ms. The browser report renders a
  responsive heatmap from the same checksum-pinned raw trace;
- an r57 four-run CPU-tail audit with exact semantic-array gating. Baseline and
  logical-CPU-2/4/6 traces keep all 56 non-timing fields byte-identical while
  p99 varies from 5.746 to 7.742 ms, over-5 ms ticks from 49 to 558, and longest
  episodes from 20 to 320 ms. This proves both that dense work matters and that
  host placement/frequency/contention can dominate the wall-time tail. A
  clock-dependent early exit is therefore rejected; fixed-governor isolated
  cycles/instructions must precede any deterministic work-budget change;
- an r58 five-run hardware-counter audit pinned to logical CPU 4. All 56
  non-timing arrays stay byte-exact, retired instructions span only 0.001081%,
  and task-clock/cycles vary by 27.63%/26.73% of their medians. Counter scope is
  explicitly the complete Python admission process, so the result proves
  deterministic work without pretending to attribute counters to one WBC
  tick. A native WBC-only counter sentinel is the next CPU measurement;
- an r59 native G1 WBC counter sentinel with adjacent one-tick setup-process
  subtraction. Five × 2,000-solve release trials retain identical non-timing
  reports, zero infeasible ticks, bitwise repeat, sub-1e-9 hard residual, and
  zero solve-loop allocations. The marginal median is 21.991 million
  instructions, 6.099 million cycles, and 1.505 ms task-clock per tick, with
  only 0.000041% instruction span. It is explicitly a simpler native marginal
  upper bound, not a replacement for r54 finite-support/full-stack admission;
- an r60 negative Jacobi A/B. Moving the cached-energy discard test before the
  coupling dot keeps direct kernel values, native semantic reports, and the
  56-field r54 corpus exact, but adds 92,433 stable retired instructions per
  native tick (+0.420%). Production retains the original lower-work path; the
  experimental branch is feature-gated and disabled by default;
- an r61 negative Jacobi A/B. Fusing the initial Frobenius and per-column
  energy scans preserves both reductions bit-for-bit and keeps native/r54
  behavior exact, but adds 58,916 retired instructions per native tick
  (+0.268%). Production retains the compiler-friendly separate reductions;
- an r62 physics-free actuator realization boundary. A typed Rust first-order
  effort response separates bandwidth lag, slew limiting, and instantaneous
  availability clipping over every r54 effort sample. Six synthetic profiles
  pass exact-repeat, source-immutability, availability, and zero-allocation
  mechanism gates. The 20/10/5 Hz cases recover across contact edges within
  10/25/55 ms; 25% availability clips 1,623 ticks for up to 2.145 s. No profile
  is G1 calibration, and no rigid-body/body-tracking claim is made;
- an r63 continuous fixed-effort acceleration consequence boundary. A compact
  allocation-free Rust scratch layout removes the torque decision block and
  applies realized effort directly in floating dynamics. Dynamics and active
  locked-contact acceleration remain exact; acceleration limits, unilateral
  force, friction, and 5 mm support are four independently scored continuous
  traces. Ideal effort reconstructs r54 acceleration within `2.43e-9` with no
  inequality violations. Synthetic 20/10/5 Hz profiles reach
  `4.58/7.30/10.73` acceleration RMS and `75/105/510 ms` maximum edge recovery;
  the 25% case remains above 1% pressure for `5.655 s`. Eight mechanism gates,
  the declared repeat witness, and zero-allocation checks pass. This query is
  state-local and rollout-free; calibration and closed-loop stability remain
  mandatory;
- an r64 independent fixed-effort differential oracle. Pinocchio 4.0 rebuilds
  floating `M`, `h`, eight sole-point Jacobians, frame/CoM Jacobians, and
  finite-difference contact `Jdot-v`; a separate NumPy SVD hierarchy rebuilds
  equality seeding and Invariant/Viability/Preference/Style nullspace freezing.
  The 48-state set includes every contact-edge neighborhood plus even/adverse
  states and four effort profiles. Rust-under-Pinocchio dynamics/contact maxima
  are `6.30e-9`/`4.38e-10`; independently solved acceleration/contact-force
  maxima differ by `1.26e-9`/`3.05e-7 N`. Six gates, source immutability, and
  exact repeat pass. This validates the state-local equations and optimizer,
  not calibrated response or closed-loop stability;
- an r65 rejected exact-zero-row micro-optimization and r66 promoted
  feasible-set task-row compaction. R65 preserves all 56 non-timing arrays but
  changes median retired instructions by only `-0.00007%`, so it remains an
  explicit negative result. R66 additionally omits satisfied fixed-bound rows
  only at terminal Style, where no lower consumer can observe a projector
  change. The four-step corpus avoids 20,176 dense row-instances and five
  pinned process pairs reduce retired instructions by `3.861%` in every pair.
  The default build passes 43/43 admission, 8/8 fixed-effort consequence, and
  6/6 independent Pinocchio/NumPy gates. Declared task shapes, residual rows,
  clipping provenance, and zero-allocation behavior remain intact; this is
  solver-internal work compaction, not removal of an authority layer. The
  `resolved-task-row-compaction-control` feature retains the exact A/B control;
- an r67 rejected coincident-step-limit freeze. The mechanism reduces three
  projected inverse evaluations to two on an exact two-coordinate co-hit unit
  witness, but removes zero pseudoinverses from the 2,317-state G1 admission:
  both builds execute 16,809 inverses and 10,604 clipped steps. All physical,
  decision, authority, feasibility, clipping, rank, and allocation arrays are
  exact; two Jacobi diagnostic samples add one aggregate sweep. Both variants
  pass 43/43 gates with zero hot-loop allocation. The opt-in experiment is
  retained as negative evidence; r66 remained the production default until the
  independent r72 execution optimization was promoted;
- r68/r69 rejected sequential-clipping shortcuts. R68 projected-gradient
  reoptimization changes the monotonic active set and fails two established
  dynamic-WBC hard-feasibility tests at `1.24e-7`/`1.69e-7`, so the path is
  removed without relaxing the `1e-8` contract. R69 uses an analytic exact
  task-nullspace repair and passes all 117 feature-build tests; it reduces two
  inverses to one on a redundant unit witness. The 2,317-state G1 corpus is
  nevertheless a complete work no-op: 16,809 pseudoinverses, 122,031 Jacobi
  sweeps, and 10,604 clipped steps in both builds, with all 56 non-timing arrays
  exact and both variants passing 43/43 with zero allocation. The r69 feature
  remains opt-in; r66 remained production until the independent r72 execution
  optimization was promoted;
- r70/r71 rejected projected-solve factorization experiments. R70 uses a
  conservative full-row-rank guard before solving wide task projections via a
  small row-Gram Cholesky factor. It passes 43/43 admission, stays allocation
  free, removes 47,918 reported Jacobi sweeps, and independently passes the r64
  Pinocchio/NumPy oracle, but changes 22/56 corpus fields and fails 19 strict
  r63 replay fields after small high-priority arithmetic changes select a
  different monotonic active set. R71 exactly specializes a one-row task solve;
  all 56 arrays are bit-exact in the corpus and five pinned A/B pairs, but
  median retired instructions change `+0.000103%` and pair signs are mixed.
  Both features remain opt-in negative evidence; r66 remained production until
  the independent r72 execution optimization was promoted;
- r72 promoted slice-addressed one-sided-Jacobi column pairs. Prevalidated
  contiguous slices replace repeated flat-index multiplication and bounds
  logic without changing a single arithmetic or comparison order. All 121
  production/control core tests pass; the complete 2,317-state corpus and all
  five pinned A/B pairs retain 56/56 bit-exact non-timing arrays, 16,809 task
  pseudoinverses, 122,031 sweeps, 10,604 clipped steps, and zero allocation.
  Every process pair retires 30.492% fewer instructions; median process CPU,
  p50, and p99 fall 6.28%, 6.28%, and 6.98%. The native marginal sentinel
  confirms 22.022 → 15.807 M instructions/tick (`-28.22%`) with identical
  semantic reports. The slice path is production default and
  `jacobi-column-slice-control` is the explicit A/B control;
- r73 rejected paired iteration over the existing Jacobi column slices. The
  candidate preserves the native semantic report and all scalar operation
  order, but five pinned instruction deltas have mixed signs and a
  `+0.0000014%` median. It remains an opt-in negative result rather than being
  promoted on compiler-equivalent noise;
- r74 promoted row-sliced general dense products after a post-r72 native cycle
  profile attributed 81.09% to pseudoinverse work and 8.21% to the multiply
  symbol. Left/output rows and each shared right row are validated once while
  output initialization, row/shared/column traversal, exact-zero skipping, and
  multiply-add order remain unchanged. Default production and flat-index
  control pass all 123 core tests. The 2,317-state corpus and five process pairs
  retain 56/56 exact arrays, 16,809 inverses, 122,031 sweeps, 10,604 clips,
  43/43 admission, and zero allocation. Process instructions/CPU/p50/p99 fall
  4.104%/2.23%/2.19%/1.94%; the native sentinel falls 15.807 → 15.317 M
  instructions/tick (`-3.098%`) with identical semantics. Production now
  composes r66, r72, and r74; `dense-multiply-row-slice-control` retains the
  pre-r74 path;
- an exact allocation-free Rust electrical/thermal update plus fixed-shape
  PyO3 batch trace. A 120 s, three-actuator synthetic workload passes 12/12
  gates over 24,000 ticks: 0.120/0.181 µs p50/p99, zero allocations, bitwise
  repeat, 1.38e-12 °C closed-form error, and 1.42e-14 W power-identity error.
  At identical 15 Nm / 1.5 rad/s demand, modeled effort scale changes from
  100% at 41.33 °C to 10% at 100.13 °C. The fixture proves the mechanism only;
  live Upkie remains UNMODELED pending calibration and telemetry;
- finite-support propagation through the ordinary stateful floating session.
  Revision r46 rebuilds patch rows from each established four-point contact,
  preserves them through the normal fallback solve, removes them with released
  contact force slots, and streams the measured support margin without a
  per-tick allocation. The 160-tick G1 toe-step respects the 5 mm erosion on
  every loaded tick with no fallback, release, infeasible, or failed status.
  Its functional gates are green, while two release-build repeats remain timing
  red at 5.57/5.49 ms p99 against the unchanged 5 ms CPU deadline;
- model-authored browser geometry over the existing frame stream. Revision r48
  adds 41 URDF visual instances to the canonical Rust model, including 25 mesh
  instances referencing ten upstream Upkie STL files. The fetch script pins and
  checksums the URDF, license, and all STL bytes at one upstream revision. The
  browser fetches each unique mesh once, applies deterministic vertex-cluster
  simplification, authored origin/scale/material, and the canonical streamed
  body transform. BODY remains the default layer and skeletal RIG is separately
  switchable. The initial root height still comes from the exact collision
  envelope, registering wheel bottoms and grid at `z=0`. The r48 architecture
  review now carries a mobile-oriented measured G1 example authority stack for
  continuous layer residual/clipping, support, joint, actuator, solver-budget,
  and persistence signals. Live Upkie evidence uses a versioned absolute
  threshold profile transported in WebSocket Hello; thermal/reliability stays
  explicitly unavailable rather than being inferred from a single solve;
- r217 live two-rate plant and unambiguous interaction contract. The isolated
  Python MuJoCo worker now owns a physical z=0 plane and advances five 4 ms
  contact steps for each held 50 Hz Rust WBC command; the stream remains 50 Hz.
  A two-second nominal smoke has zero numeric/fall resets, four ground contacts,
  0.0083 mm maximum penetration, a 139.2 µs observed controller step and a
  695.3 µs worker step on this host (one run, not tail evidence). The worker
  streams contact positions/normals/normal force, ground/total counts,
  penetration, sim time, solver iterations, backend/version and exact rates.
  Ctrl+drag uses a perspective ray against every rendered triangle, preserves
  the nearest surface hit as the application point, and can wrench any mapped
  MuJoCo body; Shift+drag always pans. Explicit PUSH remains for touch. The
  independent TARGET preview runs one 20 ms state-local WBC query and searches
  back along a base request if IK fails or the compiled collision envelope
  would cross z=0, exposing `LIMIT` instead of drawing an impossible pose;
- r122 interactive Upkie presentation and lifecycle hardening. One perspective
  camera supports empty-space orbit, wheel zoom, Shift-pan, and view-plane
  target reconstruction. The browser buffers and interpolates two 50 Hz frame
  snapshots in one display-cadence animation loop, throttles telemetry to
  10 Hz, coalesces drag traffic, pretransforms mesh topology, avoids unchanged
  canvas backing-store allocation, and exposes a `?perf=1` profiler. One torso
  and four actually translatable knee/wheel point targets replace the oversized
  squat control; hip origins are withheld pending a rotation gizmo. Tiny on-rig
  support/rolling, WBC/target, actuator, joint, and CPU bars preserve separate
  authority signals. Guided mode now keeps previewing through WBC infeasibility
  and typed ± target saturation, reset carries an acknowledged epoch, and lost
  transport ghosts the last state while disabling robot controls. A pure-
  standard-library Python smoke client passes the full local and tunneled
  identity → torso drag → infeasible/clamped streaming → reset → knee drag →
  release sequence. Actual browser draw p95 remains NOT RUN after the session
  moved from Desktop to CLI;
- r88 live flat-foot authority composition. The toy-humanoid adapter uses
  eight force points, two exact 20 mm support patches, and two geometry-derived
  six-row rigid-foot bases. Every raw-WBC query intersects its generalized
  acceleration bounds with the R83 position/velocity/braking envelope and
  streams the CoM, limiting patch/joint, support/stopping reserve, actuator
  utilization, hard residual, fixed task residuals, and solver work. A Python
  WebSocket eval applies the same squat and 160 mm chest-plane commands as the
  browser: 60/60 physical samples pass, support reaches exactly 20 mm for 13
  pressure ticks, stopping reserve stays above 184.709 rad/s², effort remains
  below 21.036%, and hard residual stays below 1.31e-9. R88 originally labeled
  the 7.370/10.647 ms p50/p99 sum of four subqueries as one query and therefore
  produced a false 59/60 five-millisecond miss; R90 supersedes that timing
  interpretation while retaining the physical trace. The hosted
  browser renders both support polygons and projected CoM and exposes this
  measured example authority stack in the mobile architecture review. Guided
  pose updates are state-local queries without policy or physics; a sustained
  unguided self-integration NumericalFailure is retained as a gap;
- a three-axis centroidal angular-momentum-rate task expressed directly over
  external contact moments about the system CoM, plus zero-momentum damping at
  the Python experiment boundary. A five-case G1 matrix retains raw traces for
  baseline, centroidal, support-preview, and combined policies. The best
  combined case extends the nominal prefix from 330 to 532 ticks and removes
  infeasible ticks, but its 104-tick normal-only touchdown and tracking errors
  keep the transfer gate red.

Still required:

- extending the admitted one-transfer LIPM boundary primitive into a
  multi-step robot-native footstep/DCM generator with continuous CoP
  derivatives and mutually feasible pelvis, CoM, contact, and contact-wrench
  intent;
- reducing the remaining authored-to-morphology projection envelope (currently
  4.99 mm foot position, 27.28 mm CoM position, and 12.96%/25.54% maximum
  foot/CoM acceleration residual) before promoting longer multi-step traces;
- wiring the public SE(3) floating-state transition into the primary quintic
  Controller profile (the dynamic browser adapter already uses it);
- compiling the remaining CoM-over-axle reference construction and
  wheel-position-to-pitch target generator into general signal operations;
- compiled contact-mode transition graph and a sustained tip-recovery policy;
- CRBA/ABA performance implementations;
- six-dimensional contact wrenches where patches require moments.

## Gates 6–7 — CUDA

Gate 6 has started at the CPU-mirror/ABI boundary; device execution has not.

Implemented in r75:

- frozen kernel ABI v1 for StateInput, ForwardKinematics, and CenterOfMass;
- fixed SoA state/root/frame-pose layout with explicit padded agent stride;
- separately named `CpuExactF64`, `CpuMirrorF32`, `CudaMirrorF32`, and
  `CudaThroughputF32` profiles;
- deterministic fingerprint of program, build, profile, scalar format, CPU
  ISA, layout/manifest kernel descriptor, and arithmetic flags;
- fixed f32 operation/reduction order with explicit FMA, fast math disabled,
  and denormal flushing disabled;
- preallocated, allocation-free Rust CpuMirrorF32 FK/CoM executor with typed
  invalid-agent status and no cross-agent failure propagation;
- PyO3/NumPy batch session for Python-owned oracle and performance reporting;
- independent Pinocchio f64 D3 checks over 16 toy and 16 Upkie states, plus
  exact D1 repeat, permutation, padded stride, chunking, malformed-agent
  isolation, and zero-allocation gates;
- retained kernel-only p50/p99 distributions for 1/32/256 Upkie agents.

Extended in r76:

- fixed floating tangent order `[root angular; root linear; joints]`;
- frame spatial Jacobian SoA layout with angular rows followed by
  frame-origin linear rows, plus a separate CoM Jacobian layout;
- allocation-free Rust Jacobian stage consuming the completed FK output;
- f64 core differential unit gates over toy and Upkie, including 100-run
  bitwise/zero-allocation coverage;
- independent Pinocchio LOCAL_WORLD_ALIGNED frame and CoM joint columns, with
  independently reconstructed world root columns;
- central-difference `J·v` checks that perturb root SO(3), root translation,
  and all joint coordinates without policy, integration, or physics;
- exact repeat, permutation, aligned/compact stride, 7+10 chunking, and
  malformed-agent zero/isolation gates over the complete Jacobian output;
- source-addressed CPU-mirror build fingerprint rather than a version-only
  witness;
- separate FK, Jacobian, and combined p50/p99 distributions for 1/32/256
  Upkie agents, with NumPy marshalling explicitly excluded.

Extended in r77:

- fixed SoA outputs for floating `M[g][g][agent]`, `h[g][agent]`, and
  `Ag[6][g][agent]` over the established world-expressed tangent;
- allocation-free Rust model-product traversal using precomputed FK and
  frame-origin Jacobians plus output-owned velocity/acceleration scratch;
- PyO3 batch execution of FK, Jacobian, and dynamics with the GIL detached and
  separate stage timers;
- f64 core unit comparison on toy and Upkie, exact mass symmetry, positive
  kinetic-energy witnesses, 100-run bitwise repeat, and zero allocation;
- independent Pinocchio free-flyer D3 comparison with explicit local-to-world
  tangent, covector, and configuration-dependent acceleration conversion;
- per-agent nonzero 3D gravity and velocity corpus, positive-definite mass
  gates, centroidal momentum consequence, exact permutation/padding/7+10
  chunking, and malformed-velocity isolation;
- full untrimmed 1/32/256 Upkie timing distributions and raw NPZ evidence.

Extended in r78:

- canonical `TaskOp::Point` plans automatically lower into fixed stable-ID
  batch query slots at executor construction; an explicit construction API is
  available for evaluator-only point plans;
- point stable ID, body index, and exact f32 local-offset bits are serialized
  in the kernel descriptor and therefore included in its fingerprint;
- fixed SoA position, floating point-Jacobian, and `Jdot-v` layouts, plus an
  allocation-free Rust stage that consumes admitted FK/Jacobian/dynamics
  outputs;
- GIL-detached Python batches with independent point-stage timing;
- Pinocchio BODY placement and LOCAL_WORLD_ALIGNED Jacobian comparison for
  four non-origin sites on toy and Upkie;
- f64 central-difference `Jv` and `Jdot-v` checks under constant
  world-expressed root/joint velocity without policy, integration rollout, or
  physics;
- exact stable-ID order, repeat, permutation, padding, 7+10 chunking,
  malformed-velocity zeroing/neighbor isolation, and zero allocation.

Extended in r79:

- fixed descriptor tables for point attractors and three-axis contact locks,
  preserving stable ID and point-query provenance; point tasks additionally
  preserve strict priority, physical-unit weight, and bandwidth;
- fixed SoA activation, target position/velocity/acceleration jets, emitted
  position/velocity residuals, physical desired acceleration, Jacobian, and
  `desired - Jdot-v` RHS buffers;
- critically damped point tracking with target acceleration feed-forward,
  while task weight/priority remain metadata for the later hierarchy instead
  of scaling physical rows;
- exact inactive-row masking, including ignored inactive NaNs, plus typed
  active-NaN invalidation that zeroes only the malformed agent;
- GIL-detached PyO3 emission batches and independent NumPy composition over
  two tasks and two contacts on 16 toy and 16 Upkie states;
- exact repeat, permutation, padding, 7+10 chunking, malformed-agent neighbor
  isolation, and zero measured Rust allocation;
- a separately reported least-squares shared-qdd residual that quantifies
  simultaneous row conflict without turning row correctness into a false
  feasibility, policy, physics, actuator, or reliability claim;
- an r79 mobile architecture-review authority stack that explicitly names
  Invariant contact rows, Viability and Intent attractors, lower fixed slots,
  conflict evidence, physical resources, and backend availability without an
  aggregate score.

Still required before any GPU claim:

- a working NVIDIA driver/toolkit and device execution of the staged
  CudaMirrorF32 executor;
- device D1 repeatability, CPU-mirror↔device D2, and Pinocchio D3 per stage;
- launch/warmup, permutation/padding/chunking, malformed-agent isolation,
  memory-scaling, latency/jitter, and failure-injection evidence;
- device certification for the implemented StateInput, FK/CoM, Jacobian,
  dynamics, point-query, emission, and solve stages, then actuation and
  integration manifests;
- frame atlas, distances, CoM/orientation/momentum tasks,
  force/friction/support/joint/effort inequalities, bounded hierarchical
  solve, actuation, and integration manifests;
- a separately fingerprinted CudaThroughputF32 profile whose relaxed math is
  never allowed to inherit mirror certification.

Extended in r97:

- a fixed-capacity `CpuMirrorF32` hierarchical solver over emitted point-task,
  contact, and box-bound rows, with stable order and no execute allocation;
- an explicit `FixedLevelApproximate` semantic tag distinct from the deployed
  `StrictLexicographic` f64 reference;
- a fingerprint covering descriptor, arithmetic constants, 64 hard sweeps,
  32 sweeps per active level, restoration order, and tolerance policy;
- continuous initial/best/final hard residuals, level RMS, priority drift,
  clipping/bound/row/work telemetry, and typed invalid/budget status;
- a strict admission split: a finite best candidate remains observable after
  budget exhaustion while the executable command is exactly zero;
- retained D1 replay, compatible-command f64 D3, neighbor isolation, padding,
  fixed-memory, timing-distribution, and allocation evidence in Python eval
  land with Rust owning every measured solve.

Extended in r98:

- a real fixed-level CUDA solve kernel with one complete agent per thread and
  the same stable 64-hard/32-per-active-level work contract as R97;
- fixed device bounds, command, retained-candidate, status, diagnostic, and
  global scratch buffers, with no device allocation, atomics, block barrier,
  cross-agent reduction, early exit, or CPU fallback;
- a seventh launch in the existing one-stream executor, one terminal
  synchronization, a solve manifest/capability, and fingerprint composition;
- separate source, CPU witness, compiler, runtime, and conditional device D1/D3
  gates. Source and CPU pass locally; NVRTC is unavailable, runtime is
  `no_device`, and every device gate is NOT RUN.

Extended in r99:

- an immutable `FloatingDynamicController` that composes strict floating WBC,
  generalized-to-actuator acceleration and dual effort mapping, exact prior
  commanded position/velocity/acceleration splice, analytic segment
  validation, typed Primary/Contingency/Rejected admission, and dense sampling;
- explicit double-buffered command state and caller-owned WBC/trajectory
  scratch/output, with capacity-preserving state copy and zero execute
  allocation;
- an independently validated braking contingency whose admitted feed-forward
  effort is exactly zero, plus visible observed/commanded divergence;
- a GIL-detached fixed-array Python eval session and retained 500-tick Upkie
  report with exact D1 replay, zero p/v/a splice error, `5.15e-12` maximum hard
  residual, 500 valid primary/contingency segments, 154.477/218.618/252.717 µs
  p50/p99/max, and zero Rust transaction allocation;
- no plant, policy, estimator, or contact-response integration: observed state
  remains external on every tick.

Extended in r100:

- exact analytic position minima/maxima for every quintic, including interior
  overshoot that endpoint-only validation cannot detect;
- exact actuator-polynomial mapping through the generalized-from-actuator
  transmission before joint-position validation, rather than assuming an
  identity actuator/joint index;
- fixed composable admission flags for solver slack/failure, plan expiry,
  actuator derivative limits, and primary/contingency joint-position limits;
- position-valid primary and braking-contingency admission in both dynamic and
  fixed-root controller segment paths;
- a 3-case × 100-repeat policy-/physics-free Upkie fault corpus with exact
  semantic replay, typed near-limit/expiry fallback, hard residuals below
  `5.2e-12`, sub-200 µs measured maxima, and zero transaction allocation.

Extended in r101:

- optional deterministic 1 ms-grid self-collision sweeps of both primary and
  braking joint polynomials using the compiled conservative sphere proxies;
- fixed primary/contingency collision flags plus minimum signed distance and
  typed first-violation stable pair/time evidence;
- explicit unknown-geometry policy: collision-enabled construction rejects
  unsupported authored shapes by default and requires a named override to
  validate only the represented subset;
- a three-case × 100-repeat policy-/physics-free collision corpus whose known
  16 ms primary violation selects a clear zero-effort brake, replays exactly,
  stays below `1.2e-16` hard residual, and allocates zero transaction bytes.

Extended in r102:

- compiled conservative per-sphere, per-joint center-speed coefficients and a
  maximum pair-relative speed bound from analytic quintic velocity extrema;
- an optional continuous-clearance policy using `sampled_minimum -
  relative_speed_bound × sample_period / 2` over the represented proxies;
- distinct primary/contingency continuous-clearance flags and fixed Python
  arrays, so “sampled clear but not continuously proved” cannot be mislabeled
  as an observed collision;
- a three-case × 100-repeat policy-/physics-free adversarial corpus: 20.232 mm
  sampled clearance, 19.008 mm certified lower bound, flag `0x200`, and an
  independently certified 44.450 mm zero-effort brake with exact replay, zero
  transaction allocation, and 50.055 µs retained worst execution.

Extended in r103:

- allocation-stable `advance_solved_into` command admission over an
  already-solved floating-WBC result, avoiding a duplicate dense solve in live
  adapters while preserving the complete output and command-state contract;
- 13 typed WebSocket authority capabilities, including separate sampled
  geometry, continuous clearance, and command-selection rows;
- distinct raw-WBC, command-admission, and four-query batch clocks;
- a retained 60-frame live torso-pull audit with 2.246 ms raw and 2.191 ms
  command maxima, 8.601 ms command batch maximum, and hard residuals below
  `1.4e-9`;
- explicit exposure of the current coarse collision-proxy limitation: all toy
  commands reject at pair 0/offset 0 with about 247 mm represented-proxy
  penetration while the guided pose remains visibly separate from execution.

Extended in r104:

- separate tight primitive geometry for read-only command admission while
  sphere covers remain the differential avoidance representation;
- exact sphere and authored-capsule semantics, cylinder-enclosing capsules,
  oriented-box point/segment distance, and conservative box SAT clearance;
- topology-derived adjacency through collisionless revolute/fixed carriers,
  with prismatic carriers explicitly retained;
- precomputed primitive transforms per trajectory sample and no unused
  collision Jacobian construction in command sweeps;
- named minimum and first-violation body pairs in the WebSocket authority row;
- a 60-frame trace with all sampled grids clear, 54 Primary and six certified
  Contingency selections, 20.106/19.911/20.029 mm sampled/primary-continuous/
  contingency-continuous minima, and 1.319/5.080 ms command/batch maxima.

Extended in r105:

- fixed caller-owned storage for one sampled minimum per validation primitive
  pair, reset and updated without per-tick allocation;
- pair-consistent continuous certificates that combine only a pair's own
  sampled minimum and compiled relative-speed bound;
- stable continuous-limiting pair/body provenance and pair-speed telemetry;
- a focused cross-pair regression fixture plus the existing r102 one-pair
  certificate, exact replay, and allocation gates;
- a browser-local 120-tick command-authority chart for sampled clearance,
  certified clearance, requirement, and Primary/Contingency/Rejected bands;
- a 60-frame path-dependent trace with 52 Primary, five Contingency, three
  Rejected, seven visible sampled violations, 19.710/19.601/19.999 mm minima,
  and 1.242/4.510 ms command/batch maxima.

Extended in r106:

- analytic maximum absolute quintic velocity on any closed subinterval using
  fixed-capacity polynomial roots;
- a broad-phase pair certificate that skips interval work for already-safe
  pairs;
- deterministic midpoint refinement of only unresolved pair/interval leaves,
  to an explicit maximum depth;
- fixed diagnostics for leaf intervals, midpoint pair queries, unresolved
  leaves, and maximum depth reached;
- sampled-collision precedence: base or midpoint violations retain stable
  pair/time and stop refinement immediately;
- a Python depth ladder proving 19.008→20.079 mm with exactly three midpoint
  queries, exact replay, and zero transaction allocations;
- live refinement diagnostics in the WebSocket and 120-tick browser history;
- a 60-frame trace with 55 Primary, four Contingency, one Rejected, five
  retained sampled violations, and 2.336/5.396 ms command/batch maxima.

Extended in r107:

- one tight primitive closest-feature witness owns distance, pair ID, normal,
  surface points, rigid-body feature points, relative velocity, Jacobian, and
  quality for fixed-root avoidance and command admission;
- sphere-cover avoidance is retained only as an explicitly named fallback;
- sphere/capsule radii use medial feature Jacobians, avoiding spurious angular
  velocity, while boxes use locally selected support/boundary features;
- a 321-state policy-/physics-free Python sweep matches analytic distance to
  `2.78e-17 m` and finite-difference Jacobians to `5.84e-10`;
- one tight pair replaces three cover pairs, removing 131.046 mm maximum
  overreach, 132 false influence samples, and 122 false hard-collision samples;
- 1,000 exact replays measure 0.311/0.391/6.181 µs p50/p99/max with zero timed
  allocation calls/bytes; the explicit cover fallback measures
  0.501/0.571/0.751 µs.

Extended in r108:

- the floating CPU WBC emits a local relative-degree-two hard collision row
  from the shared tight witness, including projected rigid-feature `Jdot-v`;
- closest represented pair and limiting active post-solve row remain separate
  typed evidence with distance quality, margin, relative velocity,
  required/achieved normal acceleration, residual, active count, and
  unsupported-shape count;
- a 321-state policy-/physics-free oracle activates 70 barriers, matches the
  analytic required acceleration to `5.33e−15 m/s²`, and retains a minimum
  residual of `−2.22e−16 m/s²`;
- the enabled/disabled counterfactual observes up to `62.171 m/s²` intervention
  with exact replay and zero timed allocation calls/bytes;
- the WebSocket and browser authority stack expose collision Viability
  separately from sampled/continuous command admission.

Extended in r109:

- an immutable finite dense SDF with analytic trilinear gradients, rotated-grid
  support, and explicit `Reject` or `OccupiedBoundary` unknown-space policy;
- deterministic conservative body-sphere probes compiled with stable IDs and
  explicit unsupported-shape counts;
- a separate floating world-collision HOCBF with closest/limiting probe/body,
  SDF source, proxy quality, gradient, distance/margin, velocity, bias,
  required/achieved acceleration, residual, and outside-policy evidence;
- a narrow PyO3 fixed-array session and 321-state policy-/physics-free NumPy
  oracle with exact replay and zero timed allocations;
- a visible editor SDF wall, fifteenth live authority capability, and distinct
  mobile architecture-review rows for CPU and live world authority;
- bounded failure evidence: the initially penetrating wall exhausted the
  feasibility budget, returned `MaxIterations`, zeroed the command candidate,
  and withheld the frame rather than animating an unsafe result.

Extended in r110:

- exact primary and braking joint quintics are independently sampled against
  the immutable world field at both endpoints and every 1 ms servo knot;
- sampled collision, conservative between-sample clearance failure, and
  Reject-policy unknown space have separate primary/contingency flags plus
  stable probe/body/time provenance;
- a global trilinear-field Lipschitz bound and precompiled fixed-root
  probe-speed coefficients drive pair-free continuous certificates, with
  analytic local velocity extrema and deterministic depth-bounded midpoint
  refinement for unresolved intervals;
- fixed-capacity world sweep scratch retains leaf, midpoint, unresolved, and
  reached-depth work with zero control-transaction allocations;
- the PyO3 dynamic session accepts an optional dense world field and selected
  probe bodies, and emits fixed NumPy arrays for both plans, splice boundaries,
  timing, flags, provenance, and bounded work;
- a 321-transaction independent oracle matches distance/certificate to
  `2.78e−16 m`, violation time exactly, and rate to `3.11e−15 m/s`; 2,000
  replays are exact at `15.855/22.003/30.287 µs` p50/p99/max;
- the live 17-capability contract and mobile authority stack show local world
  HOCBF, sampled world command, continuous world command, and selector as
  independent rows for primary and brake.

Extended in r111:

- a fixed-size six-DoF root-pose prediction segment stores one local tangent
  quintic anchored at the observed pose in smooth `control_world`; rotation is
  reconstructed on SO(3), never interpolated component-wise;
- primary uses the solved floating-root acceleration as a bounded
  constant-acceleration witness, while contingency independently brings the
  observed root twist to rest over the same 20 ms horizon;
- every world-SDF grid knot and adaptive midpoint evaluates the paired joint
  and root segments before FK, so root translation and rotation can create a
  collision that a fixed-root query would miss;
- continuous distance-rate evidence composes analytic joint velocity extrema,
  root linear twist, and root angular twist through a compiled conservative
  probe reach over authored joint ranges;
- the PyO3 trace retains primary/brake root endpoints and maximum twist in 36
  fixed columns. A policy-/physics-/integration-free NumPy oracle matches the
  root witness to `1.51e−18`, distance/certificate to `2.78e−16 m`, violation
  time exactly, and rate to `4.00e−15 m/s`, with exact replay and zero timed
  allocations;
- a discriminating state is `23.000 mm` clear under a frozen-root
  counterfactual but reaches `17.500 mm` under the primary root prediction,
  first violates at 11 ms, and transfers authority to a `20.250 mm` clear
  brake;
- the live contract has 18 capabilities and adds a dedicated floating-root
  prediction row before sampled/continuous world admission. It labels the
  witness as prediction rather than base actuation or plant response.

Extended in r112:

- every root prediction carries six fixed-size deterministic error-growth
  parameters: initial translation/attitude radii plus bounded linear/angular
  velocity and acceleration error;
- sampled world clearance is robustified at every grid knot and adaptive
  midpoint by the field Lipschitz bound times translation radius plus compiled
  probe reach times attitude radius;
- continuous clearance adds the analytic forecast-error growth rate to the
  nominal joint/root motion rate, so declining forecast quality consumes
  authority continuously rather than through a delayed timeout;
- PyO3 exposes 48 fixed root-prediction/error columns and independent
  primary/brake clearance erosion. The NumPy oracle reconstructs nominal
  motion, error radii, robust distance, and certificate rate with zero timed
  allocations;
- the prediction-error-only fixture is `21.000 mm` clear and selects Primary
  with zero error, but a declared `2.124 mm` horizon erosion produces
  `18.876 mm` robust clearance and rejects both plans;
- the live contract has 19 capabilities and places deterministic root forecast
  error between nominal prediction and robust world admission. It is explicitly
  neither covariance nor a probability or realization claim.

Extended in r113:

- every compiled world field carries a `WorldSceneStamp`: scene epoch, source
  timestamp, valid-from timestamp, and valid-until timestamp;
- command admission checks expected epoch, rejects future sources and invalid
  windows, enforces optional maximum age, and can require validity through the
  complete primary/brake command horizon;
- seven stable validity outcomes distinguish epoch mismatch, future source,
  not-yet-valid, expired-at-tick, horizon-expired, too-old, and malformed
  stamps; invalid evidence marks both world plans unknown and rejects;
- the PyO3 boundary adds seven fixed scene-evidence columns. A policy- and
  physics-free Python differential isolates all seven cases, checks exact
  replay, and retains allocation and timing distributions;
- the live contract has 20 capabilities and exposes scene epoch, age, horizon
  coverage, and typed validity between local world viability and floating-root
  prediction;
- scene epoch is independent of MotionProgram version and map/odom correction.
  Root prediction remains anchored in smooth `control_world`, so localization
  re-anchoring is not injected into a short-horizon collision trajectory.

Extended in r114:

- observed joint position and velocity are mapped into actuator coordinates
  through the exact compiled actuation transform and compared with the prior
  commanded segment at the exact splice time;
- position and velocity retain independent limiting actuator IDs plus signed
  contingency and reject headroom;
- a contingency breach withholds primary feed-forward effort and selects only
  the independently validated brake; a reject breach invalidates both plans
  because command-space geometry no longer adequately represents observation;
- a five-case policy-/physics-free Python differential isolates nominal,
  position warning/reject, and velocity warning/reject behavior across 10,000
  exact transactions with zero timed allocation;
- the live contract has 21 capabilities. An 80 mm guided disturbance retains
  54 nominal and six braking frames, maximum position/velocity mismatch of
  `0.2732/1.4944`, and no hard tracking reject;
- the evidence detects mismatch but deliberately does not infer saturation,
  latency, contact loss, thermal derating, calibration error, or mechanical
  failure.

Extended in r115:

- every dynamic observation carries producer and caller-mapped timestamps,
  stable source identity and sequence, and synchronization uncertainty; the
  Rust core reads no clock;
- signed age and synchronization headroom remain continuous, with causality,
  freshness, and synchronization validity retained independently;
- future, stale, or uncertain observation evidence invalidates both Primary
  and brake even when numerical q/v match the commanded splice;
- a seven-case policy-/physics-free differential covers exact and inclusive
  limits, one-nanosecond breaches, negative uncertainty, and combined faults
  across 14,000 exact transactions with zero timed allocations;
- the live contract has 22 capabilities and places observation timing ahead of
  R114 tracking in the example authority stack. The 60-frame trace retains
  source `0xb015`, sequence 8–244, 10/2 ms age/synchronization headroom, 54
  Primary plus six tracking brakes, and 2.036 ms command-admission p99.

Extended in r116:

- a fixed-capacity Rust observation ring preallocates every retained state slot
  and performs no online allocation;
- ingest validates program epoch, layout/manifold, mapped-time ordering,
  causality, freshness, synchronization, source ID, and sequence;
- equal mapped timestamps resolve by lowest stable source ID and then highest
  sequence independent of batch chunking; unsorted batches reject atomically;
- reconstruction exposes exact, shortest-manifold interpolated, bounded
  constant-velocity predicted, or held provenance with source interval,
  identity, sequence, age/synchronization headroom, and hard eligibility;
- held state remains visible but cannot authorize hard constraints;
- the Python audit passes fault, wraparound, gap, too-old, chunking, exact
  replay, and zero-allocation gates at `0.180/0.231 µs` ingest and
  `0.090/0.140 µs` query p50/p99.

Extended in r117:

- canonical history samples now retain joint q/v, root pose, and
  world-expressed root twist as one floating state;
- local cubic Hermite reconstruction uses continuous-joint and SO(3) tangents;
- every live floating-WBC model/contact/task/limit/command query consumes only
  the hard-eligible canonical reconstruction, removing the producer-state
  bypass;
- the WebSocket contract has 23 independent signals and exposes ring
  occupancy, ingest dispositions, source interval/identity/sequence,
  provenance, age/synchronization headroom, and hard eligibility;
- 60 guided frames pass with 60 exact hard-eligible reconstructions, ring
  occupancy 5–64/64, ingest 60/0/0, 54 Primary plus six brakes, and 64.602 mm
  minimum continuous world clearance.

Extended in r118:

- reconstruction evidence derives a typed downstream stamp at the actual
  reconstructed query time, rather than reusing either bracket endpoint;
- deterministic browser/WebSocket controls exercise exact 5 ms delivery,
  2.5 ms-lookback local cubic interpolation, 5 ms boundary prediction from an
  offset 10 ms producer, and a paused stale producer;
- 12 frames per admitted mode retain 60 exact, 60 interpolated, and 25 exact +
  35 predicted WBC/stream queries; every admitted state is hard-eligible and no
  held state reaches hard rows;
- prediction selects an independently validated braking contingency on 3/12
  frames while exact and interpolation remain Primary 12/12;
- stale evidence emits a typed `observation_withheld` event before WBC,
  controller time continues, and exact delivery recovers in-session;
- the Python report owns orchestration, assertions, timing distributions, and
  retained artifacts; the state/history/WBC/command guts remain Rust.

Extended in r119:

- canonical reconstruction evidence derives a validated deterministic error
  envelope from model exposure plus synchronization uncertainty;
- joint position/velocity, root translation/rotation, represented-point, and
  CoM error radii remain separately typed rather than becoming confidence;
- floating-WBC joint stopping intersects all `q ± error, v ± error` corners;
  finite support, self collision, and world collision respectively consume one
  CoM, two point, and one point radii;
- live metrics and the browser retain raw beside robust margins, so uncertainty
  consumption never overwrites geometry or becomes an aggregate health score;
- a 20-frame/mode policy-/physics-free WebSocket audit proves exact raw=robust
  identity, exact configured 2.5/5 ms growth values, monotone prediction
  erosion, typed stale withholding, and exact recovery.

Extended in r120:

- a release native sentinel sweeps fixed-state reconstruction exposure from
  0–10 ms through the same Rust bound, joint-corner, and floating-WBC APIs;
- 0–5 ms is the production-eligible range; 7.5/10 ms remains explicitly
  diagnostic because live reconstruction withholds beyond its horizon;
- a near-limit joint exposes monotone stopping-interval consumption that the
  live squat's global acceleration cap masked;
- one tight self-collision witness preserves its raw 10 mm geometry while the
  robust margin and required acceleration change continuously;
- an alternating nominal/robust order controls warmup drift while retaining
  p50/p95/p99/max, jitter, direct allocator counters, fixed-value sizes, and
  complete generalized-acceleration bitwise replay;
- Python validates monotonicity and semantic identities, hashes artifacts, and
  renders the retained report; timed control math remains Rust.

Extended in r121:

- `FloatingDynamicController` accepts the same validated fixed-size
  reconstruction-error object as the upstream WBC, including the caller-solved
  allocation-stable boundary;
- Primary and brake sampled/continuous self-collision retain raw distance and
  expose robust distance after two body-local point radii;
- Primary and brake world admission compose observation root translation and
  rotation radii with forecast growth, then expose a separate
  field-Lipschitz-scaled point-radius erosion;
- typed admission flags identify cases where observation error alone turns an
  otherwise valid self/world candidate invalid;
- tracking mismatch, scene validity, raw geometry, robust geometry, solver
  status, candidate selection, and plant realization remain separate authority;
- WebSocket metrics expose all eight Primary/brake sampled/continuous raw and
  robust command witnesses; the browser renders those identities in the live
  stack and retains robust command history;
- the live transaction uses exactly one query timestamp for reconstruction
  provenance/error, stamped admission, WBC, command geometry, and selection;
- policy-/physics-free Python audits prove the erosion identities for exact,
  interpolation, and prediction, typed stale withholding, in-session recovery,
  and four command queries per streamed frame.

Closest-feature switching, self-collision normal curvature, the world-SDF
Hessian, and voxel-feature switching remain outside the local barriers. R115
uses conservative sphere probes, a state-local short-horizon root predictor,
externally declared deterministic error bounds, and one immutable versioned
scene snapshot. Tracking thresholds are authored actuator-coordinate contracts,
not calibrated fault classifiers. R120 extends the R119 exclusive local-WBC
boundary with fixed-state continuous exposure, CPU, jitter, memory, allocation,
and replay evidence. R121 extends the same typed bound through Primary and brake
command geometry without turning deterministic growth into probability.
Statistical transport faults, estimator-calibrated covariance/error growth,
anisotropic/correlated bounds, cause-specific realization evidence, a dynamically re-solved observed-state
contingency, fixed-capacity scene replacement, interpolation between scene
epochs, command-history or forward-dynamics root prediction, moving-obstacle sweeps,
mesh CCD, authored exclusion matrices, and richer closest-feature
continuous-rate bounds remain unavailable.

Physical force/effort mirror assembly, Graph capture, actuation, integration,
and device evidence remain unavailable. The CUDA solve source/executor is
implemented but is not device-certified.

The current host exposes `nvidia-smi` but cannot communicate with a driver and
has no CUDA compiler. The capability therefore remains typed as
`CpuMirrorReadyDeviceExecutorUnavailable`; there is no hidden CPU fallback.

Revision r128 adds a separately gated CPU plant consequence without promoting
MuJoCo into the controller. Revisions r129–r130 extend the persistent Rust
Upkie session with a full-DCM authority witness, a measured actuator-facing
velocity fraction, continuous capture-pressure release of an odom station
preference, explicit `control_world`/`odom`/`map` evaluation, and map-jump
control isolation. The policy-component corpus, seven-value 10-second fraction
sweep, and full nominal/recovery/overload plant are retained under
`benchmarks/results/upkie-rooted-capture-r129/`,
`benchmarks/results/upkie-capture-fraction-sweep-r130/`, and
`benchmarks/results/upkie-rooted-capture-plant-r130/`. R131 adds the retained
typed live gateway under `benchmarks/results/live-upkie-plant-gateway-r131/`.
R132 adds bounded application-point and moment evidence under
`benchmarks/results/live-wrench-application-r132/`. R133 adds the external-plant
direction/contact envelope under
`benchmarks/results/upkie-disturbance-envelope-r133/`. R134 adds the rejected
differential-wheel planar-capture A/B under
`benchmarks/results/upkie-planar-capture-ab-r134/`. R135 adds the policy-free
contact-mode/axis authority surface under
`benchmarks/results/upkie-state-local-authority-r135/`. R136 adds the explicit
negative-deployment contingency A/B under
`benchmarks/results/upkie-fall-safe-contingency-r136/`. R137 separates and
promotes the admitted stale-command expiry under
`benchmarks/results/upkie-stale-command-expiry-ab-r137/`. Together these close
rooted sagittal capture/station composition, browser plant transport, one
three-point physical differential, ±vertical consequence, repeated sagittal
impulses, and first-boundary failure handling for the declared Upkie example.
Contact-mode transitions, lateral/general multi-contact recovery, persistent
material-point semantics, slope/delay/noise/simultaneous-disturbance sweeps,
browser frame-time/mobile visual QA, actuator/estimator calibration, and
hardware realization remain open.

R133 freezes a 20-case consequence envelope. Python owns case construction,
MuJoCo contact/integration, body-point wrench application, friction variation,
first-event termination, scoring, and retained NPZ/JSON/Markdown/HTML
artifacts; persistent Rust sessions own rooted capture/station state, floating
WBC, hierarchy, torque, timing, and allocation counters. The evaluator passes
all eleven gates with exact canonical replay, zero MuJoCo warnings, every red
row stopped at its first 45°/350 mm boundary, and zero Rust timed allocation.
Controller qualification stays separate: ten rows qualify and ten are red.
±4 N sagittal and vertical pushes recover, as do three repeated 2 N pulses;
6 N, every lateral/diagonal row, a 4 N handle push, and μ=0.03 fall. The ±2 N
lateral pair crosses at 1.985/1.990 s, while its 48.59% path difference remains
explicit. μ=0.10 recovers physically but records eight later WBC
non-admissions. First-boundary termination removes the earlier post-fall
BADQACC and 28 m launch without erasing the genuine 8–12 ms controller tails.
R134 makes the first measured lateral-authority attempt explicit without
weakening the admission boundary. Rust now owns heading-relative planar DCM,
persistent steering-direction state, bounded yaw slew, differential wheel
acceleration, and runtime-rotated contact bases; Python owns only the MuJoCo A/B
and scoring. The candidate preserves nominal and sagittal recovery and delays
all three lateral boundaries by `0.135–0.515 s`, with exact replay and zero
timed Rust allocation, but recovers none of the frozen lateral cases. Positive
`114.8/95.7/78.7 mm` lateral DCM-to-track margins remain at the three falls,
and a full six-second gain sweep is sign-specific and non-monotone. Evaluation
admission passes while controller promotion is rejected; the live controller
remains on the r133 path. A robust support/contact transition or a viability
program that captures the coupled pitch/yaw/contact dynamics was therefore the
largest remaining CPU control chunk at r134—not another local gain or per-tick
QP tuning pass. R136 subsequently makes the failure transition explicit without
yet solving that recovery action. Slopes, estimator faults,
simultaneous disturbances, calibrated actuators, and hardware remain outside
this admission.

R135 inserts a state-local authority surface beneath those plant conclusions.
The Python evaluator authors 196 immutable requests across LockedPoint,
NormalPoint, RollingPoint, and RollingWheel; six root axes; both signs; and
1/10/50/250 native acceleration units. One persistent Rust `FloatingWbcSession`
owns model products, contact and rolling rows, dynamics, hierarchy, effort
bounds, residuals, work, and allocation counting. All probes are hard-feasible
(22 `Solved`, 174 `SolvedWithSlack`), maximum hard/dynamics/contact residuals
are `1.712e-12/1.712e-12/2.461e-13`, and timed Rust allocation is zero. The
response curve and per-layer residuals quantify local ability continuously;
they are not a closed-loop recovery certificate and do not promote r134.

R136 implements the explicit failure transition required by the specification,
but keeps mechanism admission separate from deployment. Rust owns tilt,
angular-rate, height, and prior-solver pressure; bounded primary/contingency
authority slew; minimum hold; a bounded damping candidate; stale-command
expiry; typed reason flags; and a reset-only fallen latch. The six-case plant
A/B preserves nominal semantics bit-exactly and retains 4 N sagittal recovery;
every frozen failure degrades before its first boundary, a 20-tick outage
expires command authority to zero, exact replay holds, and Rust allocates zero
bytes in timed regions. Deployment is rejected because the 6 N overload falls
`3.120 s` earlier. The remaining controller chunk is therefore an independently
solved observed-state contingency or support-changing action—not indefinite
command hold, scalar fading, or local gain tuning.

R137 promotes the independently gated portion of that work: command freshness.
The live path does not blend r136's rejected damping objective. A current WBC
result executes only when admitted; Rust leases the prior admitted torque for
five rejection ticks and fades it to zero by tick twelve. The fresh
baseline/candidate/replay plant matrix preserves nominal and 4 N recovery
bit-exactly, moves every frozen adverse boundary later by `0.025–0.365 s`,
reduces maximum stale age in all four adverse rows, repeats exactly, and
allocates zero bytes in Rust timed regions. Physical recovery, contact/support
change, terminal-impact optimization, and hardware validation remain open.

R138 adds exact per-wheel support evidence to the replaceable plant and freezes
the next architecture boundary without deploying it. Named wheel subtrees are
checked against world contact independently of total MuJoCo contact count.
Sixteen pre-fall snapshots yield double, left-only, and zero support; 12 differ
from the WBC's permanent double-contact declaration. Ninety-six frozen-state
queries show all measured masks hard-feasible (`5.954e-12` maximum residual),
while stale double-contact rows produce 30 `MaxIterations` results and violation
up to `216.339`. The query replay is exact and allocation-free. A causal
contact estimator, mirrored right-only/chatter corpus, and independently
admitted support-state actions remain required before live promotion.

R139 adds the missing generic causal contact-observation contract to
`bonesaw-core` and exposes it through the fixed-shape Python boundary. It is
allocation-free, caller-timestamped, source/sequence/uncertainty checked,
fault-atomic, and keeps exact raw contact, debounced stable mode, provenance,
and hard eligibility distinct. Its mirrored/chatter/fault corpus passes.

R140–R145 demonstrate that the observation mechanism is not the action. Direct
measured-mask WBC execution creates nine green falls. Withholding every
reduced-support result preserves all green physical recoveries but accelerates
all existing falls and introduces later non-admissions. The current WBC
normal-load share has zero precursor coverage at fixed 10/20/30% thresholds.
R143 adds a generic allocation-free cached-command lease with explicit
authoring support, tick age, provenance, and typed revocation. Its
policy-/physics-free contract passes. R144 makes the five-tick lease the sole
post-loss command authority; lifetime and zero-output gates pass, but 7/10
green qualifications and 7/9 fall boundaries regress. R145 composes
0/2/4/8/16 ticks of contact grace ahead of r137's continuous freshness fade.
The zero row is execution-bit-exact with r141 and all green physical outcomes
survive, but every r137 fall remains earlier and added grace worsens aggregate
failure time. Both deployments are rejected. Live stays on r137 command
freshness. The next implementation chunk is now strictly an independently
solved pre-loss viability/support action, followed by delay/noise/dropout and
hardware sensing evidence.

R146–R150 close the next mechanism loop without promoting a recovery action.
R146's policy- and plant-step-free 245-query oracle finds strict local 250 ms
forecast descent in 15/16 retained states. R147 reduces this to a deterministic
40-query coordinate selector with strict descent in all 12 activated states,
3.808 ms p99 summed Rust query work, exact replay, and zero allocation. R148
makes the selected request a generic fixed-size Rust authority object with
activation/release hysteresis, per-axis slew, four-tick freshness, atomic
sequence rejection, exact-evidence revocation, and typed provenance. Every
nonzero request still requires a current exact WBC admission.

R149's four-arm 20-case plant study admits all 11 mechanism gates but rejects
physical promotion: 11/19 falling measured-control rows cross earlier and 395
planner ticks exceed 5 ms. R150 then admits measured reduced support only
behind a fresh executable request. This preserves all ten r137 green rows,
exact replay, finite state, and zero timed Rust allocation, but recovers no
adverse row, moves eight adverse boundaries earlier (worst `−1.065 s`), and
records a `16.239 ms` worst per-case p99 controller step. Live remains r137.
The largest CPU chunk is now a plant-relevant multi-step viability objective
with a fixed query budget amortized across ticks, not more lifetime or support
gating plumbing.

R151–R154 implement and causally audit that fixed-budget objective without
promoting it. R151 replaces the one-shot capture score with an allocation-free
Rust forecast over eight fixed knots and limits scheduling to one zero-request
query plus one three-point coordinate question per tick. Its broad wake rule is
physically regressive: five retained green rows are lost and the worst boundary
moves `−4.360 s`. R152 lets only current roll/lateral capture wake the planner,
while pitch, support, rate, yaw, actuator, joint, action, and action-change
pressures remain visible vetoes. It preserves all ten r137 green rows and
contracts the worst adverse regression to `−0.085 s`, but recovers no adverse
row and records 660 controller ticks above 5 ms.

R154 supplies the stricter four-arm comparison: retained r137 sentinel,
measured-contact control, candidate, and exact replay. All 11 mechanism gates
pass. Against the measured-contact control, earlier falling boundaries fall
from r149's 11/19 to 4/19, the worst regression contracts from `−1.490 s` to
`−0.250 s`, and loop overruns fall from 395 to 63. The result is still not a
deployment certificate: four boundaries remain earlier, the measured-contact
path itself loses most r137 green outcomes, and total exact work rises to 20,957
queries with a 2.00 MiB summed within-run RSS increase. Planner-only
active-set truncation does not materially change the tail, localizing the next
CPU task to fewer synchronous WBC polls or reused sensitivities rather than a
smaller feasibility-iteration cap. Live remains r137.

R155 moves the signed-coordinate proposal phase into generic, allocation-free
Rust state and reduces an active planner tick to two exact queries: a
same-state zero baseline plus one signed proposal. The four-arm matrix passes
all 11 mechanism gates and cuts exact planner work from r154's 20,957 queries
to 12,140. It does not cure the physical or timing boundary: 5/19 falling
measured-control rows move earlier, the worst delta is `−0.315 s`, and 61
loop overruns remain.

R156 adds a separately typed `ViabilityConfirmationState` before the r148
request supervisor. A proposal needs two directionally aligned improvements
under the same exact raw support mask; evidence loss, support change, failed
descent, excessive update gap, bad sequence, or invalid input revokes or
restarts the shadow. All 15 mechanism gates pass, only 26 ticks confirm across
the full matrix, and one support transition revokes. Physical promotion still
fails: `up_4n` is `−0.450 s`, `diagonal_4n` is `−0.120 s`, and no adverse
retained r137 row recovers. Worst per-case p99 reaches 6.08 ms in planner work
and 6.10 ms in the independent final WBC despite zero Rust allocation and zero
Python GC. Live remains r137.

R158 adds `ViabilityHybridGuardState`, a generic allocation-free gate between
proposal selection and confirmation. It tracks exact support dwell, forbids a
single-support roll request that opens the missing wheel, requires material
normal load on both wheels in double support, and fails closed on evidence,
sequence, bounds, or non-finite input. The four-arm plant corpus exercises its
admit/dwell/direction paths but not support-change revocation, and 62 deadline
overruns keep it evaluation-only despite zero earlier fall boundaries.

R157 and R159 add realization evidence without changing live authority. The generic
Rust forecast primitive atomically writes eight fixed hold/coast knots into
caller-owned storage. R157 measures confirmed proposal paths; R159 independently
emits the path implied by every fresh final exact-WBC acceleration and records
the reduced origin state, admitted support provenance, emission timing, and
allocation witness. Python retains immutable split selection, future-state
joins, support/terminal/external-force censoring, statistics, and reports.

The r159 20-case exact replay passes all nine measurement gates with 11,034
origins and 63,308 same-support comparisons, but rejects the global calibrated
envelope on frozen holdout (98.231% joint coverage; 95.681% at 240 ms; 14.275×
worst miss). Consequently there is still no typed execution certificate and no
path from forecast calibration to request or torque authority. The next CPU
mechanism is a state/command/contact-conditioned reachable error bound with new
holdout, sequence/expiry/evidence/support revocation, and a causal plant A/B.

R160 and r161 execute that static-conditioning hypothesis and reject it rather
than tuning after holdout. R160 issues on 99.026% of 57,999 new comparisons but
retains 19 misses and a 2.599×-scale largest bound. R161 adds the conjugate
position/rate state coordinate and caps every retained component bound at one
declared scale; issuance collapses to 16.054% on a second disjoint holdout and
eight issued misses remain. Sparse multidimensional cells are therefore not the
online certificate representation.

R162 adds `ViabilityExecutionMonitorState` to `bonesaw-core` and the fixed-shape
PyO3 boundary. It retains 32 eight-component normalized residuals without heap
allocation, validates sequence/configuration atomically, checks a new residual
against the envelope computed from prior samples, and only then updates the
window. Exact support change or evidence loss clears the window and restarts an
eight-sample warmup. A frozen third 13-case holdout exercises 7,532 comparable
ticks, 6,567 covered ticks, 70 causal exceedances, and 69 recoveries within
250 ms. Replay is exact in every case; measured Rust work is allocation-free
with a 6.833 µs maximum transition. This is execution-model observability, not
reachability or command authority. The next physical milestone remains an
independently solved support/contingency action, deterministic exact-solve
isolation, and the complete causal non-regression gate.

R163 wires that diagnostic into the experimental hybrid planner with a one-way
contract: `Exceeded` can only make the current proposal unavailable before
confirmation/request supervision. In the retained 20-case A/B, 70 veto ticks
overlap zero executable requests and the Rust transition remains allocation
free. Every one of the ten control-green rows survives; no first fall boundary
moves earlier, while the 6 N overload and handle disturbance move `+0.785 s`
and `+0.395 s`. The altered state trajectory later contains 26 executable
request ticks versus seven in control, which is retained as causal exposure
rather than misclassified as same-tick authority creation. Deployment still
fails decisively on 692 loops above 5 ms. The monitor/veto therefore remains
evaluation-only alongside the r158 planner; r137 freshness is still live.

R164 localizes the deterministic tail but fails the semantic admission gate.
The default-off sparse-nonzero traversal keeps Dykstra coefficient order,
norms, multipliers, stopping rules, and active-set behavior. It lowers worst
final-WBC p99 6.11→2.28 ms, worst loop max 39.5→5.12 ms, and overruns 58→1.
However, projections change 11,904,894→11,904,822 and the nominal and
forward-2 N plant traces are not bit-exact. Sparse coefficient arithmetic is
therefore useful localization, not an admitted kernel profile.

R165 targets the thousands-sweep fallback itself. When the bounded Dykstra
prefix leaves an equality-displaced seed, the candidate solves the equality
block before searching inequality violations. Projections fall to 942,630,
maximum observed fallback falls from 2,688 to eight sweeps, deadline overruns
fall 32→0 in its paired run, and worst loop p99 falls 4.76→1.65 ms. The timing mechanism passes,
but the different feasible seed changes downstream physics: 11 boundaries move
earlier, worst `−0.485 s`. Physical, kernel-profile, and controller promotion
therefore fail. The next CPU optimization must reproduce the established
projection through exact active-set equivalence, certified warm-start reuse, or
asynchronous completion—not accept a faster different feasible trajectory.

R166 bounds only speculative planner projections and can resume the exact
Dykstra prefix from its retained point and multipliers when the bounded result
is already within a declared violation threshold. On the frozen 20-case plant
matrix, worst planner p99 falls 6.008→1.290 ms, but >5 ms loops remain 58→58.
Plant execution is exact in every case; ten authority traces change because
some non-executable proposals become typed `MaxIterations` earlier. This is a
negative profile result with useful localization: planner work is not the
remaining deadline tail, and the executable WBC cannot adopt a semantic
shortcut merely because the current physical corpus happens to execute the
same torque.

R170 isolates r167's sparse kernel from planner continuation. Sparse Dykstra
traversal remains bit-exact by tracking actual signed-zero
coordinates in a fixed `u128` mask and reproducing only the omitted dense
`-0→+0` transitions. This closes r164's semantic defect: the complete 20-case
authority/plant trace, all 11,904,894 logical projections, and replay are exact,
with zero Rust allocation and Python GC. Generic-build timing improves from
60→32 loops above 5 ms, worst final-WBC p99 6.040→2.248 ms, and loop max
42.474→13.680 ms. The
portable/generic combined-loop profile remains default-off because exactness
alone does not satisfy the zero-overrun admission gate. R167 separately admits
the host-native authoritative-WBC kernel below 5 ms; its synchronous planner
scheduling remains open.

R169 evaluates session-local reuse of a terminal hard-feasibility result for a
bit-identical ordered hard problem. The key contains every coefficient, bound,
and feasibility-profile parameter as exact bits; final authority remains a
separate uncached session, feasible results recompute every soft hierarchy, and
exhausted results return the same fail-closed `MaxIterations`. Across the
host-native matrix it records 9,669 reuse hits, preserves complete authority
and plant traces plus logical work counters exactly, and changes synchronous
misses 5→2. The reuse mechanism passes its semantic/replay/allocation gates,
but the synchronous profile remains rejected because two loops exceed 5 ms.

R171 adds an explicit planner-update period to the Python evaluation adapter
and threads it through the existing Rust request and confirmation state
machines. A no-update tick is not a failed update; maximum candidate age and
confirmation gap are configured to the same bounded period, and the update bit
is retained in every plant trace. A fixed-planner end-to-end test proves the
three-tick pattern `update, hold, hold, update` and confirmation across the
declared gap. The complete period-1 versus period-3 plant A/B passes timing,
cadence, replay, finite, freshness, confirmation, hybrid-guard, allocation, and
GC gates: misses `3→0`, planner WBC queries `20,854→6,904`, worst loop
4.836 ms. Physical admission fails: all 26 executable-request ticks disappear,
nominal moves −10 ms, and left-2 N moves −95 ms. Uniform decimation remains
opt-in evaluation infrastructure and is not promoted; the next scheduler must
preserve proposal/confirmation throughput while moving cold work outside the
authority interval.

R174 closes that host-native synchronous compute complaint without decimating
planner updates or sharing mutable solver state. `SolverWorkspace` can export an
immutable hard-feasibility witness containing the exact ordered-problem key,
primal point, Dykstra row multipliers, terminal status, and logical-work
diagnostics. The independent uncapped authority workspace rebuilds and
bit-compares its own hard problem. A budget-independent exact terminal is
consumed directly; an exhausted bounded prefix is resumed only in the
bounded-to-uncapped direction, from the next projection sweep. Every soft task
and equality factor remains destination-owned. The frozen 20-case A/B records
11,185 copies: 11,131 terminal hits and 54 prefix resumptions, with no refused
copy. Complete authority, execution, replay, and 18,332,262 logical half-space
projections are exact. Synchronous overruns fall 5→0; worst final-WBC p99/max
are 2.667/2.945 ms, worst loop is 4.348 ms, witness copy is 7.134 µs worst-case,
and Rust allocation/Python GC remain zero. The host-native evaluation compute
profile is admitted. The experimental viability policy and controller remain
unpromoted; r137 stays live while independent support/contingency action,
robustness calibration, and hardware evidence remain open.
