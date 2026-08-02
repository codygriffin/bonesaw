# Bonesaw

Bonesaw is a Rust motion-rig and whole-body-control library. The current
milestone is a CPU-first proof of the architecture in
`motion-rig-architecture-spec.md`: one canonical robot model, deterministic
frame evaluation, fixed five-level task priority, bounded joint commands,
quintic servo segments, reproducible evaluations, and an interactive browser
client.

This is an engineering prototype, not a safety-rated robot controller.

## What works

- URDF import into a canonical parent-before-child skeleton.
- Strict validation of finite mass and physically valid inertia.
- Fixed-link composite inertia folding while authored frames remain queryable.
- Revolute, continuous, prismatic, and fixed joints.
- Forward kinematics, spatial/point Jacobians, CoM and CoM Jacobian.
- CPU joint-space mass matrix, potential/kinetic energy, recursive inverse
  dynamics, bias forces, and forward dynamics.
- Unified fixed- and floating-base inverse-dynamics WBC over root/joint
  acceleration, generalized effort, and 3D contact force, with hard
  rigid-body/contact-acceleration rows, an explicitly unactuated six-axis
  root, locked and rolling contact modes, unilateral normal bounds, torque
  bounds, and friction pyramids.
- Explicit actuator coordinates in canonical `MotionProgram` schema 6:
  actuator-space velocity/acceleration/effort/jerk limits, dense
  generalized-from-actuator and optional reverse transmission maps, and a
  power-consistent effort dual. URDF compilation emits a fingerprinted identity
  fallback; coupled/passive mappings are authored replacements, never hidden
  runtime assumptions. For square invertible coupled maps, fixed and floating
  WBC now enforce every actuator bound as an exact stable inequality row of
  `Gᵀ τ_generalized`; Style regularization and headroom diagnostics use the same
  actuator-space effort. Passive/overactuated force subspaces remain explicitly
  unsupported rather than being approximated as joint-coordinate boxes.
- Optional calibrated persistent actuator-resource state in Rust: current,
  copper loss, signed mechanical and ideal electrical power, exact lumped
  winding temperature, continuous physical derating, available effort,
  headroom, and utilization. The synthetic 24,000-tick mechanism fixture passes
  12/12 gates at 0.120 µs median / 0.181 µs p99 with zero allocations and
  bitwise repeat. It makes no Upkie/G1 calibration claim.
  See the [r50 actuator-resource report](benchmarks/results/actuator-resource-authority-r50/ACTUATOR_RESOURCE_AUTHORITY.md).
- Authoritative floating pose plus world-expressed root twist, with
  allocation-free constant-acceleration integration on SE(3).
- Timestamped bounded robot history with deterministic duplicate handling,
  continuous-joint interpolation, hold, and bounded constant-velocity
  prediction.
- First-class `control_world`, `odom`, and `map` rig frames plus explicit robot
  root pose; global map corrections cannot move the smooth WBC root.
- Topologically compiled midpoint, origin/orientation, and ground-projection
  derived frames.
- Fixed-capacity external-frame slot histories with SE(3) interpolation,
  bounded constant-twist prediction, and timestamped atlas reconstruction.
- Frame-to-frame pose and relative-twist queries with no graph search.
- Five fixed task levels and deterministic stable row ordering.
- A compiled scalar/vector/rotation signal-jet graph with analytic derivatives,
  explicit double-buffered filter/spring memory, canonical archive layout,
  and caller-owned zero-allocation evaluation storage.
- Resolved point/CoM/orientation/direction-aim task operations that bind stable signal-output IDs to
  fixed slots and frames at compile time, then execute signal evaluation,
  task emission, strict solve, trajectory synthesis, and semantic state commit
  in one allocation-free controller transition.
- Null-space hierarchical least squares for point, orientation, roll-free direction aim, CoM, and
  posture tasks with deterministic bound activation and same-priority
  re-optimization.
- Fixed-capacity task/constraint buffers and a caller-owned flat solver
  workspace; the measured controller transition performs zero heap allocations
  after construction, including collision-enabled operation.
- Deterministic general linear hard inequalities, bounded feasibility phase,
  active-row re-optimization, and typed contradiction reporting.
- Fixed-horizon quintic segments, polynomial root-isolated extrema, contingency
  stop segments, and exact 1 kHz structure-of-arrays sample blocks.
- CPU evaluations for end-effector reach, hierarchy conflict, walking-motion
  retargeting, replay determinism, frame-query throughput, rigid-body dynamics,
  and controller-path allocation counting.
- A Pinocchio 4.0 differential oracle covering every Upkie and official-G1
  body frame plus Jacobians, fixed/floating mass matrices, gravity,
  bias/inverse dynamics, CoM, centroidal maps, and centroidal momentum.
- A narrow PyO3/NumPy batch API: Python owns corpus generation, statistics,
  plots, and reports while Rust executes the complete tick loop into
  caller-owned fixed-shape arrays.
- A policy-free, simulator-free G1 admission pipeline: standalone authored
  root/CoM/foot references, allocation-free whole-body position and analytic
  velocity/acceleration projection, then 600 independent oracle-state floating
  inverse-dynamics WBC solves with hard finite-sole CoP margins. The retained
  schema-6 r50 corpus passes all 30 gates at 5 mm erosion with zero measured Rust
  allocations and bitwise-repeatable physical outputs. Threshold-free Rust
  evidence and Python capability curves cover hard residual, finite support,
  joint headroom, URDF actuator effort, solver budget, per-priority work and
  clipping, and warning persistence. Tight physical torque bounds produce
  bit-for-bit the same outputs as the retained loose-2000-Nm control; a 10 mm
  sensitivity trace retains the exposed margin/tracking boundary. See the
  [r50 G1 authority report](benchmarks/results/g1-oracle-wbc-admission-r50-actuation/ORACLE_WBC_ADMISSION.md).
- The r54 coupled-actuation admission reuses the four-step oracle with a clearly
  synthetic left/right ankle differential at ±14 Nm. The exact `Gᵀ` effort rows
  remain validated: 2,317/2,317 states solve or relax, 119 ticks reach at least
  99% pair utilization, power duality closes to 4.44e-15 W,
  Rust allocates nothing in the hot loop, and the repeat is bitwise identical.
  The corrected lexicographic result is a full **45/45 pass** while retaining
  explicit lower-layer slack and retracting the earlier 32/32 tracking claim,
  preserving the coupled-transmission evidence. The fixture is not presented
  as G1 calibration. See the
  [r54 coupled-actuation report](benchmarks/results/g1-multistep-oracle-r54-coupled-actuation/ORACLE_WBC_ADMISSION.md).
- The r54 sustained admission uses the unchanged r53 four-step reference,
  assembled from support-constrained Rust LIPM segments, and evaluates 2,317 independent G1 states
  without a policy, state integration, simulator, or physics rollout. All eight
  contact-transition windows and every hard dynamics/contact/friction/finite-CoP/
  effort gate pass: 870 states solve exactly, 1,447 use legal lower-task slack,
  none are infeasible or failed, support retains 5 mm, peak effort is 53.13%,
  and p50/p99/max execution is 3.511/5.876/7.832 ms. Root attitude and height
  now occupy Invariant, matching the integrated floating profile; horizontal
  root and CoM transfer remain Viability. All 43/43 gates pass. The report
  retains task p99, time-RMS, integrated error, violation ticks, and longest
  runs rather than equating full admission with exact Style/Preference tasks.
  See the [r54 four-step authority report](benchmarks/results/g1-multistep-oracle-r54/ORACLE_WBC_ADMISSION.md)
  and [authored reference report](benchmarks/results/g1-multistep-reference-r53/MULTISTEP_REFERENCE.md).
- The r55 CPU reference audit consolidates the current G1 oracle with the
  checksum-retained PlaCo, Pinocchio 4.0, and pinned upstream Upkie C++
  artifacts. It reports latency and jitter distributions, 1/2/5/10/20 ms
  deadline curves, CPU/wall time, RSS and Python trace memory, native allocation
  sentinels, task/nullspace residuals, physical headroom, solver-work
  correlations, and one-second execution windows. The report preserves the
  boundaries: Pinocchio is a rigid-body product oracle, Upkie is exact rolling-
  law parity, PlaCo is a same-corpus fixed-base behavior comparator, and G1 r54
  is policy-/integration-/physics-free floating WBC admission. See the
  [r55 CPU reference comparison](benchmarks/results/cpu-reference-comparison-r55/CPU_REFERENCE_COMPARISON.md).
- The r56 continuous-authority evaluator reuses those same 2,317 immutable G1
  states and introduces no policy, integration, simulator, or physics. Eleven
  independent signals retain raw pressure, warning/critical episodes,
  5/10/20/100/500 ms dwell curves, and 0.25/1/5 s leaky exposure. It does not
  compute an aggregate health score. Hard rows and the 20 ms admission budget
  have zero critical ticks; the nominal 5 ms boundary has 76 overruns with a
  40 ms longest episode; Preference and Style clipping persist for 990/575 ms.
  A responsive heatmap is linked from the architecture review. See the
  [r56 authority-over-time report](benchmarks/results/g1-authority-over-time-r56/AUTHORITY_OVER_TIME.md).
- The r57 CPU-tail audit repeats the exact four-step solve on the baseline and
  taskset logical CPUs 2, 4, and 6. All 56 non-timing arrays are byte-exact,
  yet p99 spans 5.746–7.742 ms, nominal overruns span 49–558 ticks, and longest
  episodes span 20–320 ms. This separates deterministic dense-kernel work from
  host frequency/contention effects and rejects a clock-dependent early exit in
  the pure core. The next timing experiment needs fixed-governor isolation and
  per-tick cycles/instructions before changing Jacobi or clipping budgets. See
  the [r57 CPU-tail report](benchmarks/results/g1-cpu-tail-r57/CPU_TAIL_STABILITY.md).
- The r58 hardware-counter audit runs five complete release admission processes
  pinned to logical CPU 4. All 56 semantic arrays remain byte-exact and retired
  instructions span only 0.001081% around a 239.807401 billion median, while
  task-clock spans 15.181–19.451 s and cycles span 62.530–79.415 billion. The
  counters deliberately retain whole-process scope and are not divided into a
  fictional per-WBC-tick number. This confirms deterministic work while
  motivating a native WBC-only counter sentinel. See the
  [r58 CPU-counter report](benchmarks/results/g1-cpu-counters-r58/CPU_COUNTER_STABILITY.md).
- The r59 native counter sentinel removes Python/reference/report work from the
  repeated region. Five release trials each run 2,000 fixed-shape floating WBC
  solves on the official 23-DOF G1 model, subtracting an adjacent one-tick
  process. The median marginal upper bound is 21.991 million instructions,
  6.099 million cycles, and 1.505 ms task-clock per additional tick; instruction
  span is only 0.000041%. All trials retain identical non-timing reports, zero
  infeasible ticks, bitwise final-repeat agreement, and zero solve allocations.
  This deliberately simpler microbenchmark does not replace r54 finite-support
  and full task-stack admission. See the
  [r59 native WBC counter report](benchmarks/results/g1-native-wbc-counter-r59/NATIVE_WBC_COUNTER.md).
- The r60 Jacobi A/B tests skipping coupling dots for cached columns already
  below the discard floor. The path is mathematically and bitwise neutral but
  adds 92,433 retired instructions per native tick (+0.420%): eligible pairs
  are too sparse to pay for the extra branch. Production keeps the original
  unconditional dot; the rejected experiment is disabled by default. The
  restored default still passes r54 43/43 with all 56 semantic arrays exact.
  See the [r60 negative A/B report](benchmarks/results/g1-jacobi-discarded-r60/JACOBI_DISCARDED_PAIR_AB.md).
- The r61 Jacobi A/B fuses the initial Frobenius and per-column energy scans.
  Both reductions remain bit-exact, but the fused two-accumulator loop adds
  58,916 retired instructions per native tick (+0.268%), likely by inhibiting
  the compiler-friendly independent reduction. Production retains the original
  separate scans; the experiment is disabled. The unchanged default remains
  r54 43/43 and 56-array exact. See the
  [r61 negative A/B report](benchmarks/results/g1-jacobi-energy-scan-r61/JACOBI_ENERGY_SCAN_AB.md).
- The r62 actuator-realization envelope replays all 2,317 × 23 r54 effort
  samples through allocation-free Rust first-order bandwidth, slew, and
  instantaneous availability constraints. It runs no robot policy, state
  integration, rigid-body physics, or contact simulation. Ideal response is
  bit-exact; synthetic 20/10/5 Hz profiles recover after contact edges within
  10/25/55 ms, while the 5 Hz case has 108 slew-limited ticks. A deliberately
  adverse 25%-availability case clips on 1,623 ticks and sustains >5% normalized
  error for 2.145 s. These are sensitivity parameters, not G1 calibration or a
  body-tracking claim. See the
  [r62 realization report](benchmarks/results/g1-actuator-realization-r62/G1_ACTUATOR_REALIZATION.md).
- The r63 state-local acceleration consequence consumes those six r62 effort
  traces at the same immutable r54 states. Its compact Rust decision layout is
  `[generalized acceleration, contact force]`: realized effort is moved into
  the floating dynamics right-hand side, while dynamics and active
  locked-contact acceleration remain exact equalities. Acceleration bounds,
  unilateral normal force, friction, and the 5 mm finite-patch margin are
  retained as four separate continuous signals rather than collapsed into a
  QP timeout. Ideal effort reconstructs the admitted acceleration within
  `2.43e-9`; the synthetic 20/10/5 Hz cases produce `4.58/7.30/10.73`
  generalized-acceleration RMS error and `75/105/510 ms` maximum 1%-bound edge
  recovery. The 25% case sustains >1% pressure for `5.655 s`. All eight
  mechanism gates pass, the 128-tick repeat witness is bit-exact, and measured
  Rust steps allocate zero bytes. This remains policy-, integration-, contact
  simulation-, and rollout-free sensitivity evidence, not G1 calibration or a
  stability claim. See the
  [r63 consequence report](benchmarks/results/g1-constrained-acceleration-r63/G1_CONSTRAINED_ACCELERATION.md).
- The r64 independent differential oracle checks that compact fixed-effort
  boundary at 48 immutable G1 states: all three ticks around every contact edge
  plus evenly distributed and adverse-pressure samples. Pinocchio 4.0
  independently rebuilds floating mass/bias, eight sole-point Jacobians,
  frame/CoM Jacobians, and finite-difference `Jdot-v`; NumPy independently
  rebuilds the equality seed and four strict nullspace levels. It consumes no
  Bonesaw model product or solver workspace. Across ideal, 20 Hz, 5 Hz, and
  adverse 25%-availability profiles, Rust solutions evaluated with Pinocchio
  products stay below `6.30e-9` dynamics and `4.38e-10` contact residual.
  Independent solutions differ by at most `1.26e-9` in generalized
  acceleration and `3.05e-7 N` in contact force. All 6/6 gates pass with
  checksum-retained sources and exact NumPy repeat witnesses. This remains a
  state-local equation/optimizer check—not actuator calibration, integrated
  body response, or stability evidence. See the
  [r64 differential report](benchmarks/results/g1-pinocchio-fixed-effort-r64/G1_PINOCCHIO_FIXED_EFFORT.md).
- The r65 exact-zero-row A/B removes only the inactive vertical row from the
  horizontal-only CoM task. All 56 non-timing arrays remain bit-exact and host
  time improves, but five pinned process pairs show only `-0.00007%` median
  retired instructions and one pair regresses by `+0.00034%`. The candidate is
  rejected and remains available only as an explicit experiment. See the
  [r65 negative A/B report](benchmarks/results/g1-zero-task-row-compaction-r65/ZERO_TASK_ROW_COMPACTION_AB.md).
- The r66 promoted compaction additionally removes satisfied fixed-bound rows
  only at terminal Style, where they are constant over the feasible set and no
  lower priority consumes a projector. It avoids 20,176 dense row-instances in
  the retained four-step corpus. Across five alternating pinned pairs, every
  candidate process retires fewer instructions; the median falls `3.861%` and
  process CPU falls `3.61%`, while host-sensitive p99 remains flat (`+0.12%`).
  Production passes 43/43 admission, 8/8 consequence, and 6/6 independent
  Pinocchio/NumPy gates with zero allocations. Status, clipping, pseudoinverse
  counts, active-set work, and double-support physical arrays remain exact;
  worst single-support acceleration/effort/normal-force deltas are
  `1.50e-7`/`1.46e-8`/`5.69e-7`. See the
  [r66 promotion report](benchmarks/results/g1-resolved-task-row-compaction-r66/RESOLVED_TASK_ROW_COMPACTION.md).
- The r67 coincident-step-limit experiment freezes simultaneous hard limits
  before rebuilding a projected task inverse. A two-coordinate unit witness
  reduces three inverse evaluations to two, but the immutable 2,317-state G1
  admission executes exactly 16,809 pseudoinverses and 10,604 clipped steps in
  both builds. All physical/decision/authority arrays are bit-exact, while two
  Jacobi-sweep diagnostic samples increase the aggregate by one. The experiment
  is rejected and remains opt-in; r66 stays production. See the
  [r67 negative audit](benchmarks/results/g1-coincident-step-limit-r67/COINCIDENT_STEP_LIMIT_AUDIT.md).
- R68 rejects approximate projected-gradient reoptimization at the unit
  boundary: it changes the monotonic active set and produces `1.24e-7` and
  `1.69e-7` hard-constraint violations in two established dynamic-WBC tests.
  R69 replaces approximation with an analytic exact-task-nullspace repair.
  The redundant unit witness drops from two pseudoinverses to one and all 117
  core tests pass, but every real G1 sequential hit changes the task optimum:
  control and candidate both execute 16,809 pseudoinverses, 122,031 Jacobi
  sweeps, and 10,604 clipped steps, with all 56 non-timing arrays bit-exact.
  Both candidates remain non-production. See the
  [r69 audit](benchmarks/results/g1-task-nullspace-repair-r69/TASK_NULLSPACE_REPAIR_AUDIT.md).
- R70/R71 test factorization-level alternatives. The guarded r70 row-Gram
  Cholesky path passes 43/43 admission and the independent r64 6/6 oracle while
  reducing reported Jacobi sweeps by 39.27%, but its changed arithmetic selects
  different monotonic active sets, changes 22/56 non-timing arrays, and fails
  the established r63 replay contract. R71 is an arithmetic-exact one-row
  specialization: all 56 arrays remain bit-exact in the corpus and across five
  pinned A/B pairs, but retired instructions change by only `+0.000103%` at the
  median with mixed pair signs. Both are rejected; r66 remained the production
  baseline at that stage and now composes with the promoted r72 path. See
  the [r70/r71 factorization audit](benchmarks/results/g1-projected-factorization-r71/PROJECTED_FACTORIZATION_AUDIT.md).
- R72 promotes slice-addressed one-sided-Jacobi column pairs. It preserves the
  exact pair traversal, scalar accumulation, rotation, energy re-anchoring,
  singular truncation, and hard-limit comparison order while removing repeated
  flat-index address/bounds work. All 56 non-timing arrays are bit-exact across
  the 2,317-state corpus and five pinned A/B pairs. Every pair retires 30.492%
  fewer instructions; process CPU/p50/p99 fall 6.28%/6.28%/6.98%. A separate
  native 58-variable sentinel reports 22.022 → 15.807 M instructions/tick
  (`-28.22%`) with identical semantics and zero allocation. The slice path is
  production default; `jacobi-column-slice-control` retains the flat-index
  control. See the [r72 promotion report](benchmarks/results/g1-jacobi-column-slice-r72/JACOBI_COLUMN_SLICE_AUDIT.md).
- R73/R74 re-profile that faster production build before changing another
  kernel. A 5,324-sample native profile attributes 8.21% of cycles to general
  dense products. R73 paired column iterators are exact but instruction-neutral
  (`+0.0000014%` median with mixed signs) and are rejected. R74 instead slices
  the left, right, and output rows of every dense product while preserving the
  row/shared/column accumulation order. Five complete-process pairs retain all
  56 fields exactly and reduce instructions by `4.104%`; the native 58-variable
  sentinel moves 15.807 → 15.317 M instructions/tick (`-3.098%`) with exact
  semantics and zero allocation. R74 is production default and
  `dense-multiply-row-slice-control` retains the old path. See the
  [r74 promotion report](benchmarks/results/g1-dense-multiply-row-slice-r74/DENSE_MULTIPLY_ROW_SLICE_AUDIT.md).
- R75 freezes the first `bonesaw-cuda` batch contract without making a false
  GPU claim. A preallocated Rust `CpuMirrorF32` executor implements
  StateInput/FK/CoM over fixed SoA `state[coordinate][agent]` and
  `pose[frame][component][agent]` layouts with typed per-agent failures,
  explicit FMA/no-fast-math policy, and a deterministic backend fingerprint.
  Independent Pinocchio f64 checks pass for 16 toy and 16 Upkie states;
  repeat bytes, permutation, padded stride, chunking, malformed-agent
  isolation, and zero execute allocation are exact. Upkie kernel-only
  p50/p99 is 6.732/8.577 µs at one agent, 191.101/211.130 µs at 32, and
  1.985/2.648 ms at 256 over 300 calls. CUDA device D1/D2/D3 remain NOT RUN
  because this host has no working driver/compiler; CPU execution is never
  reported as a device pass. See the
  [r75 batch ABI report](benchmarks/results/cuda-batch-abi-r75/CUDA_BATCH_ABI_AUDIT.md).
- R76 extends that contract through floating frame-origin and CoM Jacobians.
  The fixed output is
  `frame_jacobian[body][angular/linear][root-angular/root-linear/joints][agent]`
  in `control_world`. Pinocchio f64 checks over 16 toy and 16 Upkie states
  bound maximum frame/CoM error to `1.53e-7/1.79e-7` and
  `9.30e-8/5.17e-8`; a separate central-difference `J·v` property perturbs
  root orientation, root translation, and every joint without policy or
  physics. Full Jacobian outputs remain bit-exact under repeat, permutation,
  padded stride, and 7+10 chunking; a NaN agent is zeroed and cannot affect a
  neighbor. With the GIL detached during both Rust stages, Upkie FK+Jacobian
  p50/p99 is `20.238/26.782 µs` at one agent, `492.825/662.543 µs` at 32,
  and `4.186/5.949 ms` at 256, with zero Rust allocations. The retained
  256-agent scheduler tail is reported rather than promoted as throughput. See the
  [r76 Jacobian report](benchmarks/results/cuda-jacobian-abi-r76/CUDA_JACOBIAN_ABI_AUDIT.md).
- R77 extends the same fixed batch ABI through floating mass matrix `M(q)`,
  bias force `h(q,v,g)`, and centroidal momentum map `Ag(q)`. The Rust stage
  consumes precomputed FK/Jacobians, owns all scratch, and allocates zero bytes.
  An independent Pinocchio free-flyer oracle explicitly converts its local
  `[linear; angular]` tangent and configuration-dependent acceleration into
  Bonesaw's world `[angular; linear]` convention. Across 16 toy and 16 Upkie
  states, worst absolute M/h/Ag errors are `8.40e-6/3.75e-5/6.56e-6` and
  `5.55e-7/5.69e-6/3.24e-7`; every declared D3 gate passes. Mass matrices are
  bit-exact symmetric and positive definite. Repeat, permutation, padded
  stride, 7+10 chunking, and NaN-agent isolation are exact. Upkie full
  FK+Jacobian+dynamics p50/p99 is `49.784/57.391 µs` at one agent,
  `1.379/1.727 ms` at 32, and `11.417/11.873 ms` at 256 over 200 calls, with
  the untrimmed jitter tail retained. CUDA remains NOT RUN. See the
  [r77 dynamics report](benchmarks/results/cuda-dynamics-abi-r77/CUDA_DYNAMICS_ABI_AUDIT.md).
- R78 adds compiler-resolved point-query slots, the first bridge from generic
  model products to contact/end-effector row emission. Each stable ID, body
  frame, and local offset is frozen into the batch descriptor and its exact
  f32 constants affect the kernel fingerprint. Preallocated Rust emits world
  point position, floating point Jacobian, and kinematic `Jdot-v`. Four
  non-origin sites on each of toy and Upkie pass independent Pinocchio
  placement/Jacobian and f64 central-difference `Jv`/`Jdot-v` gates over 16
  states; worst absolute position/J/Jv/bias errors are
  `1.71e-7/1.51e-7/4.97e-8/2.65e-8` for toy and
  `8.57e-8/1.14e-7/5.74e-8/2.50e-8` for Upkie. Repeat, permutation, padded
  stride, 7+10 chunking, and malformed-agent isolation are exact, with zero
  measured allocations. Four Upkie point slots cost p50/p99
  `1.182/1.224 µs`, `25.098/30.250 µs`, and `213.178/251.488 µs` at 1/32/256
  agents. See the
  [r78 point-query report](benchmarks/results/cuda-point-query-abi-r78/CUDA_POINT_QUERY_ABI_AUDIT.md).
- The same finite-sole constraint in the ordinary floating session: established
  contacts construct hard patch rows for the primary and normal-fallback solve,
  release removes both force and patch constraints, and the achieved margin is
  streamed beside solver status. The r46 160-tick toe-step is functionally
  green at 5 mm with zero contingency/rejection, but remains explicitly timing
  red at 5.49–5.57 ms p99 against the unchanged 5 ms deadline.
- An isolated PlaCo reference comparison retaining raw per-step latency,
  jitter, tracking, memory, CPU, fault, context-switch, and GC traces.
- A pinned official Upkie C++ controller worker: 100,000 shared sequential
  inputs produce canonically bitwise-identical wheel commands when the Rust
  law uses upstream parameters. Live tuning deltas, temporal windows, latency,
  jitter, RSS, CPU, faults, context switches, and raw traces are retained.
- Conservative primitive self-distance samples with analytic Jacobian rows,
  hard control-barrier inequalities, soft repeller tasks, dense segment
  clearance validation, and
  differential-drive mapping for named wheel joints.
- A browser rig editor streaming targets and frame snapshots over one
  WebSocket, with empty-canvas orthographic orbit, camera-plane joint pulls,
  and a draggable torso backed by the floating contact WBC. Revision r48
  promotes 41 authored URDF visuals into the canonical Rust model, including
  25 instances of ten checksum-pinned Upkie STL assets and their materials.
  The browser loads and deterministically simplifies each unique mesh once;
  only transforms and diagnostics remain on the 50 Hz stream. BODY is the
  default viewport layer and the skeletal RIG remains independently
  switchable. Root height still comes from the exact collision envelope so
  wheel geometry and the ground grid share `z=0`.
- A checksum-pinned official Unitree G1 23-DOF walking target with authored
  mass, inertia, joint limits, and four-point sole geometry. Its canonical
  archive round-trips bit-for-bit, and each sole keeps four friction-limited
  force variables while emitting a rank-minimal six-row rigid-contact basis.
- A typed live-authority WebSocket contract with ten stable capability IDs.
  The Rust adapter distinguishes measured, unavailable, and unmodeled signals,
  and labels idle preview separately from raw dynamic WBC evidence. The mobile
  architecture review retains a complete measured example authority stack.
- An opt-in construction-time rigid-patch basis compiler. It derives exactly
  six independent rows from same-frame point geometry, rejects deficient
  patches, fingerprints the chosen modes, and keeps execution allocation-free.
  The current maximum-σmin selection score is provisional because its timing
  advantage is batch-size dependent; see the r87 report rather than treating
  geometric conditioning as a universal solve-work proxy.
- An r88 flat-foot state-local authority adapter. The Rust server constructs
  two four-point finite support patches, composes the joint stopping envelope
  on every query, and streams the CoM, limiting patch/joint, margins, task
  residuals, effort use, and bounded solver work independently. The browser
  renders both support polygons and the projected CoM. Its guided pose path is
  deliberately policy- and physics-free; an unguided self-integration drift
  failure is retained as a gap rather than presented as body-response proof.
- An r89 fresh same-session reference audit. Bonesaw and PlaCo each run three
  shared 5,000-tick fixed-base corpora in isolated processes; raw latency,
  jitter, memory, CPU, GC, tracking, and per-step arrays are retained. The
  hosted report adds seven responsive execution-over-time charts and 600
  lossless one-second window rows. Independent Pinocchio products pass at
  roundoff and the aligned upstream Upkie law has zero canonical bit
  mismatches over 100,000 samples. Results remain boundary-labeled: lower
  Bonesaw latency/RSS does not erase PlaCo's lower tracking error on two tasks.
- An r90 correction to live compute authority. A streamed 50 Hz flat-foot
  frame contains four 5 ms state-local queries; the server now times them
  independently and streams maximum-query latency, query count, and summed
  batch work. On the unchanged 60-frame disturbance, maximum-query p50/p99 is
  `1.923/3.076 ms` with `0/60` above 5 ms, while four-query batch p50/p99 is
  `7.349/9.148 ms` with `0/60` above 20 ms. Physical outputs are identical to
  r88. The old four-query sum is no longer mislabeled as one query.
- An r91 first real CUDA stage. `bonesaw-cuda` embeds deterministic PTX for
  fixed-layout StateInput, dynamically resolves the CUDA Driver API, owns its
  context/module/five device buffers, validates finite f32 inputs on device,
  and zeros invalid or padded lanes without atomics or cross-agent reduction.
  The paired CPU mirror passes repeat, permutation, isolation, and zero-allocation
  gates. This host reports `no_device`, so device JIT, D1/D2, launch, timing,
  and CUDA Graph gates remain explicitly NOT RUN; full `CudaMirrorF32` stays false.
- An r92 generic CUDA FK/CoM stage boundary. Rust now packs any admitted
  topological fixed/revolute/prismatic tree into immutable f32 constants and
  owns a fixed-buffer, no-fallback StateInput→FK/CoM Driver API executor.
  NVRTC is loaded dynamically, one thread owns one complete agent, and the
  kernel uses no atomics, block barrier, cross-agent reduction, or device
  allocation. The source/ABI and 200-call CPU repeat/isolation/padding/allocation
  witness pass. This host has no NVRTC library and reports `no_device`, so
  generated PTX and device D1/D2/timing/memory/Graph remain NOT RUN. See the
  [r92 FK/CoM report](benchmarks/results/cuda-fk-com-r92/CUDA_FK_COM_AUDIT.md).
- An r93 CUDA floating-kinematics boundary. A deterministic parent-joint table
  now extends the fixed-buffer StateInput→FK/CoM stream through every
  frame-origin Jacobian and the center-of-mass Jacobian, using the frozen
  `[angular; linear]` row and `[root angular; root linear; joints]` column
  conventions. The 200-call CPU witness passes complete-output repeat,
  Pinocchio/finite-difference, invalid-neighbor, padding, and allocation gates;
  both CUDA sources pass static ABI policy. This host has no NVRTC library and
  reports `no_device`, so generated PTX and device D1/D3, timing, memory, and
  Graph gates remain NOT RUN. See the
  [r93 CUDA kinematics report](benchmarks/results/cuda-kinematics-r93/CUDA_KINEMATICS_AUDIT.md).
- An r94 CUDA floating-dynamics boundary. The immutable model pack now includes
  each body's inertia about its center of mass, and the fixed-buffer executor
  extends the one-stream pipeline through `M(q)`, `h(q,v,g)`, and `Ag(q)`.
  Per-agent world root/joint velocity and gravity feed a deterministic
  zero-acceleration spatial recursion; one thread still owns one complete
  agent. The 200-call CPU witness passes complete-output repeat, Pinocchio and
  physical-identity gates, invalid-neighbor isolation, padding, and allocation
  gates. All three CUDA sources pass static ABI policy, but this host has no
  NVRTC library and reports `no_device`, so generated PTX and device D1/D3,
  timing, memory, and Graph remain NOT RUN. See the
  [r94 CUDA dynamics report](benchmarks/results/cuda-dynamics-r94/CUDA_DYNAMICS_AUDIT.md).
- An r95 CUDA fixed-point-product boundary. Four compiler-resolved frame/local
  offset sites extend the same stream through world position, floating point
  Jacobian, and `Jdot-v`, with immutable query constants and fixed output
  buffers. The 200-call CPU witness and f64 position/J/Jdot-v oracle pass, as
  do source policy, padding, isolation, and allocation gates. All four NVRTC
  probes are `library_unavailable` and runtime is `no_device`, so generated PTX
  and device D1/D3, timing, memory, and Graph remain NOT RUN. See the
  [r95 CUDA point report](benchmarks/results/cuda-point-queries-r95/CUDA_POINT_QUERIES_AUDIT.md).
- An r96 retained CUDA emission boundary. Fixed point-task/contact metadata
  is uploaded once; runtime masks and target jets emit deterministic task and
  mode-filtered contact rows into fixed buffers in the same stream. Source
  policy, 200-call byte replay, a zero-error independent row oracle, malformed
  input isolation, deterministic zeros, and allocation gates pass. This host
  cannot compile or launch CUDA, so device D1/D3 remain NOT RUN. See the
  [r96 CUDA emission report](benchmarks/results/cuda-emission-r96/CUDA_EMISSION_AUDIT.md).
- An r97 deterministic `CpuMirrorF32` hierarchy before CUDA solve. Stable row
  order, 64 hard sweeps, 32 sweeps per active soft level, f32/FMA arithmetic,
  continuous initial/best/final hard residuals, prior-level preservation, and
  candidate-versus-command admission are fingerprinted and allocation-free.
  A seven-agent corpus passes 500-call output replay and compatible-command D3
  against strict `CpuExactF64`; a budget-exhausted candidate remains diagnostic
  and the command stays zero. See the
  [r97 mirror solve report](benchmarks/results/cpu-mirror-solve-r97/CPU_MIRROR_SOLVE_AUDIT.md).
- An r98 no-fallback CUDA fixed-level solve boundary. One full agent per CUDA
  thread executes the fingerprinted 64 hard and 32-per-active-level sweep
  contract using fixed global scratch and output buffers, preserving the
  candidate-versus-command admission split and continuous diagnostics. Source
  policy and the retained CPU witness pass; all six NVRTC probes are
  `library_unavailable` and runtime is `no_device`, so generated PTX and device
  D1/D3, memory, timing, isolation, and Graph gates are explicitly NOT RUN. See
  the [r98 CUDA solve report](benchmarks/results/cuda-solve-r98/CUDA_SOLVE_AUDIT.md).
- An r99 integrated floating CPU command transaction. The strict Upkie WBC
  query maps joint acceleration and effort into actuator space, splices from
  exact prior commanded position/velocity/acceleration, validates primary and
  braking-contingency quintics analytically, and emits one contiguous 20×1 ms
  block without integrating a plant. The retained 500-tick corpus passes D1,
  hard-residual, splice, contingency, deadline, and zero-allocation gates. See
  the [r99 dynamic advance report](benchmarks/results/dynamic-advance-r99/DYNAMIC_ADVANCE_AUDIT.md).
- An r100 analytic dynamic-admission gate. Every actuator quintic is mapped
  back through the exact transmission and checked against joint-position
  extrema, including interior overshoot. Fixed admission flags keep solver,
  expiry, actuator-derivative, and joint-position causes separate. A retained
  3-case × 100-repeat Upkie corpus admits nominal primary, selects safe braking
  for a near-limit primary and an expired plan, replays exactly, and allocates
  zero transaction bytes. See the [r100 fault report](benchmarks/results/dynamic-admission-r100/DYNAMIC_ADMISSION_AUDIT.md).
- An r101 deterministic dynamic self-collision gate. Primary and braking
  segments are independently swept over the compiled conservative proxies on
  the exact 1 ms servo grid, with typed first-violation pair/time and separate
  clearance evidence. A known 16 ms primary collision selects a clear,
  zero-effort braking contingency; disabled or unsupported geometry follows an
  explicit policy. See the [r101 collision report](benchmarks/results/dynamic-collision-r101/DYNAMIC_COLLISION_AUDIT.md).
- An r102 conservative between-sample clearance gate. Raw 1 ms sample evidence
  remains separate from a compiled relative-center-speed certificate. An
  adversarial primary is clear at all 21 samples (20.232 mm) but has only
  19.008 mm provable clearance; `0x200` selects a 44.450 mm-certified brake,
  while explicit grid-only policy selects the same primary. See the
  [r102 continuous-clearance report](benchmarks/results/dynamic-continuous-clearance-r102/DYNAMIC_CONTINUOUS_CLEARANCE_AUDIT.md).
- An r103 live command-authority stream. The WebSocket contract exposes sampled
  geometry, continuous clearance, and typed command selection as three
  independent browser rows. It reuses the already-solved WBC result rather
  than repeating the physical solve, while retaining separate raw, command,
  and frame-batch clocks. The first 60-frame toy trace deliberately exposes a
  coarse-proxy rejection instead of animating it as executable. See the
  [r103 live stream report](benchmarks/results/live-command-authority-r103/LIVE_COMMAND_AUTHORITY_AUDIT.md).
- An r104 tight primitive admission path. Read-only command sweeps use one
  supported primitive per authored shape while deterministic sphere covers
  remain available for avoidance rows. Collisionless revolute/fixed carrier
  links collapse into physical adjacency; prismatic carriers remain tested.
  In the 60-frame, 80 mm torso trace, all sampled grids are clear (20.106 mm
  minimum), 54 frames select Primary, and six select a continuously certified
  brake when the primary lower bound reaches 19.911 mm. Command admission max
  is 1.319 ms and the four-query batch max is 5.080 ms. See the
  [r104 primitive report](benchmarks/results/live-command-authority-r104/LIVE_COMMAND_AUTHORITY_AUDIT.md).
- An r105 pair-tight continuous certificate and retained authority timeline.
  Every primitive pair keeps its own sampled minimum in caller-owned Rust
  scratch and subtracts only that pair's conservative speed reserve. The live
  contract names the limiting pair/bodies and streams its speed; the browser
  retains 120 ticks of sampled clearance, certified clearance, requirement,
  and selection. The 80 mm trace records 52 Primary, five Contingency, and
  three Rejected transactions: admitting more primary motion changes future
  command state and exposes seven later chest↔forearm sample violations rather
  than turning a local certificate improvement into a closed-loop claim.
  Command/batch maxima are 1.242/4.510 ms. See the
  [r105 pair-tight report](benchmarks/results/live-command-authority-r105/LIVE_COMMAND_AUTHORITY_AUDIT.md).
- An r106 bounded adaptive pair/interval certificate. The cheap pair-consistent
  bound skips safe pairs; only unresolved leaves evaluate analytic local
  velocity extrema and midpoint primitive distance, to a declared depth.
  Identical clear-grid cases improve 19.008→19.620→19.926→20.079 mm at depths
  0→1→2→3 using exactly 0→1→2→3 midpoint pair queries. A penetrating midpoint
  becomes sampled pair/time evidence, and any sampled failure stops further
  refinement. The 60-frame live trace retains 55 Primary, four Contingency,
  one Rejected, and five sampled violations while passing separate raw,
  command, and batch clocks. See the [r106 adaptive corpus](benchmarks/results/dynamic-adaptive-clearance-r106/DYNAMIC_ADAPTIVE_CLEARANCE_AUDIT.md)
  and [r106 live report](benchmarks/results/live-command-authority-r106/LIVE_COMMAND_AUTHORITY_AUDIT.md).
- An r107 shared tight-primitive avoidance witness. Fixed-root differential
  barriers and command admission now consume the same stable primitive pair,
  signed distance, closest features, normal, relative velocity, and analytic
  Jacobian; sphere-cover emission remains a separately named fallback. A
  321-state box–sphere sweep matches analytic distance to `2.78e-17 m` and
  finite differences to `5.84e-10`, removes 131.046 mm maximum cover overreach
  plus 132 false influence and 122 false collision samples, replays exactly,
  and allocates zero calls/bytes. See the
  [r107 shared-geometry report](benchmarks/results/tight-primitive-avoidance-r107/TIGHT_PRIMITIVE_AVOIDANCE_AUDIT.md).
- An r108 floating collision-viability barrier. The floating CPU WBC emits a
  locally linearized relative-degree-two hard row from the same tight witness
  and reports closest/limiting pair provenance, quality, active-pair count,
  relative velocity, required/achieved normal acceleration, post-solve
  residual, and unsupported-shape count. A 321-state policy-/physics-free
  oracle passes with zero timed allocations and exact replay; the browser
  streams the row separately from trajectory command admission. See the
  [r108 floating-barrier report](benchmarks/results/floating-collision-barrier-r108/FLOATING_COLLISION_BARRIER_AUDIT.md)
  and [r108 live authority report](benchmarks/results/live-collision-barrier-r108/LIVE_COLLISION_BARRIER_AUDIT.md).
- An r109 immutable CPU world-SDF barrier. A finite dense field provides
  analytic trilinear gradients in `control_world`; explicit `Reject` and
  `OccupiedBoundary` policies prevent unknown space from becoming silently
  free. Deterministic conservative body probes feed a separate floating HOCBF
  and typed closest/limiting probe, source, proxy, gradient, margin, residual,
  unsupported-geometry, and outside-policy evidence. A 321-state independent
  NumPy oracle passes with exact replay and zero timed allocations. The live
  editor renders the wall and exposes the fifteenth authority capability
  separately from self-collision and command admission. See the
  [r109 world-SDF report](benchmarks/results/world-sdf-barrier-r109/WORLD_SDF_BARRIER_AUDIT.md)
  and [r109 live report](benchmarks/results/live-world-sdf-r109/LIVE_WORLD_SDF_AUDIT.md).
- An r110 swept world-SDF command gate. The exact mapped primary and braking
  quintics are scanned independently at every 1 ms knot and conservatively
  certified between knots with a measured field Lipschitz bound, precompiled
  probe-speed coefficients, analytic local velocity extrema, and bounded
  midpoint refinement. Sampled collision, continuous-clearance uncertainty,
  and unknown space retain different flags and probe/body/time witnesses. The
  321-transaction NumPy oracle passes with exact replay and zero timed
  allocations; the live 17-capability stack keeps local world viability,
  sampled command world geometry, continuous command world clearance, and
  final selection separate. See the
  [r110 CPU report](benchmarks/results/world-sdf-command-r110/WORLD_SDF_COMMAND_AUDIT.md)
  and [r110 live report](benchmarks/results/live-world-sdf-command-r110/LIVE_WORLD_SDF_AUDIT.md).
- An r111 floating-root world-SDF command gate. Each primary/brake actuator
  quintic now carries an explicit local-SE(3) root prediction in smooth
  `control_world`; grid knots and adaptive midpoints evaluate both before FK.
  Root linear and angular twist participate in the continuous certificate,
  while the browser keeps prediction separate from base actuation and plant
  response. A fixed-root-clear 23.000 mm fixture becomes a measured 17.500 mm
  primary collision and transfers to a 20.250 mm brake. The independent NumPy
  oracle and exact replay remain policy-/physics-free and zero-allocation. See
  the [r111 CPU report](benchmarks/results/world-sdf-root-command-r111/WORLD_SDF_COMMAND_AUDIT.md)
  and [r111 live report](benchmarks/results/live-world-sdf-root-command-r111/LIVE_WORLD_SDF_AUDIT.md).
- An r112 deterministic floating-root forecast-error gate. Six fixed-size
  translation/attitude error-growth bounds robustify every sampled and
  continuous world-SDF witness through probe-specific clearance erosion. A
  zero-error 21.000 mm Primary fixture becomes 18.876 mm and Reject under a
  declared 2.124 mm horizon erosion, without a policy, physics, integration,
  allocations, covariance, or probability claim. The live 19-capability stack
  shows nominal root prediction, forecast error, robust world clearance, and
  selection as separate rows. See the
  [r112 CPU report](benchmarks/results/world-sdf-root-uncertainty-r112/WORLD_SDF_COMMAND_AUDIT.md)
  and [r112 live report](benchmarks/results/live-world-sdf-root-uncertainty-r112/LIVE_WORLD_SDF_AUDIT.md).
- An r113 versioned world-scene snapshot gate. The immutable SDF now carries
  scene epoch, source time, validity interval, and derived age. Expected epoch,
  future/not-yet-valid/expired windows, maximum age, and complete command-horizon
  coverage fail closed before primary or brake world evidence can be admitted.
  A seven-case policy-/physics-free differential retains stable reasons, exact
  replay, fixed arrays, zero timed allocations, and latency. The live
  20-capability stack places scene freshness between local world viability and
  root/world trajectory evidence. Scene epoch remains separate from program
  version and map/odom re-anchoring. See the
  [r113 CPU report](benchmarks/results/world-scene-epoch-r113/WORLD_SCENE_EPOCH_AUDIT.md)
  and [r113 live report](benchmarks/results/live-world-scene-epoch-r113/LIVE_WORLD_SDF_AUDIT.md).
- An r114 observed-versus-commanded authority gate. Observed joint q/v are
  mapped through the exact actuation transform and compared with the commanded
  splice. Position and velocity retain separate limiting actuators and signed
  braking/reject headroom. Warning-level mismatch selects only the validated
  brake with zero feed-forward effort; hard mismatch rejects both plans. The
  five-case policy-/physics-free differential runs 10,000 exact allocation-free
  transactions. The live 21-capability stack shows 54 nominal and six braking
  frames during the 80 mm disturbance. This detects mismatch without claiming
  a cause or plant model. See the
  [r114 CPU report](benchmarks/results/command-tracking-authority-r114/COMMAND_TRACKING_AUTHORITY_AUDIT.md)
  and [r114 live report](benchmarks/results/live-command-tracking-r114/LIVE_WORLD_SDF_AUDIT.md).
- An r115 robot-observation timing gate. Each dynamic transaction carries the
  producer timestamp, caller-mapped monotonic control timestamp, stable source
  ID and sequence, and synchronization uncertainty; Rust reads no clock.
  Signed age/synchronization headroom and independent future, stale, and
  uncertain flags fail closed for both Primary and brake. Seven cases run
  14,000 exact allocation-free transactions at `6.652/10.730/80.883 µs`
  p50/p99/max. The live 22-capability stream retains source `0xb015`, sequence,
  zero-age in-process evidence with `10/2 ms` headroom, 54 Primary plus six
  tracking brakes, and a 64.602 mm robust world certificate. See the
  [r115 CPU report](benchmarks/results/robot-observation-authority-r115/ROBOT_OBSERVATION_AUTHORITY_AUDIT.md)
  and [r115 live report](benchmarks/results/live-observation-authority-r115/LIVE_WORLD_SDF_AUDIT.md).
- An r116 fixed-capacity canonical observation history. Rust preallocates every
  state slot, rejects malformed or unsorted batches without partial mutation,
  and resolves equal mapped timestamps by lowest stable source ID then highest
  sequence independent of chunking. Queries return exact, shortest-manifold
  interpolated, bounded constant-velocity predicted, or held state with source
  provenance, age/synchronization headroom, and hard-constraint eligibility.
  Root pose and world-expressed root twist are retained with joint q/v; local
  cubic Hermite interpolation operates on continuous-joint and SO(3) tangents.
  Held state is visible but never hard-eligible. The policy-/physics-free audit
  passes epoch, future, stale, uncertain, invalid-state, wraparound, gap, and
  too-old gates across 2,000 exact replays with zero timed allocation. Four-row
  ingest is `0.180/0.231 µs` p50/p99 and query is `0.090/0.140 µs`. See the
  [r116 report](benchmarks/results/robot-observation-history-r116/ROBOT_OBSERVATION_HISTORY_AUDIT.md).
- An r117 live canonical-state boundary. The toy producer may still be guided
  or integrated for demonstration, but every floating-WBC model, contact,
  task, joint-limit, and command-admission query consumes only state emitted by
  the 64-slot history. WebSocket evidence exposes ring occupancy, ingest
  dispositions, reconstruction interval/source/sequence, provenance, age and
  synchronization headroom, and hard eligibility. A 60-frame 80 mm pull used
  60 exact hard-eligible reconstructions with 60/0/0 accepted/ignored/rejected
  stream samples, 54 Primary plus six brakes, and 64.602 mm minimum continuous
  world clearance. See the
  [r117 live report](benchmarks/results/live-observation-history-r117/LIVE_WORLD_SDF_AUDIT.md).
- An r118–r121 live delayed-observation corpus through the real WebSocket/Rust
  WBC boundary. Browser controls select exact 5 ms delivery, local cubic
  interpolation at 2.5 ms lookback, a 10 ms producer whose fourth command query
  is a 5 ms prediction, or a paused stale producer. The r121 audit retains four
  command queries per streamed frame: 48 exact, 48 interpolated, and 24 exact +
  24 predicted across twelve frames/mode, with zero held hard rows. The visible
  reconstruction, error object, observation stamp, WBC, Primary/brake geometry,
  and selection now share one query timestamp. Prediction remains hard-eligible
  and selected the independently certified brake on 3/12 frames. Stale input
  emits a typed `observation_withheld` event at 10 ms source age and recovers
  exact delivery in-session. See the
  [r121 report](benchmarks/results/live-observation-transport-r121/LIVE_OBSERVATION_TRANSPORT_AUDIT.md).
- An r119 typed reconstruction-error envelope through the actual Rust WBC.
  Exposure plus caller-authored growth produces separate joint q/v, root,
  represented-point, and CoM bounds. Joint stopping intersects every
  `q ± error, v ± error` corner; finite support loses one CoM radius,
  self-collision loses two point radii, and world collision loses one. A
  policy-/physics-free 20-frame/mode WebSocket audit proves exact raw=robust
  identity and monotone interpolation→prediction erosion while preserving
  stale withholding and in-session recovery. See the
  [r119 report](benchmarks/results/live-observation-uncertainty-r119/LIVE_OBSERVATION_UNCERTAINTY_AUDIT.md).
- An r120 native exposure sweep over the exact same error-growth and robust-WBC
  APIs. At the production 5 ms horizon, a near-limit joint loses 11.3474 rad/s²
  of upper stopping authority and a tight 10 mm self-collision margin loses
  0.525 mm while required outward acceleration rises 3.0000→3.0525 m/s². The
  bound costs 53.12 ns/call; alternating nominal/robust solves add
  +0.030/+0.040 µs at p50/p99 in the retained run. Across 200,000 bound calls
  and 5,000 paired solves per exposure, allocator calls/bytes remain zero and
  generalized accelerations replay bitwise. See the
  [r120 report](benchmarks/results/observation-uncertainty-exposure-r120/OBSERVATION_UNCERTAINTY_EXPOSURE_AUDIT.md).
- An r121 command-trajectory uncertainty boundary. The same fixed-size typed
  error object used by the WBC is passed into allocation-stable Primary and
  brake admission. Sampled and continuous self-collision lose two body-local
  represented-point radii. World admission adds observation root translation
  and rotation radii to the root forecast, then loses one field-Lipschitz-scaled
  local point radius. A policy-/physics-free 20-frame/mode live audit proves
  exact/interpolated/predicted self losses of 0/0.25625/0.525 mm and world
  losses of 0/0.128125/0.2625 mm for both candidates and both certificate types;
  transport query time equals the command observation stamp on every frame.
  See the
  [r121 report](benchmarks/results/live-command-observation-uncertainty-r121/LIVE_COMMAND_OBSERVATION_UNCERTAINTY_AUDIT.md).

## Run it

The workspace uses stable Rust.

```bash
cargo test --workspace --lib --bins
./scripts/run-reference-comparison.sh --ticks 5000 --warmup 250
./scripts/run_reference_trace_report.sh
./scripts/run-upkie-controller-comparison.sh --ticks 100000
./scripts/run-g1-oracle-wbc-admission.sh
./scripts/run-g1-coupled-actuation-oracle.sh
./scripts/run-actuator-resource-authority.sh
./scripts/run-dense-multiply-row-slice-audit.sh
./scripts/run_live_authority_stream_report.sh
./scripts/run_live_observation_transport_report.sh --url ws://127.0.0.1:8818/ws
./scripts/run_live_observation_uncertainty_report.sh --url ws://127.0.0.1:8819/ws
./scripts/run_observation_uncertainty_exposure_report.sh --repeats 5000
./scripts/run_world_sdf_barrier_report.sh
./scripts/run_world_sdf_command_report.sh
./scripts/run_world_scene_epoch_report.sh
./scripts/run_command_tracking_authority_report.sh
./scripts/run_robot_observation_authority_report.sh
./scripts/run_robot_observation_history_report.sh
./scripts/run_live_world_sdf_report.sh
./scripts/run_auto_rigid_patch_basis_report.sh
./scripts/run_live_flat_foot_authority_report.sh
./scripts/run_cpu_mirror_solve_report.sh
./scripts/run_cuda_state_input_report.sh
./scripts/run_cuda_fk_com_report.sh
./scripts/run_cuda_kinematics_report.sh
./scripts/run_cuda_dynamics_report.sh
./scripts/run_cuda_point_queries_report.sh
./scripts/run_cuda_emission_report.sh
./scripts/run_cuda_solve_report.sh
./scripts/run_dynamic_advance_report.sh
./scripts/run_dynamic_admission_fault_report.sh
./scripts/run_dynamic_collision_fault_report.sh
./scripts/run_dynamic_continuous_clearance_report.sh
./scripts/run_dynamic_adaptive_clearance_report.sh
./scripts/run_tight_primitive_avoidance_report.sh
./scripts/run_live_command_authority_report.sh --url ws://127.0.0.1:8798/ws --expected-support measured
cargo run --release -p bonesaw-tools --bin bonesaw-server -- models/upkie/upkie.urdf
./scripts/manage-live-upkie.sh start
./scripts/manage-live-upkie.sh url
./scripts/manage-live-upkie.sh stop
BONESAW_BIND=0.0.0.0:8797 BONESAW_LIVE_GUIDED=1 \
  cargo run --release -p bonesaw-tools --bin bonesaw-server -- models/toy_humanoid.urdf
cargo run --release -p bonesaw-tools --bin bonesaw-compile -- \
  --model models/upkie/upkie.urdf --output /tmp/upkie.motion \
  --collision-avoidance
./scripts/run-pinocchio-oracle.sh --model models/upkie/upkie.urdf --samples 50
```

The full comparison script creates an isolated temporary Python environment,
rebuilds the current Rust extension, generates shared target corpora, evaluates
Bonesaw and PlaCo in separate worker processes, and executes the pinned Upkie
C++ wheel controller through its own native worker. Its consolidated report
is [benchmarks/results/reference-latest/REFERENCE_COMPARISON.md](benchmarks/results/reference-latest/REFERENCE_COMPARISON.md);
the direct Upkie report is
[benchmarks/results/reference-latest/UPKIE_CONTROLLER_COMPARISON.md](benchmarks/results/reference-latest/UPKIE_CONTROLLER_COMPARISON.md).
Raw `.npz` traces and machine-readable JSON are retained beside them.
The fresh r89 run and its responsive execution-over-time view are retained at
[benchmarks/results/reference-r89/REFERENCE_COMPARISON.md](benchmarks/results/reference-r89/REFERENCE_COMPARISON.md)
and served as `web/REFERENCE_COMPARISON_R89.html`; the latter is regenerated
from the raw arrays with `scripts/run_reference_trace_report.sh`.

The server listens on port 8787 on local interfaces by default. Open
[http://127.0.0.1:8787](http://127.0.0.1:8787) on the host, or use its LAN IP
from another device on the same trusted network. Set `BONESAW_BIND` to narrow
the listener. The hosted interactive default is the pinned Upkie model. Drag
empty canvas to orbit the perspective camera, use the wheel to zoom, and drag
a joint to apply an intent in the camera-facing plane through that joint. The
browser sends the resulting world-space target; the server streams at 50 Hz
while the floating WBC runs one query on each 20 ms controller tick, then returns the
complete canonical frame state and tick telemetry. The display coalesces only
visual state at the animation-frame boundary, refreshes the large telemetry
surface at 10 Hz, and sends at most one drag command per display frame; server
tick IDs continue to expose the controller cadence.
Append `?perf=1` to expose retained presentation distributions for frame
interval, full draw, geometry draw, snapshot arrival, command acknowledgement,
telemetry work, coalescing, and pointer-to-camera latency. A lost connection
greys and ghosts the last mesh and disables robot controls. The torso handle is
`TORSO · BASE`, an ordinary three-axis Cartesian target like the other green
handles. Planar leg IK preserves the wheel anchors. If an authored path would
put a compiled collision primitive below z=0 or leave the IK domain, the preview
searches back to the nearest safe point and exposes `LIMIT` plus the unapplied
distance; it does not silently reset or claim plant response.

Ctrl+drag starts a physical wrench on the nearest rendered triangle of any
robot body. The exact surface hit becomes the application point, the
camera-plane drag becomes a force capped at the server-advertised 8 N, and the
140 ms lease is refreshed at 20 Hz until release. Shift+drag always pans, so it
no longer shares the wrench gesture. The explicit PUSH tool remains available
for touch devices. That mode lazily creates a Python-owned MuJoCo plant with a
physical z=0 plane: MuJoCo advances at 250 Hz, persistent Rust WBC runs at
50 Hz, each torque command is held for five 4 ms contact steps, and state
streams at 50 Hz. Contact points, normals, normal force, ground-contact count,
penetration, sim time, solver iterations, backend/version, wrench force and
moment are rendered separately from the diagnostic TARGET state.

The live plant lifecycle is explicit: `plant_pause` releases the wrench and
freezes MuJoCo while keeping a heartbeat, `plant_resume` restarts from the
measured frozen state without resurrecting the wrench, and `plant_reset`
rebuilds standing state, clears the wrench, increments `reset_epoch`, and keeps
the paused state when reset was requested while paused. The WBC reads measured
MuJoCo qpos/qvel/root at 50 Hz; it does not integrate a private plant proxy.
The full intent/wrench/feedback contract is in
[`docs/LIVE_PLANT_INTENT_WRENCH_CONTRACT.md`](docs/LIVE_PLANT_INTENT_WRENCH_CONTRACT.md)
and the hosted [r235 lifecycle report](/LIVE_MUJOCO_FEEDBACK_LIFECYCLE_R235.html).

For public inspection, `scripts/manage-live-upkie.sh` runs exactly one server,
`bonesaw-public`, on port 8777 and one `bonesaw-tunnel`; it also stops the old
`bonesaw-local` unit on every replacement. `start` clears only those exact
project units and validated absolute-path orphans, refuses an unrelated 8777
listener, scopes URL discovery to the current tunnel invocation, and verifies
the Upkie model through public HTTP and WebSocket before printing the URL.
`stop` is idempotent, and the default eight-hour runtime limit cleans both
active units up automatically. It never stops the unrelated machine Cloudflare
tunnel. Set `BONESAW_LIVE_TTL` to a systemd duration for a shorter session.
The dependency-free acceptance sequence is:

```bash
python3 python/evals/live_editor_smoke.py \
  --url http://127.0.0.1:8777 --exercise-drag

python3 python/evals/live_editor_limit_recovery.py \
  --url http://127.0.0.1:8777 --cycles 100
```
With `BONESAW_LIVE_GUIDED=1`, commanded poses are state-local feasibility queries and
the adapter does not claim policy, synthetic physics, or integrated body response.
Raw floating ticks include clipped priority levels and fourteen
stable, allocation-free task residual slots (nine semantic base slots, one
arbitrary-frame angular slot, and four Cartesian point slots), so `DEGRADED` can be inspected
without treating task compromise as hard infeasibility. The live Authority
stack separately shows hard-row residual, finite-patch support margin when
defined, nearest joint-limit headroom, peak solved torque/URDF effort use, the
fixed 20 ms live WBC deadline, per-priority residual/clipping, and warning persistence.
Absolute warning/critical values come from the versioned threshold profile in
the WebSocket Hello record, which also declares actuator names and whether each
has a calibrated resource model; threshold-free physical evidence remains in core.
Thermal/reliability remains explicitly `UNMODELED`; rolling contact reports
finite-polygon support as `N/A` rather than fabricating a margin. Static review assets
are served with `Cache-Control: no-store` so architecture revisions appear
immediately. The development adapter has no authentication. A Cloudflare Quick
Tunnel is suitable only for temporary non-sensitive demos; do not attach
private telemetry or hardware authority without access control.

The header’s **Architecture review** control opens the current review surface:
runtime dataflow, crate boundaries, frozen and provisional decisions, a clearly
labeled example authority stack, measured evidence, known gaps, and questions
requesting feedback. The current measured r90 example uses the live flat-foot
query and shows hard dynamics/contact, finite support, joint stopping, task
residuals, actuator effort, solver work, backend conformance, and unavailable
thermal realization as independent rows. It explicitly shows maximum-query and
four-query frame-batch clocks as separate resources and never produces an
aggregate score. The same example stack now includes adjacent r102 sampled
geometry and continuous-clearance rows: the former can remain green while the
latter turns red when no collision was observed but between-sample clearance
was not proved. R89 adds the
fresh reference audit as a separate evidence card rather than treating a
benchmark ratio as another authority layer. R95 shows the
StateInput+FK/CoM+frame/CoM-Jacobian+M/h/Ag+point-position/J/Jdot-v backend as separate source/ABI, CPU
mirror, compiler, runtime, and device-conformance authority rows and leaves every
unavailable device gate visible. Its versioned source is
`web/architecture.json`. Significant architectural changes must update that
manifest’s revision, date, decisions, evidence, and gaps in the same change so
the hosted review never silently drifts from the implementation.

Machine-readable evaluation output:

```bash
cargo run --release -p bonesaw-tools --bin bonesaw-eval -- --ticks 1000 --json
```

This Rust executable is the low-level correctness/allocation sentinel. New
behavioral experiments and cross-implementation analysis belong in
`python/evals`, using the native batch boundary rather than Python per-tick
calls.

The fixed-layout CPU-mirror/device-admission report is reproduced with:

```bash
./scripts/run-cuda-batch-abi-audit.sh
```

The frame/CoM Jacobian stage report is reproduced with:

```bash
./scripts/run-cuda-jacobian-abi-audit.sh
```

The floating model-product stage report is reproduced with:

```bash
./scripts/run-cuda-dynamics-abi-audit.sh
```

The compiler-resolved point-query report is reproduced with:

```bash
./scripts/run-cuda-point-query-abi-audit.sh
```

The fixed point-task/contact-lock emission report, including the concrete
example authority stack and non-gating shared-qdd conflict diagnostic, is
reproduced with:

```bash
./scripts/run-cuda-emission-abi-audit.sh
```

## Robot models

The included `models/toy_humanoid.urdf` is deliberately small and
primitive-based so the controller and browser demo stay inspectable.

The external hardware reference is
[Upkie](https://github.com/tasts-robots/upkie_description), an Apache-2.0
open-source wheeled biped whose URDF contains joint limits, wheel joints,
collisions, masses, CoMs, and inertia tensors. The pinned model and license are
in `models/upkie`; reproduce them explicitly with:

```bash
chmod +x scripts/fetch-upkie-reference.sh
./scripts/fetch-upkie-reference.sh
cargo run --release -p bonesaw-tools --bin bonesaw-eval -- \
  --model models/upkie/upkie.urdf --inspect
```

The Bonesaw importer accepts the Upkie model. Fixed virtual-link masses are
folded into their dynamics parent, unbounded wheel joints are canonicalized,
and authored visual geometry/materials remain body-local canonical model data.
`MESHES.sha256` verifies the URDF, license, and ten fetched STL assets together.
Keeping the fetch explicit prevents an upstream hardware update from silently
changing a compiled program.

`bonesaw-compile` writes a deterministic, checksummed canonical archive and
reads it back before reporting success. Archive loading verifies the schema,
payload checksum, content fingerprint, timing, and canonical ID invariants.
`--collision-avoidance` opts a model into the default conservative proxy
policy and fingerprints that policy into the archive; hardware profiles should
author model-specific margins and pair filters before deployment.

## Evaluation intent

The implemented scenarios are small enough for every change and target distinct
failure modes:

| Evaluation | What it exposes | First acceptance target |
|---|---|---|
| Pinocchio differential | independent FK/Jacobian/dynamics conventions | reference thresholds in `docs/PINOCCHIO_ORACLE.md` |
| PlaCo reference corpus | independent WBC behavior, latency, jitter, memory, and tracking | no 20 ms misses; retain raw distributions |
| Moving bimanual reach | FK/Jacobian conventions and task response | bounded RMS error, no limit violation |
| Bimanual priority conflict | singularity and hierarchy leakage | viability level preserved; typed degradation |
| Walking retarget | pinned CMU 37/01 foot/hand tracking across three cadences | foot/hand RMS `≤ 5/3 cm`, stance/swing foot RMS `≤ 6 cm`, no failures |
| Floating walking retarget | moving-root CMU 37/01 with measured touchdown anchors and stance transitions | no contingency/rejection; root/stance/swing/hand RMS `≤ 5/2/8/5 cm`; hard residuals `<1e-8` |
| G1 synthetic toe-step | deterministic short liftoff, sole-orientation landing, touchdown admission, and relock | functional gates pass; touchdown `≤8` ticks; report the independent 5 ms p99 gate |
| Replay determinism | hidden state and unstable ordering | bitwise-equal velocity commands |
| Frame-query throughput | fast historical/editor queries | `> 100k` current-state queries/s |
| Historical atlas throughput | reconstruction + FK + rooted atlas evaluation | `> 20k` queries/s |
| Dynamics identities | mass/inverse/forward convention drift | symmetry, positive mass matrix, `< 1e-10` round trip |
| Unified dynamic WBC | dynamics/contact/friction/torque conformance | residuals `< 1e-8`; zero infeasible; Upkie p99 `< 5 ms`; retain DOF scaling curve |
| G1 oracle-state admission | morphology and finite-support inverse-dynamics admission without policy, integration, or physics | 600/600 solve; 5 mm hard CoP margin; declared projection gates; residuals `< 1e-8`; zero allocations; exact repeat |
| G1 coupled-actuator admission | exact non-diagonal actuator polytope response at the same independent oracle states | 600/600 solve; pair materially reaches its bound; power duality `<1e-10 W`; zero allocations; exact repeat |
| G1 integrated finite-support toe-step | ordinary contact-transition path and streamed support-margin diagnostic | 5 mm margin on every loaded tick; zero contingency/rejection; functional gates green; retain independent 5 ms p99 gate |
| Collision differential | distance/Jacobian sign and convention drift | finite-difference max error `< 1e-6` |
| Hard-constraint conformance | active row and contradiction semantics | zero violation/residual; typed infeasible |
| Compiled signal graph | derivative propagation, state isolation, archive drift, and hidden allocation | bitwise repeat; zero allocations |
| Fixed task/contact emission | stable provenance, target-jet algebra, exact inactive masks, and simultaneous-row conflict | NumPy composition pass; exact repeat/reindex/isolation; zero allocations; feasibility reported separately |
| Allocation sentinel | accidental work-path heap traffic | zero allocations after construction |
| CPU tick latency | deployability at 50 Hz | release p99 `< 5 ms` on target hardware |

The pinned CMU walking corpus is now data-backed, checksum-verified, morphology
scaled, and evaluated at 0.75×, 1.0×, and 1.25× cadence. Its fixed-pelvis
grounding invariant rejects flight phases before either implementation runs.
The current 5,000-step result is intentionally red: Bonesaw records `5.549 cm`
foot and `1.411 cm` hand RMS, missing the `5 cm` foot gate; PlaCo records
`3.701 cm` foot RMS but misses the `3 cm` swing-clearance gate at `3.301 cm`.
Both complete without contingency or rejected ticks. Raw traces and the full
latency, jitter, memory, CPU, GC, cadence, and transition analysis are in
[REFERENCE_COMPARISON.md](benchmarks/results/reference-latest/REFERENCE_COMPARISON.md).

The current official-G1 moving-root/contact-aware Rust WBC corpus is published
in
[FLOATING_WALK_CORPUS.md](benchmarks/results/floating-g1-liftoff-latest/FLOATING_WALK_CORPUS.md).
Across 260 strict 5 ms ticks, the moving liftoff proof passes every behavioral
and timing gate: `0.748 cm` root RMS, effectively zero stance-foot drift,
`0.173 cm` swing-foot RMS, `2.256°` maximum root rotation, and zero fallback,
release, or infeasible ticks. Dynamics/contact residuals remain
`1.22e-9`/`5.16e-11` under the unchanged `1e-8` contract. Revision r30's
cached-energy Jacobi kernel records `2.320/3.536/3.904 ms` p50/p95/p99. The
longer
full-transfer stress still
saturates the `8 rad/s` joint-speed envelope before a clean contact transfer.
That complete red trace is retained in
[the 600-tick transfer report](benchmarks/results/floating-g1-transfer-latest/FLOATING_WALK_CORPUS.md).
These are performance and controller/reference-design failures, not hidden by
relaxed feasibility tolerances.

Revision r24 adds a centroidal angular-momentum-rate task over external contact
moments and a reproducible five-case support-policy matrix. The strongest
combined support-preview/centroidal case extends the nominal window from 330 to
532 ticks and removes infeasible ticks, but a 104-tick normal-only touchdown and
large tracking errors keep every full-transfer case red. The canonical policy
therefore remains unchanged. Full raw traces and the comparison table are in
[G1_SUPPORT_POLICY_SWEEP.md](benchmarks/results/g1-support-policy-sweep/G1_SUPPORT_POLICY_SWEEP.md).

Revision r26 separates a controller acceptance proof from that deliberately hard
CMU transfer. The deterministic official-G1 toe-step uses a predeclared 1 cm
forward/1 cm clearance reference, a 40-tick swing, and the same strict solver
path. Rust-owned `Swing → Precontact → TouchdownNormal → Locked` telemetry is
streamed per target, pre-contact translation targets the sole centre, and a
separate frame-angular task prevents hidden foot roll. The case passes every
functional check: zero fallback/release/infeasible ticks, root/stance/swing RMS
`4.298/0.095/0.639 cm`, `1.585°` maximum root rotation, and a three-tick
touchdown at `0.034 mm/s` whole-patch tangential speed. The combined gate stays
red because the regenerated release p99 is `21.661 ms` against the unchanged
5 ms deadline.

Revision r28 caches immutable feasibility-row norms and skips exactly zero
projection updates; all 37 shared non-timing arrays remain byte-for-byte equal
across cold/optimized builds. Five pinned hardware-counter runs show 6.729%
fewer instructions, 12.714% fewer cycles, and 11.341% less task-clock time;
per-tick p99 remains noisy and explicitly red. The full A/B is in
[the r28 feasibility-kernel report](benchmarks/results/g1-synthetic-step-r28-ab/FEASIBILITY_KERNEL_AB.md).
Revision r29 bounds the ordinary Dykstra prefix at eight sweeps and then uses a
preallocated Euclidean primal/dual active set, capped at 64 iterations. The
toe-step stays functionally green and its five-run p99 median falls from
`24.162 ms` to `6.349 ms`; median whole-call thread CPU falls 41.0%. The
moving-liftoff p99 is now `4.122 ms` and combined green. A same-revision
600-tick CMU control falls from `177.348 ms` p99 and 126 primal-infeasible ticks
to `13.547 ms` and zero primal-infeasible ticks, but its tracking/contact
behavior remains explicitly red. Exact methodology, state deltas, jitter,
memory, CPU, rejected thresholds, and execution-over-time links are in
[the r29 hybrid feasibility report](benchmarks/results/g1-feasibility-r29/HYBRID_FEASIBILITY_AB.md).
Revision r30 caches one-sided-Jacobi column energies inside each sweep and
recomputes them in canonical summation order at every sweep boundary. An
isolated uncached control records `8.449 ms` p99 and `617.2 ms` whole-call
thread CPU; five cached repeats record a `4.364 ms` median p99 and `411.1 ms`
median thread CPU, with identical non-timing fingerprints. Four of five runs
meet the 5 ms gate; the retained scheduler-tailed run reaches `5.926 ms` p99.
The latest toe-step is combined green at `4.282 ms` p99 with
root/stance/swing RMS `3.567/0.002/0.639 cm`. The active-set accelerator now
detects repeated working sets and, on failure, restores the exact Dykstra state
and spends the remaining bounded projection budget instead of issuing a false
infeasibility verdict. Because within-sweep arithmetic changes later active
boundaries, r30 is reaccepted by unchanged physical and hard-row gates rather
than described as trace parity. The complete A/B is in
[the r30 Jacobi-energy report](benchmarks/results/g1-jacobi-r30/JACOBI_ENERGY_CACHE_AB.md).
The exact report and raw arrays are in
[the G1 synthetic-step result](benchmarks/results/g1-synthetic-step-latest/FLOATING_WALK_CORPUS.md)
and reproduce with `scripts/run-g1-synthetic-step.sh`. The current typed-phase
support-preview CMU stress remains explicitly red despite zero fallback,
release, infeasible, or failed ticks and a green `4.808 ms` p99: its
root/stance/swing RMS is still `23.670/26.328/28.588 cm`. Its complete trace is
in [the r30 support-preview transfer result](benchmarks/results/g1-cmu-transfer-r30-final/FLOATING_WALK_CORPUS.md).
The default full-transfer trace is retained separately and remains much worse.

Revision r31 makes planned contact establishment depend on measured sole state
rather than the authored phase bit alone. The landing material point must be
within `2.5 cm` of its stored anchor, below `0.20 m/s` tangential speed across
the whole patch, and below `0.20 m/s` normal speed. During a handoff, the old
support remains locked until the replacement locks. The accepted toe-step is
still combined green at `4.040 ms` p99 with zero delayed-admission ticks. The
CMU stress now truthfully denies the impossible touchdown for 172 ticks and
keeps the right foot locked, instead of manufacturing a mid-air contact and
then releasing the only support. Its tracking remains red, but the contact
state is physically honest and its `4.763 ms` p99 still passes. The full A/B,
memory, CPU, phase, residual, and execution evidence is in
[the r31 measured-touchdown report](benchmarks/results/g1-touchdown-admission-r31/MEASURED_TOUCHDOWN_ADMISSION.md).

Revision r32 adds allocation-free actual-CoM observation to the fixed-shape
Rust-to-NumPy trace and uses it to locate the first sustained-transfer defect.
The support-preview experiment moves its CoM target `23.7 cm` laterally over
40 ticks and produces a `57.8 m/s²` finite-difference acceleration edge: it is
treating the support centre like a CoM command when it is actually closer to a
ZMP/CoP policy input. On an otherwise identical 250-tick prefix, disabling only
that CoM task reduces root/CoM/swing RMS from `2.323/3.550/8.921 cm` to
`0.400/1.160/0.360 cm`; that control later loses balance, so disabling CoM is
causal evidence rather than the proposed controller. Default toe-step and
moving-liftoff cases remain combined green at `4.125 ms` and `3.852 ms` p99.
The 600-tick CMU stress stays functionally red but CPU-green at `4.795 ms` p99,
now with measured CoM RMS `26.808 cm`. The next canonical walking experiment is
measured CoM velocity/capture-point feedback with a virtual ZMP clipped to the
measured support polygon. See
[the r32 CoM-reference causal report](benchmarks/results/g1-com-reference-r32/COM_REFERENCE_CAUSAL_AB.md).

Revision r33 implements that experiment in `bonesaw-core`: measured DCM,
fixed-capacity convex support-hull construction, metric margin erosion,
virtual-ZMP projection, and horizontal LIPM CoM acceleration are allocation
free. The Python corpus adds a backward DCM reference built from an admissible
support-centre ZMP sequence and archives measured/target DCM, raw/clipped ZMP,
CoM velocity/command, natural frequency, support vertices, and height-floor
telemetry. On the first 250 ticks sliced from the full trace, swing RMS improves
from the r32 static servo's `9.613 cm` to `1.821 cm` with `0.966 cm` DCM RMS.
The full transfer is still rejected: touchdown is denied for 172 ticks, the
right support enters 173 normal-fallback ticks, tracking diverges, and p99
reaches `48.250 ms`. The policy therefore remains experimental. Unchanged
toe-step and moving-liftoff regressions stay combined green at `4.020 ms` and
`3.844 ms` p99. See
[the r33 DCM/ZMP report](benchmarks/results/g1-dcm-zmp-r33/DCM_ZMP_EXPERIMENT.md).

Revision r34 localizes the remaining loss to task authority rather than an
early phase edge. Eval-only 5/10/20-tick liftoff holds all make the transfer
worse. Moving posture authority upward or activating a smooth joint-velocity
envelope postpones the first contingency from tick `427` to as late as `490`
and cuts DCM RMS from `43.355 cm` to `12.219 cm`, but late root attitude and
tracking still diverge. `bonesaw-core` now includes the allocation-free smooth
velocity-envelope law and `bonesaw-py` assembles its active subset in
preallocated Rust storage; it remains disabled by default. See
[the r34 task-authority report](benchmarks/results/g1-task-authority-r34/TASK_AUTHORITY_EXPERIMENT.md).

Revision r35 moves authority scheduling into the CPU core: measured contact
phase is typed, task-scale transitions are bounded, and the Rust batch path
streams applied scale plus active joint coordinates. Hard release and symmetric
ramping are rejected. Immediate engagement with a 200-tick release produces the
best full-run DCM RMS so far (`10.831 cm`) and preserves `33.777 cm` swing RMS,
but root attitude still becomes unrecoverable during the long precontact state.
Static root height/attitude reweighting also fails. The next schedule must use
DCM margin, root-attitude residual, precontact reachability, and joint
utilization—not phase alone. See
[the r35 phase-authority report](benchmarks/results/g1-phase-authority-r35/PHASE_AUTHORITY_EXPERIMENT.md).

Revision r36 closes that schedule over measured feedback. The CPU core now
reports signed DCM margin to the exact eroded support polygon and blends capture
need with root-attitude error and precontact age into a continuous authority
target. A 150-tick release postpones the first contingency to tick `472` with
zero rejected solves; a 200-tick release achieves `28.436 cm` stance RMS. The
trace also proves why reweighting alone cannot finish the transfer: DCM margin
reaches `-85.855 cm` and is inside support on only `48.33%` of ticks. The next
slice must move/retime the landing while it remains reachable. See
[the r36 feedback-authority report](benchmarks/results/g1-feedback-authority-r36/FEEDBACK_AUTHORITY_EXPERIMENT.md).

Revision r37 implements that spatial policy in Rust. A pure capture-landing
primitive moves the latched sole-center anchor toward measured DCM while
simultaneously enforcing an authored-offset disk, future-root leg reach, fixed
landing height, and per-tick slew. A commitment horizon freezes the anchor and
the existing measured touchdown gate retimes contact. The stable 8 cm case
extends the clean prefix to tick `482`, cuts DCM RMS to `18.131 cm`, and has
zero rejected solves. New gate telemetry proves why it still does not land:
minimum position error is `19.857 cm` and whole-patch tangential speed never
falls below `0.678 m/s`, versus `2.5 cm` / `0.20 m/s` limits. The planner stays
disabled by default; the next slice must retime the complete root/CoM/swing
trajectory rather than manufacture contact. See
[the r37 capture-landing report](benchmarks/results/g1-capture-landing-r37/CAPTURE_LANDING_EXPERIMENT.md).

Revision r38 audits the reference generator before adding another controller
mechanism. The backward DCM oracle now uses the declared fixed receding horizon,
and lateral root registration is calibrated from the pinned canonical CMU cycle
rather than the caller's output length. Exact 600/800-prefix regressions cover
root, endpoint, contact, cadence, phase, and bounded future influence. On the
corrected corpus, a 200-tick horizon is behaviorally bitwise-repeatable across
three runs and retains a clean prefix through tick `480` with zero rejected
solves. It improves minimum landing distance to `9.73 cm`, but tangential speed
still bottoms out at `1.626 m/s`; the unchanged `2.5 cm / 0.20 m/s` physical
gate therefore rejects every touchdown sample. See
[the r38 bounded-reference audit](benchmarks/results/g1-bounded-preview-r38/BOUNDED_PREVIEW_CAUSAL_REPORT.md)
and [the refreshed reference comparison](benchmarks/results/reference-r38/REFERENCE_COMPARISON.md).

Revision r55 adds a report-only consolidation of that retained independent
reference suite with the current r54 four-step admission. It deliberately does
not retime old artifacts or publish cross-boundary speedup ratios. The report
includes the complete current G1 latency/jitter/deadline distribution, process
CPU and memory, morphology and WBC tracking, per-task nullspace residuals,
solver-work correlation, and 1 s execution windows, then places the PlaCo,
Pinocchio, and exact upstream Upkie results beside their precise comparability
limits. See the
[r55 CPU reference audit](benchmarks/results/cpu-reference-comparison-r55/CPU_REFERENCE_COMPARISON.md).

Revision r123 closes the remaining policy-free Upkie composition gap. The
preallocated PyO3 oracle trace now admits explicit per-target `RollingWheel`
descriptors, while the Rust solver still owns row emission and the complete
floating `[qdd, actuator torque, contact force]` hierarchy. A 256-state corpus
uses both real Upkie wheel-center frames and independently reconstructs mass,
nonlinear effects, Jacobians, and `Jdot·v` in Pinocchio. Returned commands close
the independent dynamics/contact equations at `1.97e-11` / `4.61e-12` L∞,
allocate zero bytes, and replay bitwise exactly across repeat, reverse order,
and chunking. The audit deliberately retains large lower-layer tracking error
and acceleration-cap saturation beside hard admission, plus friction/effort
sweeps, slip recovery, fault typing, CPU, memory, jitter, and the exact pinned
Upkie C++ controller-law parity evidence. See the
[r123 state-local Upkie WBC audit](benchmarks/results/upkie-state-local-wbc-r123/UPKIE_STATE_LOCAL_WBC_AUDIT.md).

Revision r124 turns that static descriptor into a long explicit contact-mode
trace without adding hidden contact state. The seeded 20,000-state corpus
exercises `LockedPoint`, `NormalPoint`, `RollingPoint`, and `RollingWheel`,
including double/single/mixed/no support, 288 mode edges, and bounded no-slip
repair. Pinocchio independently selects the correct contact rows per state and
reconstructs dynamics from the returned packed forces. Every state solves or
uses typed lower-layer slack; dynamics/contact close at `3.02e-11` /
`6.13e-12`, repeat/reverse/four-chunk results are bitwise exact, and the Rust
tick records zero allocations and zero 0.5 ms misses. The API now validates
the complete input call before output mutation, so nonfinite state, invalid
mode/activity, and degenerate wheel coefficients are typed and atomic. See the
[r124 Upkie contact-transition audit](benchmarks/results/upkie-contact-transition-r124/UPKIE_CONTACT_TRANSITION_AUDIT.md).

Revision r125 makes generalized-acceleration and actuator-effort availability
explicit per-state inputs to that same allocation-free Rust oracle. A 10,000-
state policy-/physics-free replay compares nominal, acceleration-derated,
effort-derated, and combined authority. The combined case contacts acceleration
and effort bounds in 6,201 and 1,368 queries (526 simultaneously) while all
queries remain `Solved` or `SolvedWithSlack` and independent Pinocchio
dynamics/contact residuals remain `6.11e-11` / `1.19e-11`. All-ones scales are
bitwise transparent; repeat, reverse, and four-chunk replays are exact; invalid
scales reject atomically; the measured Rust loop allocates zero bytes. These
scales are declared availability, not inferred thermal health or plant
realization. See the
[r125 Upkie resource-authority audit](benchmarks/results/upkie-resource-authority-r125/UPKIE_RESOURCE_AUTHORITY_AUDIT.md).

Revision r126 makes unreachable interactive targets recoverable without a
reset. The fixed-root Rust controller now uses bounded uniform velocity
backtracking to find the first complete position/velocity/acceleration/jerk/
collision-valid quintic, reserves one later braking horizon in its velocity
bounds, and fails closed to a represented-state hold if even the contingency
is invalid. The live protocol streams represented q/v, commanded velocity,
backtracking scale/work, active rows, and per-coordinate position/stopping
headroom and correlated command IDs; its read-only arbitrary-q FK query does
not mutate controller state. Cartesian frame drag and typed soft joint drag are
separate because a position-only endpoint cannot select both knee IK branches
through a straight leg. The Python end-to-end audit uses that explicit joint
path to drive all eight finite Upkie hip/knee lower/upper cases for 100
hold/retreat/double-release cycles. Every case runs 12–13 times with no reset
or hard-bound crossing, and motion away begins on the next 20 ms frame. This is
command-side evidence, not policy, physics, or plant realization. See the
[r126 live joint-limit recovery audit](benchmarks/results/live-editor-limit-recovery-r126/LIVE_EDITOR_LIMIT_RECOVERY.md).

The next corpus additions are broader randomized fault injection, long-run
replay, and dynamic contact-mode transitions. See
[docs/EVALUATION.md](docs/EVALUATION.md).

## Architecture and status

The core remains a library. It does not own a clock, ROS node, subscription,
worker, or global state. The browser server is an adapter and supplies explicit
tick times and controller state.

The controller has an explicit double-buffered state transition,
caller-owned scratch, reusable borrowed input, output buffer, immutable program
epoch, and deterministic commanded-segment splice. Fixed-capacity task and
constraint slots feed a flat, preallocated hierarchical-solver workspace.
`MotionProgram` archive schema 6 also carries a canonically compiled
scalar/vector/rotation signal graph. Its stable topology, inferred types, state
slots, outputs, and resolved point/CoM/orientation/direction-aim task operations participate in the program
fingerprint; runtime filter/spring memory remains explicit and
double-buffered. Signal-output stable IDs are resolved to fixed slots and
frames at compilation, and the primary controller evaluates them before task
emission. Rotation smoothing evolves on SO(3) and exposes world angular
velocity/acceleration jets; pose and inverse-dynamics task bindings remain next.
The native allocation sentinel measures zero calls and zero bytes inside the
controller transition for reach, priority conflict, walking retarget, and
collision-enabled operation. The Python boundary performs no per-tick Python
object creation and the long comparison records zero Python GC collections.
The hosted browser computes a contact-consistent planar reference with
allocation-free Rust damped Gauss-Newton IK and runs a floating WBC authority
query with rolling contacts. Its torso control is a Cartesian floating-base
target in XYZ; lowering it produces a squat only as a consequence of preserving
the two wheel anchors. The target remains authored even when the query is
infeasible. The controller retains Upkie's open-source
position/pitch PI structure but observes an articulation-aware virtual pitch
from axle-to-CoM geometry. The public 8777 deployment explicitly enables the
separately labelled guided preview; the older raw SE(3) self-integration path
remains experimental evidence and is not a plant. A 1 Hz
critically damped spring in the compiled
signal topology shapes discontinuous editor height targets. The 5,000-step
RollingWheel sentinel reaches the full 12 cm squat with `5.93e-6 m` maximum
constrained slip, 1.73 cm permitted wheel travel, zero infeasible steps, and
zero hot-loop allocations. Its latest isolated endurance p50/p99 are
`262.2/277.8 µs`.
Root attitude, filtered root translation, and CoM feedback execute
as three resolved floating task slots from the same four-node compiled policy
used by the native endurance eval. A Python-owned nine-case corpus covers
admissible initial ground velocities from -5 to +5 cm/s; all train and held-out
cases pass twice with exact non-timing metrics. The strengthened r127 WebSocket
test reaches an authored three-axis base target with `5.55e-17 m` final
residual, continues streaming through red WBC evidence, releases cleanly, and
separately retains joint dragging. The same raw controller previously held a
browser-driven squat for 18 seconds without solver or transport errors; that
historical result is not plant evidence. The
viewport no longer depends on an abstract stick figure: the WebSocket Hello
record carries canonical collision and visual geometry in body-local
coordinates, and the browser composes it with the same streamed rigid
transforms used by frame queries. BODY geometry is enabled by default while the
RIG overlay can be toggled independently. The browser loads and shades the ten
checksum-pinned upstream Upkie STL assets once and reuses their transformed
topology on subsequent frames.
R128 admits the first closed-loop CPU plant boundary while preserving the
policy-/physics-free solver gates. MuJoCo receives an explicit floating-root
Upkie before URDF parsing, static-body fusion is disabled so its CoM and
sagittal mass matrix match Pinocchio, and no unmodeled armature or damping is
invented. A persistent Rust `UpkieBalanceSession` owns reference PI state,
signs, clamps, balanced-standing projection, and axle-to-CoM virtual pitch;
Python owns soft contact/integration, disturbance timing, and scoring. The
4.5 s nominal run peaks at `0.0063°`. A `4 N × 100 ms` forward torso push peaks
at `24.920°`, is briefly contactless for at most `5 ms`, and recovers in
`1.440 s`; the otherwise identical `6 N` overload falls. The admitted 4 N run
records controller p50/p99 `130.7/164.1 µs`, full-loop p99 `415.7 µs`, zero
5 ms overruns, zero Rust timed allocations, zero Python GC collections, and
`7.11e-9/5.21e-9` maximum admitted dynamics/contact residuals. Four startup
diagnostic candidates are rejected and held fail-closed through `20 ms`; every
later tick is admitted. See
[the r128 MuJoCo plant audit](benchmarks/results/upkie-mujoco-plant-r128/UPKIE_MUJOCO_PLANT_AUDIT.md).
The earlier [r127 failure](benchmarks/results/upkie-mujoco-plant-r127/UPKIE_MUJOCO_PLANT_AUDIT.md)
is retained rather than overwritten. Revision r131 now supplies the separately
typed live plant gateway described below. Direction,
friction/slope, repeated impulses, delay/noise, actuator dynamics/thermal
state, support-mode transitions, and exact/swept collision certification remain
subsequent work.

R129–r130 extend that boundary with rooted capture and station semantics. Rust
now consumes explicit `control_world <- odom` and `map <- odom` transforms,
computes a full sagittal DCM authority witness, fades the lower-priority odom
station anchor with a C1 curve, and runs the reference-matched Upkie PI state
without hot-path allocation. A 10,000-call policy-component corpus proves that
a 10 m map correction changes reporting while wheel commands and non-map
control diagnostics remain bit-exact. A seven-value, 10-second MuJoCo sweep
qualifies velocity fractions 0.05–0.40 and rejects 0.60/1.00 by fall; the frozen
0.20 default gives the fastest sampled station re-entry. The retained r130
4 N recovery returns tilt in `1.450 s`, re-enters the station envelope in
`1.870 s`, finishes at `6.9 µm` station error, records Rust p99 `166.0 µs`, and
has zero timed allocation, Python GC, 5 ms overrun, or post-startup rejection.
See the [rooted policy-component audit](benchmarks/results/upkie-rooted-capture-r129/UPKIE_ROOTED_CAPTURE_AUDIT.md),
[fraction sweep](benchmarks/results/upkie-capture-fraction-sweep-r130/UPKIE_CAPTURE_FRACTION_SWEEP_AUDIT.md),
and [r130 plant audit](benchmarks/results/upkie-rooted-capture-plant-r130/UPKIE_MUJOCO_PLANT_AUDIT.md).

R131 closes the browser PUSH transport at a narrow, explicit boundary. TARGET
remains a green guided query on `/ws`; orange PUSH lazily opens `/plant-ws`.
Rust validates finite wrench vectors, caps force at `8 N`, correlates commands,
expires an unrefreshed force after `140 ms`, and supervises one isolated
Python/MuJoCo child per client. The worker feeds measured root/joint state into
the persistent Rust capture/WBC sessions and returns torque to MuJoCo; the
browser renders measured plant transforms, a force arrow, and separate
capture/station/actuator/compute rows. Five fresh-session trials pass response,
recovery, invalid-input survival, expiry, reset, and timing, with maximum Rust
controller p99 `458.3 µs`, worker p99 `4.236 ms`, and stream p99 `41.5 ms`.
A supplemental localhost and Cloudflare trace also proves reported fall,
next-step automatic simulator reset, recovery to `0.024 mm` station error, and
fresh-worker reconnect without disguising that as hardware recovery. See the
[r131 live gateway audit](benchmarks/results/live-upkie-plant-gateway-r131/LIVE_UPKIE_PLANT_GATEWAY_AUDIT.md).

R132 turns the PUSH point into bounded physical authority. Rust admits the
request only when the body exists in the latest plant state and the world point
is within `750 mm` of that body origin. The Python worker independently checks
the exact MuJoCo COM lever, maps the force with `mj_applyFT`, and streams the
resulting moment and lever back to the browser. Five trials across fresh
−200/0/+200 mm sessions produce final pitch moments
`−0.400765/0.000289/+0.401712 N·m`; their `0.802477 N·m` differential matches
the `0.800000 N·m` cross-product prediction, and signed pitch/rate consequences
are strictly ordered with bit-exact physical replay. A 751 mm point is rejected,
the active push clears, and streaming continues. Controller and four-tick
worker maxima are `325.5 µs` and `2.071 ms`. See the
[r132 application-point audit](benchmarks/results/live-wrench-application-r132/LIVE_WRENCH_APPLICATION_AUDIT.md).

R133 supersedes the first disturbance matrix with a 20-case, first-boundary
plant-consequence envelope without moving control logic into Python. The frozen
matrix covers ±sagittal, ±lateral, diagonal, ±vertical, equal-impulse duration,
three repeated sagittal pulses, base-versus-handle application, and
μ=1.00/0.10/0.03. Every 200 Hz observation/torque tick crosses persistent Rust
`UpkieBalanceSession` and floating-WBC sessions. Ten controller cases qualify
and ten remain red. The canonical `4 N × 100 ms` row peaks at `107.6 mm` and
`24.91°` and recovers in `1.950 s`; ±4 N vertical rows recover in
`0.335/0.380 s`, and three 2 N pulses recover `1.690 s` after the final pulse.
A 1 N lateral push still falls. The ±2 N lateral pair crosses the boundary at
`1.985/1.990 s`, although its first-boundary paths differ by 48.59% and are not
misreported as spatially symmetric. Every red rollout stops at the first
45°/350 mm fall or numeric-fault boundary. All traces are finite with zero
MuJoCo warnings, eliminating the former post-fall BADQACC and 28 m launch while
retaining genuine controller tails during authority collapse. All eleven
evaluator gates pass with exact canonical replay and zero Rust timed allocation.
See the
[r133 disturbance-envelope audit](benchmarks/results/upkie-disturbance-envelope-r133/UPKIE_DISTURBANCE_ENVELOPE_AUDIT.md).

R134 adds an allocation-free experimental differential-wheel planar-capture
path without promoting it into the live controller. The generic PyO3 WBC batch
now accepts validated per-tick contact bases, while Rust owns heading-relative
DCM projection, persistent steering-direction state, bounded yaw-rate slew,
differential wheel acceleration, and 30-value diagnostics. A fresh MuJoCo A/B
preserves nominal and 4 N sagittal recovery and delays the left-1 N,
left-2 N, and right-2 N first-fall boundaries by `+0.400/+0.135/+0.515 s`,
with exact replay, finite traces, and zero timed Rust allocation—but recovers
`0/3` frozen lateral cases. The candidate still has positive lateral
DCM-to-track margins of `+114.8/+95.7/+78.7 mm` at those falls, proving that
the scalar margin is evidence rather than a nonlinear recovery certificate.
A full six-second neighboring-gain sweep recovers only one sign-specific row;
the mirrored rows fall and the response is non-monotone. Evaluation admission
passes, controller promotion is therefore rejected, and the proven live path
is unchanged. See the
[r134 planar-capture A/B](benchmarks/results/upkie-planar-capture-ab-r134/UPKIE_PLANAR_CAPTURE_AB_AUDIT.md).

R135 adds the missing policy- and physics-free local authority surface. Python
authors 196 immutable probes spanning four contact modes, six root axes, both
signs, and 1/10/50/250 native-unit requests; one persistent Rust floating-WBC
session owns every model, dynamics, contact, hierarchy, effort, residual,
timing, and allocation operation. All requests remain hard-feasible, with
maximum hard/dynamics/contact residuals `1.712e-12/1.712e-12/2.461e-13` and
zero timed Rust allocation. The retained JSON keeps achieved directional
response, orthogonal leakage, per-layer RMS/clipping, friction, effort, and
solver work separate. This explains the local roll/lateral authority boundary
but does not overturn r134's rejected controller promotion or r133's plant
outcomes. See the
[r135 state-local authority audit](benchmarks/results/upkie-state-local-authority-r135/UPKIE_STATE_LOCAL_AUTHORITY_AUDIT.md).

R136 turns failure handling into an explicit Rust state transition rather than
an indefinite last-command hold. Four independent pressure channels—tilt,
angular rate, root height, and previous-solver availability—feed a C1 risk
witness; primary authority falls by at most `0.04` per 5 ms tick and returns by
at most `0.005`. A minimum contingency hold prevents chatter, a low-energy
damping candidate is emitted into the ordinary WBC, stale command authority
expires to zero, and the `Fallen` state remains latched until an explicit
reset. A six-case fresh MuJoCo A/B preserves nominal behavior bit-exactly and
keeps the 4 N sagittal recovery qualified. Every declared failure enters the
typed degraded path before its first boundary, exact replay and zero timed
Rust allocation pass, and a 20-tick solver outage reaches zero command
authority. Live deployment is nevertheless rejected: the 6 N overload reaches
the first boundary `3.120 s` earlier. This is an admitted contingency contract,
not a safer controller or new recovery authority. See the
[r136 contingency-transition audit](benchmarks/results/upkie-fall-safe-contingency-r136/UPKIE_FALL_SAFE_CONTINGENCY_AUDIT.md).

R137 separates the deployable freshness fix from that rejected physical-risk
blend. The current WBC result must still be admitted; after a rejection, only
the last admitted torque receives a Rust-owned five-tick lease and fades to
zero by tick twelve. Nominal and the qualified 4 N recovery are bit-exact. All
four adverse first boundaries are no earlier (`+0.025…+0.365 s`), maximum stale
ages fall from `48/26/20/41` to `17/22/11/28` ticks, replay is exact, and timed
Rust allocation remains zero. This is promoted in the live worker as command
freshness, not as lateral or overload recovery. See the
[r137 stale-command expiry A/B](benchmarks/results/upkie-stale-command-expiry-ab-r137/UPKIE_STALE_COMMAND_EXPIRY_AB_AUDIT.md).

R138 identifies a concrete model/plant boundary underneath the remaining
falls. Sixteen frozen pre-fall observations contain exact named wheel-contact
patterns `11`, `10`, and `00`; 12 disagree with the controller's permanent
double-RollingWheel declaration. Ninety-six state-local queries reuse identical
q/v/root states and rate-arrest requests while changing only declared versus
measured active masks. All 48 measured-mask queries remain hard-feasible with
maximum residual `5.954e-12`; the stale double-contact rows produce 30
`MaxIterations` results and violation up to `216.339`. Replay is exact and
timed Rust allocation is zero. This admits contact evidence, not an estimator,
transition action, or recovery controller. See the
[r138 measured-contact audit](benchmarks/results/upkie-contact-truth-authority-r138/UPKIE_CONTACT_TRUTH_AUTHORITY_AUDIT.md).

R139 moves contact admission into generic allocation-free `bonesaw-core`.
Caller-owned tick time, mapped observation time, age, source identity, sequence,
and synchronization uncertainty are validated before state changes. Exact raw
contact, debounced mode, and hard-row eligibility remain separate; held,
missing, or rejected evidence is never hard. Mirrored single-contact, chatter,
expiry, and six malformed-observation families pass all nine gates with exact
replay. See the [r139 causal contact contract](benchmarks/results/upkie-contact-observation-contract-r139/UPKIE_CONTACT_OBSERVATION_CONTRACT_AUDIT.md).

R140–R142 retain three negative consumers rather than promoting a plausible but
unsafe mask switch. Direct reduced-support execution makes 9/10 green rows
fall. Keeping reduced-support results diagnostic-only preserves all green
physical outcomes but makes every baseline fall `0.345–3.455 s` earlier. Fixed
10/20/30% WBC normal-load-share thresholds provide zero advance coverage for
the exact contact edge, which also occurs transiently in green recoveries.
Live therefore remains on r137. See the
[r140 direct A/B](benchmarks/results/upkie-contact-observed-controller-ab-r140/UPKIE_CONTACT_OBSERVED_CONTROLLER_AB_AUDIT.md),
[r141 command-gate A/B](benchmarks/results/upkie-reduced-support-command-gate-ab-r141/UPKIE_REDUCED_SUPPORT_COMMAND_GATE_AB_AUDIT.md), and
[r142 precursor audit](benchmarks/results/upkie-contact-unload-precursor-r142/UPKIE_CONTACT_UNLOAD_PRECURSOR_AUDIT.md).

R143 closes the policy-/physics-free bounded contact-command lease contract.
Generic `bonesaw-core` owns cached command bytes, authoring support, tick age,
Fresh/Leased/Unavailable provenance, and typed evidence/support/sequence/finite
command revocation. The immutable mirrored/zero-support corpus passes 10/10
gates with exact replay and zero hot-path allocation. R144 then makes that lease
the sole post-loss command authority for five ticks. Lifetime, zero post-expiry
torque, replay, finiteness, and allocation gates pass, but 7/10 green
qualifications are lost and 7/9 existing falls move earlier, so physical
deployment is rejected. See the
[r143 contract](benchmarks/results/upkie-contact-command-lease-contract-r143/UPKIE_CONTACT_COMMAND_LEASE_CONTRACT_AUDIT.md) and
[r144 physical A/B](benchmarks/results/upkie-contact-command-lease-ab-r144/UPKIE_CONTACT_COMMAND_LEASE_AB_AUDIT.md).

R145 separately tests the more continuous composition: bounded r143 contact
grace first, followed by the already admitted r137 five-to-twelve-tick
freshness fade. The 0-tick row is execution-bit-exact with r141. All
0/2/4/8/16-tick rows preserve the ten green physical outcomes, exact replay,
finite state, and zero Rust allocation, but all nine r137 fall rows remain
earlier. Nonzero grace worsens aggregate fall time from the zero-row
`−12.675 s` to `−12.730/−12.775/−12.895/−13.015 s`. Timeout composition is
therefore closed as the missing recovery action. See the
[r145 composition sweep](benchmarks/results/upkie-contact-command-freshness-composition-r145/UPKIE_CONTACT_COMMAND_FRESHNESS_COMPOSITION_AUDIT.md).

R146–R148 turn the remaining viability question into a bounded causal
mechanism. A 245-query frozen-state oracle finds local forecast descent in
15/16 states; a 40-query coordinate selector strictly descends all 12 active
states; and generic allocation-free Rust owns activation/release hysteresis,
request slew, freshness, sequence rejection, typed provenance, and exact-
evidence revocation. Python owns evaluation search orchestration only, and all
nonzero requests still require a current exact WBC admission.

The external plant rejects the action. R149's four-arm 20-case study passes all
11 mechanism gates but moves 11/19 falling measured-control boundaries earlier
and records 395 planner ticks beyond 5 ms. R150 gates reduced support behind a
fresh executable request and thereby preserves all ten r137 green rows, exact
replay, and zero timed Rust allocation. It nevertheless recovers no adverse
case, moves eight adverse boundaries earlier (worst `−1.065 s`), and reaches a
`16.239 ms` worst per-case p99 controller step. The request remains
evaluation-only and the live worker remains r137. See the
[r150 request-gated plant A/B](benchmarks/results/upkie-conditional-viability-planner-ab-r150/UPKIE_CONDITIONAL_VIABILITY_PLANNER_AB_AUDIT.md).

R151 replaces the one-shot request score with an allocation-free eight-knot,
240 ms Rust viability forecast and a four-query-per-tick local poll. The first
broad wake rule loses five retained green rows. R152 restricts wake authority
to roll/lateral capture, retains the other pressure channels as explicit
vetoes, preserves every r137 green row, and contracts the worst adverse
regression to `−0.085 s`; it still recovers nothing and records 660 loop
overruns. R153 applies an eight-iteration planner-only active-set cap; plant
outcomes and query counts remain identical and the 19.992 ms worst p99 leaves
the dominant tail outside that inner cap.

R154 provides the causal four-arm result: r137 sentinel, measured-contact
control, candidate, and exact replay. All 11 mechanism gates pass. Against the
measured-contact control, earlier falling boundaries contract from r149's
11/19 to 4/19, the worst regression contracts `−1.490→−0.250 s`, and >5 ms
loop events contract 395→63. Promotion still fails: four boundaries remain
earlier, aggregate exact work is 20,957 queries, summed within-run RSS rises
2.00 MiB, and the measured-contact path itself does not retain r137's green
behavior. The live worker therefore remains r137. See the
[r154 causal plant A/B](benchmarks/results/upkie-multistep-measured-contact-plant-ab-r154/UPKIE_VIABILITY_REQUEST_PLANT_AB_AUDIT.md).

R155 moves signed proposal scheduling into allocation-free Rust and cuts active
planner work to a same-state zero baseline plus one proposal. It reduces exact
queries 20,957→12,140, but still moves 5/19 measured-control fall boundaries
earlier and records 61 loop overruns. R156 inserts a generic two-update shadow
confirmation before request supervision. Directional disagreement, raw support
change, evidence loss, failed descent, expiry, invalid input, or sequence
reordering revokes it. The full four-arm matrix passes all 15 mechanism gates:
19 shadow ticks, 26 confirmed ticks, one support revocation, exact replay, zero
timed Rust allocation, and zero Python GC. Physical promotion remains rejected:
2/19 boundaries are earlier, worst `−0.450 s`, and 60 loop overruns remain.
The retained r137 worker is unchanged. See the
[r155 paired-poll A/B](benchmarks/results/upkie-paired-multistep-measured-contact-plant-ab-r155/UPKIE_VIABILITY_REQUEST_PLANT_AB_AUDIT.md)
and [r156 confirmation A/B](benchmarks/results/upkie-confirmed-multistep-measured-contact-plant-ab-r156/UPKIE_VIABILITY_REQUEST_PLANT_AB_AUDIT.md).

R157 exposes the forecast's exact eight hold/coast state knots through the
allocation-guarded NumPy boundary and compares them with later 200 Hz plant
observations. The 20-case exact replay retains 26 confirmed origins, 173
complete knot comparisons, 35 fall-truncated comparisons, and zero timed Rust
allocation. Same-support normalized max error is p99/max `0.313/0.324`; the
148/173 comparisons that cross raw support reach `1.222/1.235`. At 240 ms the
median error is `1.140×` the configured state limits. This admits the
measurement mechanism, not an online envelope: the retained percentiles are
descriptive and the live r137 worker remains unchanged. See the
[r157 forecast-realization audit](benchmarks/results/upkie-forecast-realization-envelope-r157/UPKIE_FORECAST_REALIZATION_AUDIT.md).

R158 adds an allocation-free hybrid-support guard between proposal selection
and confirmation. Exact support must pass the same three-sample hard-contact
dwell; double support requires material load on both wheels; and an opening
single-support roll request receives physical authority only when it opposes a
signed roll-capture pressure of at least 0.20. A sub-threshold opening proposal
may advance the bounded confirmation shadow but cannot execute. The full
four-arm matrix passes all 19 mechanism gates with 42 physical admissions,
eight shadow-only direction rejections, nine dwell rejections, exact replay,
zero timed Rust allocation, and zero Python GC. Consequence is neutral-to-better:
0/19 falling boundaries are earlier, two are later (`left_2n` +0.095 s and
nominal +0.010 s), and 17 are neutral. Promotion remains rejected solely by the
timing gate: 58 loops exceed 5 ms. The retained r137 worker is unchanged. See the
[r158 hybrid-guard A/B](benchmarks/results/upkie-hybrid-guard-measured-contact-plant-ab-r158/UPKIE_VIABILITY_REQUEST_PLANT_AB_AUDIT.md).

R159 broadens calibration from the 26 confirmed proposal origins to every
fresh final exact-WBC command. Across a frozen 10-case calibration / 10-case
holdout split it retains 11,034 origins, 63,308 support-stable knot comparisons,
exact replay, and zero Rust allocations; path emission is 0.311 µs p50 and
0.511 µs p99. The calibration maximum plus 5% reserve covers only
32,207/32,787 holdout pairs (98.231%), falling from 100% at 30 ms to 95.681% at
240 ms, with a 14.275× worst bound miss. Excluding intervals containing a known
future external force improves aggregate coverage to 99.063% but still misses
the frozen bound. This is a useful continuous confidence measurement and a
clear negative certificate result—not authority. See the
[r159 executed-command forecast contract](benchmarks/results/upkie-executed-forecast-realization-contract-r159/UPKIE_FORECAST_REALIZATION_CONTRACT_AUDIT.md).

R160 tests the requested state/command/contact conditioning on 13 new holdout
cases. Its origin-time cells issue 99.026% of 57,999 comparisons, but retain 19
issued misses and need a 2.599×-state-scale largest bound. R161 adds the paired
position/rate coordinate and refuses cells wider than one state scale. That
reduces usefulness to 16.054% issuance and still leaves eight issued misses on
a second disjoint holdout. Both static certificates are rejected and have no
authority path. See the
[r160 conditioned certificate](benchmarks/results/upkie-conditioned-forecast-certificate-r160/UPKIE_CONDITIONED_FORECAST_CERTIFICATE_AUDIT.md)
and [r161 paired-state certificate](benchmarks/results/upkie-paired-state-forecast-certificate-r161/UPKIE_PAIRED_STATE_FORECAST_CERTIFICATE_AUDIT.md).

R162 replaces sparse static cells with a generic allocation-free Rust
execution-residual monitor. A prediction authored from the preceding tick's
final exact-WBC acceleration is checked against the *pre-update* 32-sample
envelope; only then is its residual inserted. Missing evidence or exact support
change resets warmup. On a frozen third 13-case holdout it observes 7,532
comparable ticks, covers 6,567, reports 70 causal exceedances, and sees 69 later
covered ticks within 250 ms. All replays are exact; the measured Rust transition
uses zero allocations and takes 6.833 µs worst-case. The 3961× worst exceedance
is retained as visible model-break evidence rather than learned away. This
signal may lower confidence or drive a health bar, but cannot authorize a
request. See the
[r162 causal residual audit](benchmarks/results/upkie-execution-residual-monitor-r162/UPKIE_EXECUTION_RESIDUAL_MONITOR_AUDIT.md).

R163 evaluates the only safe first consumer of that signal: an exceedance may
make the current proposal unavailable before confirmation, but cannot create a
request or touch fallback torque. Across the retained 20-case causal A/B, all
70 veto ticks have zero same-tick executable requests and zero Rust allocation.
All ten control-green rows survive, no first fall boundary moves earlier, and
the 6 N overload / handle fall move `+0.785/+0.395 s` later. The changed rollout
later contains 26 rather than seven executable-request ticks, so pointwise
authority and trajectory aggregate remain explicitly separate. Deployment is
still rejected: 692 candidate loops exceed 5 ms. See the
[r163 residual-veto plant A/B](benchmarks/results/upkie-execution-residual-veto-plant-ab-r163/UPKIE_EXECUTION_RESIDUAL_VETO_PLANT_AB_AUDIT.md).

R164 and r165 isolate the remaining CPU tail. R164 compiles exact nonzero
coordinate/value lists for sparse Dykstra rows. It cuts worst final-WBC p99
`6.11→2.28 ms`, worst loop max `39.5→5.12 ms`, and overruns `58→1`, but the
frozen plant gate rejects it: projections change `11,904,894→11,904,822` and
the nominal and forward-2 N execution traces are not bit-exact. R165 then tests a
default-off equality-first active-set repair. It cuts work to 942,630
projections, caps observed fallback at eight sweeps, removes all 32 paired-arm
overruns, and lowers worst loop p99 `4.76→1.65 ms`. That faster seed is physically rejected:
11 boundaries move earlier, worst `−0.485 s`. Neither profile is promoted. See
the [r164 sparse timing A/B](benchmarks/results/upkie-bounded-feasibility-timing-ab-r164/UPKIE_BOUNDED_FEASIBILITY_TIMING_AUDIT.md)
and [r165 equality-first physical A/B](benchmarks/results/upkie-equality-first-feasibility-plant-ab-r165/UPKIE_EQUALITY_FIRST_FEASIBILITY_PLANT_AB_AUDIT.md).

R166 tests exact-prefix continuation only on speculative planner queries. A
512-sweep candidate either resumes the same Dykstra point and multipliers when
its remaining violation is at most 0.1, or returns typed `MaxIterations`
without request authority; the executable WBC remains uncapped. Worst planner
p99 falls `6.008→1.290 ms`, but total >5 ms loops remain `58→58`. All 20 plant
execution traces are exact, while ten complete authority traces differ because
some speculative proposals now fail closed earlier. The profile is rejected:
it localizes the remaining deadline tail to the executable WBC and does not
justify changing planner semantics. See the
[r166 bounded planner-continuation A/B](benchmarks/results/upkie-planner-continuation-plant-ab-r166/UPKIE_PLANNER_CONTINUATION_PLANT_AB_AUDIT.md).

R170 isolates the signed-zero sparse kernel without the planner cap used by
r167. It repairs the exactness defect exposed by r164 without restoring dense zero
multiplies. Sparse Dykstra rows retain a fixed `u128` mask of actual `-0`
coordinates and reproduce the dense kernel's signed-zero transition only where
it can affect later bitwise active-set tie/freeze checks. The frozen 20-case
matrix is exact end to end: `11,904,894→11,904,894` projections, complete
authority/plant trace, replay, zero Rust allocation, and zero Python GC. On the
unisolated generic build, >5 ms loops fall `60→32`, worst final-WBC p99
`6.040→2.248 ms`, and worst loop max `42.474→13.680 ms`. The exact CPU kernel is a
real improvement but remains default-off because the zero-overrun gate still
fails. See the
[r170 signed-zero sparse feasibility A/B](benchmarks/results/upkie-signed-zero-sparse-feasibility-ab-r170/UPKIE_SIGNED_ZERO_SPARSE_FEASIBILITY_AUDIT.md).

R167 combines exact sparse-nonzero Dykstra traversal with a planner-only
anytime prefix: speculative queries stop at 512 sweeps unless their exact
remaining hard violation is at most `0.1`, in which case they resume from the
same point and multipliers. The authoritative WBC stays uncapped. On the full
20-case plant matrix, all executable commands, statuses, logical projection
counts, and plant traces are exact; Rust timed allocation and Python GC remain
zero. Synchronous loop misses fall `58→7`, while every authoritative WBC call
meets 5 ms (worst observed `3.479 ms`). This admits the host-native CPU WBC kernel profile,
not the combined synchronous supervisor: its seven remaining misses occur when
planner continuation shares a tick with final authority. The retained r137
worker is unchanged. See the
[r167 anytime feasibility A/B](benchmarks/results/upkie-anytime-feasibility-timing-ab-r167/UPKIE_BOUNDED_FEASIBILITY_TIMING_AUDIT.md).

R169 removes repeated hard-feasibility work inside each speculative planner
session without sharing state with final authority. A fixed-capacity cache key
contains every ordered hard coefficient and bound as exact IEEE-754 bits plus
the feasibility profile. Exact and polished seeds still run every soft task;
an exhausted cached result returns the same typed `MaxIterations` and never
gains command authority. The full 20-case cold/reuse/replay matrix records
9,669 deterministic hits, identical 18,332,262 logical half-space projections,
exact complete authority and plant traces, zero timed Rust allocation, and zero
Python GC. Host-native synchronous misses fall `5→2`; every final WBC remains
below 5 ms (worst `2.910 ms`). The cache profile passes its semantic and WBC
gates but the combined synchronous supervisor still fails, so it is not
promoted and r137 remains live. See the
[r169 bit-identical feasibility-reuse A/B](benchmarks/results/upkie-hard-feasibility-reuse-timing-ab-r169/UPKIE_HARD_FEASIBILITY_REUSE_TIMING_AUDIT.md).

R171 evaluates the first explicit cross-tick scheduling contract instead of a
wall-clock heuristic. The planner runs every three 5 ms ticks; intervening
ticks are typed no-update holds, request freshness is bounded to the same
period, and two-update confirmation may bridge exactly that gap. The complete
20-case host-native run is deterministic, replay-exact, allocation-free, and
clears the synchronous deadline (`3→0` misses; `4.836 ms` worst loop) while
reducing planner WBC queries `20,854→6,904`. It is nevertheless rejected:
executable-request exposure collapses `26→0`, nominal falls 10 ms earlier, and
left-2 N falls 95 ms earlier. The mechanism remains an opt-in evaluation knob,
not a controller profile. The result rules out uniform decimation and points
the remaining scheduler work toward a stateful/background pipeline that does
not starve confirmation. See the
[r171 cross-tick cadence A/B](benchmarks/results/upkie-cross-tick-planner-cadence-ab-r171/UPKIE_CROSS_TICK_PLANNER_CADENCE_AUDIT.md).

R172 and r173 isolate the session boundary before admission. R172 proves that
copied hard witnesses are exact and allocation-free when planner and authority
profiles match, but aligning the capped planner changes four executable-request
traces. R173 keeps that planner unchanged and permits terminal exact witnesses
only; semantics stay exact, while 54 refused exhausted copies leave two misses
in its retained run. These are negative controls, not promoted profiles. See
the [r172 matched-profile witness A/B](benchmarks/results/upkie-cross-session-feasibility-witness-ab-r172/UPKIE_CROSS_SESSION_FEASIBILITY_WITNESS_AUDIT.md)
and [r173 exact-only cross-profile A/B](benchmarks/results/upkie-cross-profile-feasibility-witness-ab-r173/UPKIE_CROSS_PROFILE_FEASIBILITY_WITNESS_AUDIT.md).

R174 removes the duplicated planner/final hard-feasibility prefix without
changing the every-tick request stream. The bounded planner copies an immutable
terminal seed or exhausted Dykstra point plus row multipliers into the separate
uncapped authority workspace. Rust rebuilds and bit-checks the complete ordered
hard problem; only the projection ceiling and continuation threshold may differ,
only in the bounded-to-uncapped direction. Terminal seeds enter the complete
soft hierarchy, while exhausted prefixes resume from the next sweep and remain
typed `MaxIterations` if the uncapped solve still exhausts. Across the frozen
20-case host-native matrix, all `11,185` copies are consumed (`11,131` terminal
hits and `54` prefix resumptions), logical work stays exactly `18,332,262`
half-space projections, and complete authority, execution, and replay traces are
exact. Synchronous misses fall `5→0`; worst final-WBC p99/max are
`2.667/2.945 ms`, worst loop is `4.348 ms`, and the copy boundary is `7.134 µs`
worst-case with zero Rust allocation and zero Python GC. This admits the CPU
evaluation compute profile, not the experimental viability policy or controller;
r137 remains live. See the
[r174 bounded-planner witness-continuation A/B](benchmarks/results/upkie-cross-profile-feasibility-witness-ab-r174/UPKIE_CROSS_PROFILE_FEASIBILITY_WITNESS_AUDIT.md).

R175 adds the missing state-local contingency action without turning it into a
recovery claim. A generic allocation-free Rust primitive consumes the exact
current support mask, FK/CoM state, root twist, and joint velocity. Double
support brakes toward the wheel midpoint, left/right support brake toward the
sole observed wheel with a coordinated inertial-tilt target, and flight requests
exact ballistic gravity instead of inventing a ground impulse. Python owns only
the frozen 256-state corpus and report: every request is submitted to a separate
floating WBC under the same support mask. All `256/256` actions are typed
`Solved`/`SolvedWithSlack`; fresh-session replay and query reordering are exact,
mirrored single-support actions agree, maximum hard violation is `2.05e-11`,
WBC p99 is `124.453 µs`, and Rust allocation/Python GC are zero. This admits
instantaneous action generation plus admission. Causal plant non-regression,
delay/noise/dropout transitions, and hardware realization remain open. See the
[r175 observed-support contingency admission](benchmarks/results/upkie-support-contingency-admission-r175/UPKIE_SUPPORT_CONTINGENCY_ADMISSION_AUDIT.md).

R176 supplies the complete causal consequence test and rejects the first action
without weakening its mechanism evidence. The 20-case evaluator retains r137,
measured-contact control, a non-executing r175 shadow, causal selection, and a
fresh selected replay. Shadow execution is physically exact; all 1,703 selected
ticks follow exact double-support arming, observed support loss, and a typed
separate-WBC admission; replay is exact, traces are finite, and timed Rust
allocation/Python GC remain zero. Promotion fails: nine of ten r137 green cases
physically fall and all ten lose qualification once 33 loop overruns are
included; the only measured-contact green row is also lost, and eight existing
fall boundaries move earlier. No live
controller changes. The next revision must change or condition the action while
retaining this authority contract, then add delayed/noisy/dropped observation
coverage. See the
[r176 causal support-contingency plant A/B](benchmarks/results/upkie-support-contingency-plant-ab-r176/UPKIE_SUPPORT_CONTINGENCY_PLANT_AB_AUDIT.md).

R177 explains the largest r176 regression without promoting a workaround. The
r139 contact filter intentionally requires three positive samples before adding
a hard contact; r176 let the primary WBC execute the startup/reacquisition
no-contact program during that interval. Keeping the established primary
outside this unconfirmed transition makes the non-executing shadow physically
bit-exact to r137 across all 20 cases and restores all 10/10 retained green
rows. The selected action still moves seven r137 red boundaries earlier by up
to `3.295 s`, and the preserved primary has no current observation authority,
so promotion remains rejected. Raw 200 Hz misses are `642/645/202` for the
r137/preserved-shadow/selected arms; unequal early terminations make these
counts evidence, not normalized speed ratios. See the
[r177 primary-support preservation diagnostic](benchmarks/results/upkie-support-contingency-primary-preservation-ab-r177/UPKIE_SUPPORT_CONTINGENCY_PRIMARY_PRESERVATION_AUDIT.md).

R178 adds the generic Rust mechanism implied by that diagnosis. The
allocation-free `ContactProgramAuthority` admits a fresh primary or
current-support command only when exact raw/stable/hard masks agree, uses only
the existing bounded retained-command lease during debounce, withholds at
startup without prior authority, and revokes on evidence/mask/sequence/command
faults or expiry. A fixed-array PyO3 transition exposes it for Python-owned
evaluation. All 266 workspace tests pass; plant integration and promotion are
still open.

R179 conditions the support-loss action without pretending the authority
composition is solved. Only the first exact flight transition gets a candidate
query, and that evaluation lease is consumed on accept or reject. Typed WBC
admission must be accompanied by at least `0.10` improvement from the Rust
eight-knot forecast over inertial continuation and achieved root angular
acceleration within `40 rad/s²`. In the five-arm 20-case replay, 17 queries
yield 11 transfers and six guard rejections; all 10 r137 green rows are
preserved, no fall boundary moves earlier, and three move later by
`0.200/0.055/0.035 s`. Shadow and replay are exact, WBC/forecast query maxima
are `143.902/0.521 µs`, timed allocation/GC are zero, and candidate overruns
fall `642→592`. This passes the action gates only inside r177's
preserved-primary scaffold. Controller promotion remains rejected until the
r178 current-observation combiner is composed without losing those results.
See the [r179 guarded flight A/B](benchmarks/results/upkie-guarded-flight-contingency-plant-ab-r179/UPKIE_GUARDED_FLIGHT_CONTINGENCY_PLANT_AB_AUDIT.md).

R180 composes measured contact, the r178 Rust contact-program state, and a
separate actual-support WBC realization without the fictitious r177 primary
support. The current-support query fixes the active primary-program actuator
effort and supplies an acceleration/contact-force witness against the exact
hard mask. Startup requires three explicit contact samples and establishes
primary authority first; transition-time current-support authority is allowed
only after prior authority exists. The evaluated profile deliberately uses a
zero-tick retained-command lease: the two-tick experiment moved red boundaries
and was rejected. Across five arms and all 20 cases, 1,227 fresh
current-support selections include 381 debounce selections, every execution
trace is bit-exact to r137, all 10 green rows survive, all 10 fall boundaries
are unchanged, replay and sparse-row semantics are exact, and Rust
allocation/Python GC remain zero. The mechanism passes, but promotion remains
rejected: the retained ordinary-process run has nine 5 ms loop misses and one
5.036 ms controller-call outlier, and delay/noise/dropout, mirrored-transition,
and higher-fidelity/hardware evidence remain open. See the
[r180 contact-program authority plant A/B](benchmarks/results/upkie-contact-program-authority-plant-ab-r180/UPKIE_CONTACT_PROGRAM_AUTHORITY_PLANT_AUDIT.md).

R181 adds the missing causal ablation rather than treating r180's exact result
as self-justifying. A raw independently optimized current-support transfer is
run beside the retained r137 sentinel, fixed-effort realization, and exact
replay. The raw transfer changes the terminal boundary or qualification in
16/20 rows. The realized path executes the effective r137 primary torque
bit-for-bit across all 18,209 control ticks: 16,982 primary selections and
1,227 current-support selections, with zero withheld, retained-lease, or
fallback ticks. All plant/command traces, 10 green rows, and 10 fall boundaries
remain exact; maximum hard violation is `2.77e-9`, the worst per-case proof-WBC
p99 is `108.011 µs`, the maximum proof solve is `128.713 µs`, and Rust
allocation/Python GC remain zero. This admits the semantic composition and
also proves that current-support realization must not feed false health back
into r137 freshness authority. The ordinary synchronous profile and hardware
controller remain rejected: this run retains 10 red-tail loop misses, a
5.639 ms controller-call maximum, and no delay/noise/dropout or hardware
evidence. See the
[r181 current-support realization ablation](benchmarks/results/upkie-current-support-realization-ablation-r181/UPKIE_CURRENT_SUPPORT_REALIZATION_PLANT_AB_AUDIT.md).

R182 executes the missing observation-robustness matrix rather than checking
only the contact state machine in isolation. Forty-nine paired/replayed plant
runs cover exact observation, 5/20 ms acquisition delay, periodic 5 ms and
10 ms dropout, left/right bit chatter, and mirrored ±1 N lateral pushes across
seven representative cases. All 234 unavailable ticks withhold authority and
emit exactly zero torque; both bit directions, the inclusive 20 ms age boundary,
finite execution, exact replay, zero Rust allocation, and zero Python GC pass.
All three exact-green cases remain green. Robustness is nevertheless rejected:
seven existing failures move earlier, led by handle/20 ms delay at `−1.625 s`
and low-friction/20 ms delay at `−0.985 s`, and the ordinary process records 56
5 ms misses. This turns the next gate from “add delay/dropout evidence” into
“make delayed/dropout consequence non-regressing.” See the
[r182 contact-program robustness A/B](benchmarks/results/upkie-contact-program-robustness-ab-r182/UPKIE_CONTACT_PROGRAM_ROBUSTNESS_AUDIT.md).

R183 separates steady sensor age from the timestamp discontinuity embedded in
the r182 delay rows. The r182-style onset correctly produces four rejected
backwards timestamps and four or five fail-closed startup ticks. With age-aware
prestart acquisition, every sample is monotonically newer and exactly 20 ms old
from the first runtime tick. All seven paired-replay runs then have zero
timestamp faults, zero withheld ticks, and plant/command/terminal traces that
are bit-exact to exact observation. This is specific to fixed-effective-effort
realization; it does not make delayed contact generically safe. It reclassifies
the two r182 delay regressions while leaving five periodic-dropout regressions,
17 loop misses, and a 5.569 ms controller tail open. See the
[r183 observation-delay startup A/B](benchmarks/results/upkie-observation-delay-startup-ab-r183/UPKIE_OBSERVATION_DELAY_STARTUP_AUDIT.md).

R184 shifts the periodic dropout phase by a full 250/500 ms period so loss is
first encountered after primary authority is established. This does not remove
the regression. All unavailable ticks withhold and emit zero torque, all three
exact-green cases recover, replay is exact, and Rust allocation/Python GC stay
zero, but ten case/profile boundaries move earlier. Established 5 ms gaps move
left/right/handle failures by `−0.685/−0.350/−1.245 s`; established 10 ms gaps
also regress mirrored and low-friction rows. This rejects zero torque as the
unavailable-observation command and leaves a separately admitted bounded
hold/brake/impact action as the next CPU authority problem. See the
[r184 observation-dropout phase A/B](benchmarks/results/upkie-observation-dropout-phase-ab-r184/UPKIE_OBSERVATION_DROPOUT_PHASE_AUDIT.md).

R185 adds a distinct Rust authority path for bounded inexact-observation
retention; it does not weaken or reuse the contact-transition lease. The typed
`RetainedInexactObservation` selection can replay only an already-admitted
effective effort for one or two ticks, carries explicit age/provenance, expires
to withheld zero torque, emits no contact-force witness, and remains
allocation-free. Across seven cases, eight profiles, and exact replay, all
mechanism gates pass; dormant hold-0/1/2 exact streams are identical and all
274 held ticks reproduce the preceding effort bit-for-bit. The one-tick 5 ms
hold is a strong near-miss: its only earlier boundary is the left row at
`−0.040 s`, while right and handle improve by `+0.515/+0.570 s`. It is still
rejected by strict non-regression. One- and two-tick 10 ms burst policies are
also rejected, worst `−1.655 s`; no physical profile is promoted, and the
ordinary process records 81 loop misses plus a 6.845 ms controller maximum.
See the [r185 inexact-observation hold A/B](benchmarks/results/upkie-inexact-observation-hold-ab-r185/UPKIE_INEXACT_OBSERVATION_HOLD_AUDIT.md).

R186 makes that retained-effort authority continuous in Rust with an exact,
non-compounding Q15 fraction. Zero disables retention and exactly matches the
withheld control; the same causal first-loss fraction is applied to established
5 ms and 10 ms gaps. All mechanism, replay, provenance, finite-output,
allocation, and GC gates pass across seven cases and 13 profiles. No tested
global fraction (`0/0.25/0.5/0.75/1`) passes strict plant consequence: each
advances an existing fall or creates a new one, and `0.5` turns the recovering
forward-4 N reference/5 ms row into a fall at 2.495 s. The current retained run
records 131 loop misses and a 6.955 ms controller maximum. See the
[r186 continuous inexact-hold authority sweep](benchmarks/results/upkie-inexact-hold-authority-sweep-r186/UPKIE_INEXACT_HOLD_AUTHORITY_SWEEP_AUDIT.md).

R187 makes the forbidden information explicit with a duration-oracle negative
control. It adds Q15 points `0.45` and `0.95` and scores 5 ms and 10 ms arms
separately. Both fractions pass the seven-case 10 ms consequence slice, while
no tested fraction passes the 5 ms slice and no fraction is admitted across
both. This is not a controller win: the first unavailable tick cannot know
whether the next sample will return. The denser sweep also confirms strongly
non-monotone behavior—neighboring fractions create green falls or move red
boundaries by over a second—so interpolation is not a safety argument. All
mechanism/replay/Q15/allocation/GC gates pass; the global policy and synchronous
profile remain rejected with 196 loop misses and a 7.182 ms controller maximum.
See the [r187 duration-oracle ablation](benchmarks/results/upkie-inexact-observation-duration-oracle-ab-r187/UPKIE_INEXACT_OBSERVATION_DURATION_ORACLE_AUDIT.md).

R188 replaces that oracle with a causal, allocation-free Rust selector. On
every unavailable tick it forces support unknown and scores retained-effort
fractions `0/0.25/0.5/0.75/1` from only the current reduced root state and the
last admitted command's achieved-acceleration/resource witness. Exact ties
choose lower authority; a matched 5/10 ms pair has identical first-loss state,
score vector, selection, and torque. All mechanism/replay/finiteness/allocation
gates pass. Across two independent full-process repetitions, the selector
maximum is `3.015–6.192 µs`. Plant consequence
rejects the score minimum as authority: it creates a new 3.000 s fall in the
recovering backward-4 N row and advances six existing boundaries, worst
`−1.995 s` at the handle. Those repetitions record 100–104 loop misses,
6.122–6.247 ms worst loops, and 5.697–5.866 ms controller maxima. See the
[r188 support-free forecast-selector A/B](benchmarks/results/upkie-inexact-hold-forecast-selector-ab-r188/UPKIE_INEXACT_HOLD_FORECAST_SELECTOR_AUDIT.md).

R189 adds a finite nonnegative forecast-improvement margin to that Rust
selector and sweeps `0/0.001/0.005/0.025/0.1/0.25/1.0` plus a `1e9`
withheld-equivalent endpoint. The gate is causal, allocation-free, and cannot
extend the independent one-tick command budget. Every mechanism gate passes,
including exact execution equality between the endpoint and explicit hold-0.
No threshold passes both 5 ms and 10 ms consequence: `0.001` removes R188's new
green fall but advances the left boundary by 0.620 s; larger margins converge
toward withheld failures. Across three repetitions, timing records 192–235
loop misses and a 5.699–6.645 ms controller maximum. See the [r189
improvement-gate sweep](benchmarks/results/upkie-inexact-hold-improvement-gate-ab-r189/UPKIE_INEXACT_HOLD_IMPROVEMENT_GATE_AUDIT.md).

R190 changes the action instead of gating cached torque. On every missing
contact sample, Rust authors a current-state flight-mode brake—ballistic
gravity, attitude damping, and joint damping—and a separate zero-contact
floating WBC admits it before generic authority emits typed selection 5. It
never refreshes Primary age/health or emits contact force. Mechanism, causal
prefix, replay, finiteness, and allocation gates pass. Across three
repetitions, author/WBC maxima span
`3.486–9.778/154.372–178.638 µs`. Plant consequence rejects the action: it improves both
left-fall boundaries by `+0.450/+0.885 s`, but creates forward/backward green
falls under 5 ms loss and advances right/handle/low-friction boundaries, worst
`−1.935 s`. The runs have 113–132 loop misses and 5.779–6.261 ms controller
maxima. See
the [r190 support-free brake A/B](benchmarks/results/upkie-inexact-support-free-brake-ab-r190/UPKIE_INEXACT_SUPPORT_FREE_BRAKE_AUDIT.md).

R191 implements the explicit terminal objective rather than another reduced
capture score. Allocation-free Rust predicts ballistic time-to a declared
`0.225 m` Upkie root-impact plane and vertical specific energy, then propagates
each already-admitted candidate's roll/pitch and joint acceleration to score
terminal tilt/rate, joint headroom/speed, effort, and admission separately.
Withhold is the baseline; retained or fresh support-free authority must improve
at least one component without regressing any. All mechanism, causal-prefix,
exact replay, independent re-audit, allocation, and GC gates pass in three
complete process runs. Each covers 62,817 first-run ticks and 481 audited
queries with zero online/re-audit disagreements. The retained artifact selects
withhold/retained/support-free `337/20/124` times; observed online-selector and
re-audit maxima span 0.992–8.106/0.972–1.102 µs.
The objective preserves both
green sagittal rows and delays handle/5 ms by 1.340 s, but still advances
left/right boundaries by as much as 0.680 s; plant and synchronous promotion
remain rejected with 115–126 loop misses, 6.620–6.777 ms controller maxima,
and 6.950–7.117 ms loop maxima. See the
[r191 terminal chooser A/B](benchmarks/results/upkie-inexact-terminal-chooser-ab-r191/UPKIE_INEXACT_TERMINAL_CHOOSER_AUDIT.md).

R192 measures the missing plant boundary instead of tuning the chooser. For
each of r191's 481 selected actions, it compares the exact candidate
roll/pitch and six-joint acceleration with the finite difference of measured
velocities over the next complete 5 ms MuJoCo interval. Every action spans all
seven named cases and the audit remains finite with zero Rust allocation and
zero Python GC. A leave-one-case-out componentwise maximum trained on the other
cases covers only 96.881% of complete samples (99.142% of scalar components);
even a 5% reserve covers only 97.089%, with a 25.234× worst miss. Wheel/contact
response dominates the tail. The global envelope is rejected and does not
change authority. See the [r192 realization calibration](benchmarks/results/upkie-terminal-realization-calibration-r192/UPKIE_TERMINAL_REALIZATION_CALIBRATION.md).

R193 corrects the withhold candidate itself. Zero actuator effort does not mean
zero generalized acceleration, so a separate no-contact fixed-zero-effort
floating WBC now supplies gravity/coupling qdd to the unchanged terminal
scorer. The correction is state-local, replay-exact, independently re-audited,
and allocation-free across 40,308 first-run ticks and 259 terminal queries.
All nominal and sagittal green rows survive, but consequence rejects the
chooser: handle/5 ms falls at 3.055 s, 1.575 s earlier than exact control and
2.915 s earlier than r191. At the first changed choice, support-free eliminates
2.472 joint-velocity pressure, but its 0.862 tilt pressure exceeds the corrected
withhold baseline's 0.781; the universal componentwise veto therefore changes
action 2→0. The corrected dynamics model stays, but the Pareto rule is not
promoted. See the [r193 zero-effort baseline A/B](benchmarks/results/upkie-inexact-zero-effort-baseline-ab-r193/UPKIE_INEXACT_ZERO_EFFORT_BASELINE_AUDIT.md).

R194 then measures that corrected candidate only where typed withhold actually
executes. The 179 following 5 ms plant intervals contain 169 physical
double-support samples, six flight samples, and four single-support samples,
even though the online contact observation is unavailable in every case. The
no-contact prediction therefore misses realized acceleration by as much as
1643.7 rad-or-m/s² and terminal-pressure norm by 28.200. Leave-one-case-out
componentwise pressure bounds cover 93.3% of mirrored-right samples and 86.4%
of handle samples; the mirrored-right miss exceeds its trained bound by
20.831. Offline Rust rescoring remains allocation-free with a 6.562 µs maximum,
but the empirical envelope is rejected. No-contact remains one explicit support
hypothesis, not a conservative substitute for unknown contact. See the
[r194 zero-effort support-realization calibration](benchmarks/results/upkie-zero-effort-realization-calibration-r194/UPKIE_ZERO_EFFORT_REALIZATION_CALIBRATION.md).

R195 evaluates the explicit alternative rather than assuming it is sufficient.
For each of three typed actions, the Rust/Python boundary runs fixed-effort
dynamics under none/left/right/double support, Rust takes a componentwise harm
envelope, and the following 5 ms plant interval is rescored against that same
envelope. The mechanism passes replay, first-loss causality, typed fail-closed
handling, and zero-allocation gates across 40,982 first-run ticks, 270 attempts,
and 266 valid selections. Realization does not: only 23/266 selections (8.65%) are
componentwise bracketed, with 267.523 maximum pressure exceedance and 271.247
maximum measured-physical-hypothesis pressure-error norm. Two low-friction
profiles have invalid baseline hypotheses on four loss ticks and correctly
execute typed zero effort rather than deleting those hypotheses. Plant
consequence and timing also reject promotion: handle/5 ms falls 1.245 s before
exact control, and 70 loop deadlines are missed with 6.578/5.793 ms
loop/controller maxima. The four-mode representation stays as visible
diagnostic structure, not authority. See the
[r195 support-hypothesis terminal envelope A/B](benchmarks/results/upkie-inexact-support-hypothesis-envelope-ab-r195/UPKIE_INEXACT_SUPPORT_HYPOTHESIS_ENVELOPE_AUDIT.md).

R201 replaces empirical smoothing with a physical, allocation-free Rust outer
bound. For every prospective contact it authors
`J_n,max = m_eff,max (1 + e_max) v_close,max + F_n,max Δt_max`, wraps the two
tangential axes in a conservative friction box, and projects the interval
through caller-owned `M⁻¹Jᵀ`. Python records wheel-bottom distance/velocity
before the interval and uses completed MuJoCo impulse only as a label. All six
predeclared physical profiles cover normal and tangential impulse on all 266
selections. The tight no-added-closing-reserve row has a 0.703 N·s p95 normal
bound and 18.0% p95 utilization; the declared 100 m/s² uncertainty row is more
conservative at 6.042 N·s and 1.7%. Its Rust p99 is 0.148 µs with zero
allocation. This admits the interval mechanism only—not controller authority.
Bonesaw-model `M⁻¹Jᵀ`, full velocity-jump coverage, useful width, plant
consequence, and deterministic 5 ms timing remain gates. See the
[r201 contact-transition interval audit](benchmarks/results/upkie-contact-transition-interval-audit-r201/UPKIE_CONTACT_TRANSITION_INTERVAL_AUDIT.md).

R202 removes the caller-owned dynamics response from that boundary. A reusable
Rust scratch now owns forward kinematics, floating mass assembly, one in-place
Cholesky factorization, prospective point Jacobians, every `M⁻¹Jᵀ` solve, and
directional effective mass. The full 12-coordinate velocity-jump audit unions
the four available support hypotheses for the selected terminal action across
266 retained intervals. The declared total-mass-floor profile covers 264/266
complete samples (99.248%) and 99.937% of components; both misses are root
linear x in the 4 N, μ=0.03 case, with 0.01665 m/s maximum exceedance, while
measured impulse remains 100% covered. Model response runs at 7.697 µs p99 and
interval projection at 0.771 µs p99 with zero allocation. Width is not hidden:
the same profile reaches 86.738 rad/s root-angular, 6.467 m/s root-linear, and
3294.670 rad/s joint p95 width. The model-owned mechanism is admitted, but the
interval is neither strict nor useful enough for authority; continuous
generalized-acceleration uncertainty, fresh morphology/friction/timing holdout,
plant consequence, and timing remain gates. See the
[r202 model-owned contact-response audit](benchmarks/results/upkie-contact-transition-response-audit-r202/UPKIE_CONTACT_TRANSITION_RESPONSE_AUDIT.md).

R203 separates continuous acceleration uncertainty from that contact impulse.
The allocation-free Rust interval accepts componentwise generalized-
acceleration lower/upper endpoints and evaluates every acceleration/time corner
before adding the unchanged model-owned contact response. A predeclared ±5
m/s² root-linear reserve closes the R202 low-friction misses and covers all
eight samples in a new μ=0.02 row, but still misses one retained
backward/drop10 root-angular-y component by 0.01254 rad/s; doubling the linear
reserve cannot affect it. The separately predeclared structured ±5 rad/s² root
angular, ±5 m/s² root linear, ±50 rad/s² joint row covers 274/274 complete
samples, including the fresh friction row, at 8.046/0.851 µs model/interval p99
with zero allocation. It adds only 0.05 rad/s angular, 0.05 m/s linear and
0.5 rad/s joint width over 5 ms, but the underlying independent friction box
still leaves p95 widths of 29.779 rad/s, 2.255 m/s and 1154.415 rad/s. The
mechanism is admitted; the frozen narrow primary remains failed and authority
stays off. See the
[r203 continuous-acceleration interval audit](benchmarks/results/upkie-contact-transition-acceleration-interval-audit-r203/UPKIE_CONTACT_TRANSITION_ACCELERATION_INTERVAL_AUDIT.md).

R204 attacks the dominant width rather than inflating acceleration reserves.
Rust limits each tangential impulse axis by
`min(μJ_n, m_eff |v_slip| + F_t Δt)` using directional effective mass and
prospective slip. Signed world-frame impulse components are retained as
label-only evidence. The primary passive 100 m/s², 8 N profile covers every
measured impulse and the fresh μ=0.02 row, while shrinking p95
root-angular/root-linear/joint width from 29.779/2.255/1154.415 to
9.687/0.991/58.020—joint width is 5.03% of the Coulomb baseline. It covers only
261/266 retained velocity samples (98.120%); the worst miss is 0.13226 rad/s
pitch velocity in backward/drop10. Rust bound p99 is 1.237 µs with zero
allocation. The mechanism is retained as a diagnostic width layer; strict
coverage, typed slip/load provenance, another morphology/contact-law holdout,
consequence and timing still block authority. See the
[r204 directional contact-transition audit](benchmarks/results/upkie-directional-contact-transition-audit-r204/UPKIE_DIRECTIONAL_CONTACT_TRANSITION_AUDIT.md).

R205 localizes why complete signed-impulse coverage does not imply a complete
generalized-velocity tube. A label-only ablation supplies the exact completed
world impulse to the Bonesaw model response. Support acceleration alone covers
10.526% of retained samples; projecting exact impulse at the causal
prospective point raises coverage to 83.835%, but replacing that point with the
completed normal-impulse centroid lowers coverage to 81.579%. The centroid is
20.001 mm from the prospective point at p95 and 21.702 mm at maximum; it also
raises maximum component miss from 2.598 to 5.317/s. The result rejects a
single relocated force point as the missing answer. The next contact layer
must retain spatial impulse moment/contact distribution, with root/generalized-
momentum residual kept independently calibrated. Both exact impulse and
centroid are post-step labels and admit no authority. See the
[r205 response-localization audit](benchmarks/results/upkie-contact-response-localization-audit-r205/UPKIE_CONTACT_RESPONSE_LOCALIZATION_AUDIT.md).

R206 tests the stronger spatial answer. The plant integrates the complete
signed force impulse and moment about world origin—including contact free
torque—over every 1 kHz substep, then translates it to the causal prospective
point. Rust emits a twelve-axis spatial response/Delassus operator and a
separate generalized-momentum residual `M(q)Δv`. Exact spatial wrench does not
improve the decomposition: retained coverage changes 83.835→81.579%, fresh
stays 62.5%, and the momentum-residual p95 is essentially unchanged
0.06636→0.06645. The measured moment is only 0.001003 N·m·s p95. Spatial
response and momentum machinery are retained at 11.595/5.899 µs p99 with zero
allocation. A separate `MΔv` conservation check against MuJoCo's accumulated
constraint impulse improves p95 reconstruction residual 0.010120→0.009407,
showing that the spatial wrench is more faithful even though the force-only
point benefits from error cancellation in pose coverage. Completed wrench
remains a label and authority stays off. See
the [r206 spatial contact-wrench audit](benchmarks/results/upkie-spatial-contact-wrench-audit-r206/UPKIE_SPATIAL_CONTACT_WRENCH_AUDIT.md).

R207 retains the complete contact-space coupling instead of collapsing it to
six directional masses. Rust emits the symmetric two-wheel Delassus matrix
`J M⁻¹Jᵀ` and solves a bounded passive impulse with deterministic
forward/reverse projected sweeps. The 274-sample audit finds real but modest
cross-wheel coupling (3.25% p95) and a nearly singular operator (condition
number 4.73e9 p95). Raw complete-velocity coverage is 10.219%. More iterations
make label agreement worse: one→sixteen sweeps changes impulse RMSE from
0.05087→0.05445 N·s and maximum generalized miss from 62.118→156.499, while
cost rises 0.444→3.478 µs p99. A leave-one-named-case-out residual reaches at
most 95.985% and is explicitly not authority. The full response query costs
9.721 µs p99 with zero allocation, and the frozen NPZ permits subsequent
policy/physics-free regularization sweeps. See the
[r207 coupled contact-response audit](benchmarks/results/upkie-coupled-contact-response-audit-r207/UPKIE_COUPLED_CONTACT_RESPONSE_AUDIT.md).

R208 performs the promised policy/physics-free follow-up: the evaluator loads
only the immutable R207 NPZ and runs a predeclared 7-ratio × 5-sweep grid
through the Rust kernel. Explicit diagonal compliance is allocation-free and
keeps physical post-contact velocity evaluated through the unmodified
Delassus operator. No scalar ratio dominates. Ratio 10 lowers impulse RMSE to
0.04374 N·s but worsens LOCO coverage and joint residual width; ratio 0.1
slightly improves retained raw coverage and root widths but worsens maximum
miss. Fresh raw coverage remains 62.5% throughout. The result retains
compliance as visible model machinery without selecting a label-tuned online
profile. See the
[r208 coupled-contact regularization replay](benchmarks/results/upkie-coupled-contact-regularization-replay-r208/UPKIE_COUPLED_CONTACT_REGULARIZATION_REPLAY.md).

R209 implements the separate generalized-momentum route without hiding a
plant rollout inside evaluation. Rust computes
`M(q)(Δv_observed−Δv_predicted)` into caller-owned covector storage and maps a
signed covector box back through the exact full `M⁻¹` into a generalized-
velocity interval, with atomic validation and zero timed allocation. Python
fits strict leave-one-named-case-out boxes on the immutable R206 replay and
performs zero physics and zero policy steps. The narrow nearest-candidate
oracle diagnostic covers 98.872% retained and 100% fresh samples at
49.056/5.691/179.104 root-angular/root-linear/joint p95 width; enclosing all
candidate residuals is wider and does not improve coverage. A 1.25× expansion
still misses the same retained samples. The mechanism is retained, but the
label-fit tube is rejected for authority because it is not an online-causal
calibration, is less useful than R204's 9.687/0.991/58.020 width, fails strict
retained coverage, and has no second-morphology/contact-law witness. See the
[r209 generalized-momentum residual replay](benchmarks/results/upkie-momentum-residual-tube-replay-r209/UPKIE_MOMENTUM_RESIDUAL_TUBE_REPLAY.md).

R210 asks the narrower optimistic question: if the exact completed R206
spatial wrench is subtracted first, can the remaining generalized-momentum
box become strict without losing R204-scale width? The coordinate LOCO row
reaches 99.624% retained / 100% fresh and 50.823 rad/s joint width, but still
misses one right-wheel component and widens root angular/linear response to
28.683 rad/s / 3.370 m/s. A group-symmetric box reaches 100%/100% only at
45.463/5.384/584.787. Thus even oracle-quality contact accounting does not
make a simple unconditioned residual box useful. Residual and coordinate/group
projection p99 are 6.717 and 14.198/8.181 µs with zero allocation; no profile
or authority is promoted. See the
[r210 spatial-conditioned residual audit](benchmarks/results/upkie-spatial-conditioned-momentum-residual-audit-r210/UPKIE_SPATIAL_CONDITIONED_MOMENTUM_RESIDUAL_AUDIT.md).

R211 removes the remaining Upkie-only API boundary from this machinery. A
generic Rust `ContactTransitionModelSession` resolves arbitrary compiled URDF
frames once and exposes allocation-free point/spatial Delassus,
generalized-momentum residual, and exact full-inverse-mass box queries through
caller-owned NumPy arrays. A zero-policy, zero-controller, zero-physics oracle
compares the momentum queries against independently assembled Pinocchio 4.0
floating mass matrices over 64 pinned-Upkie and 64 pinned-Unitree-G1 states.
Singletons, signed boxes, and three-candidate residual batches agree within
2.984e-13 maximum absolute error against a 1e-9 D1 gate. Upkie/G1 box p99 are
8.781/77.813 µs and residual p99 are 6.500/52.119 µs, with zero timed Rust
allocation. This admits cross-morphology model machinery only; it does not
transfer R204/R210 residual calibration or contact/consequence authority to
G1. See the
[r211 cross-morphology momentum oracle](benchmarks/results/floating-momentum-cross-morphology-oracle-r211/FLOATING_MOMENTUM_CROSS_MORPHOLOGY_ORACLE.md).

R212 freezes a causal grouped continuous-acceleration reachable set before the
next morphology. It reruns only the Rust directional bound over the immutable
274-sample R207 replay—zero physics, policy, or controller steps. Raising the
root-angular reserve alone from 5 to 50 rad/s² closes four of five R204 misses;
the remaining invariant root-linear miss proves this is not a one-axis fix.
The round **50/10/50** root-angular/root-linear/joint profile covers all 266
retained and eight fresh samples at p95 width 10.137/1.041/58.020. The R204
baseline replays bitwise-identically and the primary Rust bound costs 0.491 µs
p99 with zero allocation. This is Upkie calibration evidence, not authority;
the profile is frozen unchanged for r213. See the
[r212 grouped reachable-set replay](benchmarks/results/upkie-grouped-acceleration-reachable-set-replay-r212/UPKIE_GROUPED_ACCELERATION_REACHABLE_SET_REPLAY.md).

R213 applies that exact 50/10/50 construction to the pinned official G1. Every
one of 96 five-millisecond pre-impact samples resets independently under either
soft/pyramidal/RK4 or stiff/elliptic/implicit MuJoCo contact over eight
primitive foot points, with zero controller or policy steps. The generic Rust
directional bound covers 100% of samples under both laws, but the independent
point boxes compound through G1's leg Jacobians: p95 width becomes
6.660/1.043/1383.925 root-angular/root-linear/joint units, failing every frozen
2.0/0.5/10.0 usefulness gate. An independently frozen momentum sensitivity is
also rejected at 34.881/0.917/336.739. The generic mechanism and zero-
allocation 2.486 µs worst-law p99 stay; both profiles and authority are
rejected. The next construction must couple the finite foot patch or carry a
causal spatial-wrench set instead of summing eight independent point boxes.
See the [r213 G1 contact-law transition holdout](benchmarks/results/g1-contact-law-transition-holdout-r213/G1_CONTACT_LAW_TRANSITION_HOLDOUT.md).

R214 replaces axis-aligned generalized-impulse geometry with a mass-metric
ellipsoid `pᵀM⁻¹p ≤ E₂`. Rust computes exact component support from the inverse-
mass diagonal without allocation. An analytic radius frozen before two fresh
G1 laws (`f=0.50`, pure-translation scale 0.02453 m/s) cuts p95 width to
1.092/0.075/19.966—about 69× narrower at the joints than r213. The medium law
covers 48/48 reset samples; the hard pyramidal/Euler law covers 47/48 and
misses one `left_ankle_roll_joint` component while joint width still exceeds
the 10 rad/s gate. The primitive is retained; the untuned profile and authority
are rejected. See the
[r214 kinetic impulse ellipsoid holdout](benchmarks/results/g1-kinetic-impulse-ellipsoid-holdout-r214/G1_KINETIC_IMPULSE_ELLIPSOID_HOLDOUT.md).

R215 implements the complementary causal contact construction: one resultant
spatial wrench per G1 foot, with passive-slip/Coulomb tangential impulse and
CoP moments tied to the same nonnegative normal impulse. The fixed-capacity
Rust support function explicitly checks both passive-slip saturation kinks and
is allocation-free at 3.410 µs p99. On two further fresh
law/state sequences, compliant/elliptic/RK4 covers 48/48 while rigid/elliptic/
implicit covers only 40/48. P95 width is still
3.529/0.655/878.035 and 3.180/0.634/923.910—better than eight independent
points but nowhere near useful. Thus neither the full patch reachable set nor
one scalar kinetic residual works alone. The next construction needs a causal
center contact prediction plus state-conditioned kinetic residual; profile and
authority remain rejected. See the
[r215 spatial-patch transition holdout](benchmarks/results/g1-spatial-patch-transition-holdout-r215/G1_SPATIAL_PATCH_TRANSITION_HOLDOUT.md).

R216 composes the two retained mechanisms without taking another plant step.
From the immutable R215 replay, one resultant point per foot is predicted only
from prospective corner gap/velocity, exact model effective mass, declared
friction, and declared contact relaxation. Rust then evaluates exact support
for independently budgeted root and articulated impulse ellipsoids; Python
fits an affine severity envelope for construction diagnosis. The allocation-
free split primitive matches an independent block-matrix oracle and the fitted
envelope covers 96/96 construction samples, but p95 width is
15.890/0.994/141.487. More fundamentally, the causal predictor's rigid-law
p95 joint residual is 11.075 rad/s, so any centered component interval that
covers it has an unavoidable 22.151 rad/s p95 width before residual geometry
is chosen. The predictor/profile are rejected; the next missing mechanism is
coupled contact-law realization and CoP evolution, or a typed estimator
uncertainty set. See the
[r216 zero-physics causal-center replay](benchmarks/results/g1-causal-center-split-kinetic-replay-r216/G1_CAUSAL_CENTER_SPLIT_KINETIC_REPLAY.md).

R217 replaces the independent causal-center approximation with one coupled
eight-point time-step solve through the full Delassus operator. Rust owns the
allocation-free projected solve, nonnegative normal impulse, circular Coulomb
projection, and the generic NumPy boundary; Python owns the immutable replay
and construction statistics. The compliant-law joint residual falls to
1.521 rad/s p95, but the rigid law remains 11.277 rad/s, imposing a
22.554 rad/s minimum centered width. Its all-label profile is therefore not
authority. See the
[r217 coupled contact-law construction replay](benchmarks/results/g1-coupled-contact-law-replay-r217/G1_COUPLED_CONTACT_LAW_REPLAY.md).

R218 evaluates that construction unchanged on two untouched contact laws and
state ranges: soft/pyramidal/Euler and stiff/elliptic/RK4. All 96 samples reset
independently; 480 MuJoCo substeps generate labels, while policy and controller
steps remain zero. Both laws reach strict coverage and all timed Rust queries
remain allocation-free, but frozen p95 width is 2.420/0.142/35.754 and
13.170/0.785/175.201. The stiff predictor alone requires at least
17.498 rad/s of centered joint width, so the holdout rejects the calibration
without tuning it. The generic solver stays; typed estimator uncertainty or a
higher-order compliant law is the next gate. See the
[r218 fresh-law holdout](benchmarks/results/g1-coupled-contact-law-holdout-r218/G1_COUPLED_CONTACT_LAW_HOLDOUT.md).

R219 makes contact-estimator ambiguity a typed, finite Rust boundary rather
than an implicit residual scalar. Each explicitly enumerated hypothesis owns
contact velocity, impulse caps, friction, restitution, and compliance while
sharing one state-local Delassus and generalized response. Caller-owned scratch
keeps the query allocation-free, every scenario validates before outputs
change, and the API states that it does not certify unenumerated values between
scenarios. On immutable R218 labels, a 315-row set including all `2⁸` signed
normal-velocity patterns reaches only 81.25% strict coverage at a useful
9.185 rad/s joint p95 width. Larger rows reach 92.708% at 23.306 rad/s; cap
inflation reaches 96.875% near 22.5 rad/s and still misses. Those 315-scenario
queries also take 6.383–7.784 ms p99 in the retained run, exceeding the 5 ms
period. No construction row advances to a fresh holdout. See the
[r219 typed contact-hypothesis replay](benchmarks/results/g1-contact-estimator-hypothesis-replay-r219/G1_CONTACT_ESTIMATOR_HYPOTHESIS_REPLAY.md).

R220 replaces static scenarios with stateful compliant evolution inside the
prediction interval. Rust carries signed gap and contact velocity through fixed
microsteps, applies explicit Kelvin–Voigt normal impulse and circular Coulomb
tangent projection, updates every point through the full Delassus operator,
and advances gap after the coupled impulse. The construction-selected
128-microstep profile covers all 96 immutable R218 labels with a fitted
1.748/0.299/9.802 groupwise residual width and a 55.705 µs p99 query. The
profile is frozen but not promoted. See the
[r220 compliant construction replay](benchmarks/results/g1-substepped-compliant-contact-replay-r220/G1_SUBSTEPPED_COMPLIANT_CONTACT_REPLAY.md).

R221 tests that exact profile on untouched mid/elliptic/implicit-fast and hard/
pyramidal/RK4 laws at new state offsets. The mid law covers 44/48 complete
samples; the hard law covers only 31/48. Component coverage remains
99.7126%/98.1322%, queries stay near 50 µs p99 with zero Rust allocation and
bitwise repeat, and all non-timing artifacts reproduce exactly—but strict
coverage is conjunctive. The mechanism stays and the frozen profile is rejected
without holdout tuning. See the
[r221 compliant fresh-law holdout](benchmarks/results/g1-substepped-compliant-contact-holdout-r221/G1_SUBSTEPPED_COMPLIANT_CONTACT_HOLDOUT.md).

R222 quantifies why the rejected point prediction cannot drive terminal
selection even when its componentwise residual is nominally covered. A generic
Rust batch scores paired predicted/oracle post-contact states through the
existing ballistic terminal proxy without physics, policy, controller, or
integration. The predictor is optimistic on 33/48 mid-law and 34/48 hard-law
samples, causing 7 and 1 false-safe harm-threshold crossings. All eight occur
inside the frozen R220 componentwise tube: the interval itself must be
propagated through nonlinear consequence rather than scoring only its center.
Pair scoring is below 1 µs p99, bitwise repeatable, and allocation-free. See the
[r222 terminal consequence audit](benchmarks/results/g1-compliant-terminal-consequence-audit-r222/G1_COMPLIANT_TERMINAL_CONSEQUENCE_AUDIT.md).

R223 separates the editor's authored and realized state instead of making the
user infer the boundary from mode switches. TARGET retains the green guided
WBC pose and labels it `WBC TARGET PREVIEW · NOT PLANT`; an always-running
dashed orange rig shows `MUJOCO MEASURED` state and its contacts. PUSH makes
that measured stream primary. The viewport now fills the exact z=0 plane,
reserves 0.25 mm collision clearance for the preview, reports collision and
visual-mesh clearance separately, and colors visual vertices below ground.
The end-to-end gate transforms all 41 visuals and 25 STL instances from the
streamed frames: reachable and one-metre-down/clamped base intents retain
0.250000 mm collision clearance and at least 0.135636 mm exact visual
clearance locally and through Cloudflare. MuJoCo additionally streams root
motion, six actuator efforts, twelve generalized accelerations, twelve
generalized constraint forces, kinetic/potential energy, warning count,
contacts, penetration, and solver state at the existing 250/50 Hz split. See
the [r223 live ground/simulator-state audit](benchmarks/results/live-ground-sim-state-r223/LIVE_GROUND_SIM_STATE_AUDIT.md).

R234 makes that simulator boundary geometric rather than label-only. The
filled grid is constructed from MuJoCo's streamed plane point and normal, and
TARGET overlays the measured collision body as an orange wireframe alongside
the dashed measured rig. MuJoCo's physical CoM and its ground projection are
separate from the green preview CoM. The plant record adds actual actuator
force, generalized actuator/passive/bias force, scalar constraint
force/position/velocity, solver forward/inverse residual, exact constraint-row
count, and summed ground-normal load. The expanded live gateway passes on the
existing single port-8777 service, while the one-metre-down preview still
retains 0.250000 mm collision and 0.162025 mm exact visual clearance. Green
base intent remains a non-plant query; R234 admits observability, not physical
realization. See the [r234 measured MuJoCo ground-state audit](benchmarks/results/live-mujoco-ground-state-r234/LIVE_MUJOCO_GROUND_STATE_AUDIT.md).

R224 closes the nonlinear consequence-propagation gap exposed by R222. Generic
Rust now scores an entire componentwise generalized-velocity box: ballistic
impact time is bounded from vertical-speed endpoints, while root attitude,
angular rate, joint position, and joint velocity use conservative interval
propagation across that time interval. Pressure and aggregate fields are upper
bounds and joint headroom is a lower bound. A 729-point core oracle, the
generic Python boundary, late-row atomicity, bitwise repeat, and zero timed
allocation all pass. On the immutable R221 replay, observed false-safe
threshold crossings fall from 7→0 and 1→0; no contained completed state
violates the bound. Query p99 is 1.388/1.439 µs. The frozen transition profile
still covers only 91.667% of mid-law source rows and 64.583% of hard-law source
rows (the contact-only terminal projection covers 91.667%/39.583%), and the
wide bound conservatively rejects 10/48 and 2/48 safe completed states. The
mechanism stays; profile, selector, and authority remain rejected. See the
[r224 terminal velocity-box audit](benchmarks/results/g1-terminal-velocity-box-audit-r224/G1_TERMINAL_VELOCITY_BOX_AUDIT.md).

R247 adds a Rust-owned `ContactTransitionModelSession` selector over exactly
three R224 velocity-box candidates. It validates every row before mutating
diagnostics, repeats the conservative componentwise chooser in the timed
region, and the Python contract test observes zero timed Rust allocation,
deterministic selection, and late-row atomic rejection. The state-local R247
audit then evaluates zero-desired-acceleration WBC, velocity damping, and
neutral recovery on all 96 spent R246 states without policy or physics. All
288 primary WBC queries admit below the five-millisecond p99 gate, the selector
chooses 85 / 8 / 3 candidates with zero component regression, and an
independent rerun reproduces 34/34 non-timing arrays bitwise. Only root height
and model-predicted contact activation are opened per sample; the residual
width was fit on spent R246 labels, so this is a design freeze rather than a
holdout. It emits no actuator torque, plant command, lease, or authority. The
missing gate is a fresh MuJoCo baseline/candidate non-regression matrix with no
candidate retuning. See the [r247 terminal-box WBC action audit](benchmarks/results/g1-terminal-box-wbc-action-audit-r247/G1_TERMINAL_BOX_WBC_ACTION_AUDIT.md).

R248 runs that matrix on the primitive two-probe G1 fixture, two new pyramidal
contact laws, and disjoint offsets 250,000/260,000. Each of 96 pre-impact
states is forked before advancement:
the baseline holds zero generalized joint effort and the candidate holds the
R247-selected WBC torque for five 4 ms MuJoCo steps, one 20 ms / 50 Hz WBC
tick. The mechanism passes with zero policy queries, 960 physics steps, no MuJoCo warnings, zero
timed Rust allocation, and sub-five-millisecond WBC p99. The action profile
does not: only 14/48 implicitfast and 12/48 RK4 rows avoid every terminal
component regression. Joint-position pressure dominates. R248 is retained
as failed evidence that torque magnitude constraints do not model actuator
bandwidth or slew. The next profile must add that realization on spent R248
data, freeze it, and face new laws/offsets. See the [r248 fresh plant A/B](benchmarks/results/g1-terminal-box-wbc-plant-ab-r248/G1_TERMINAL_BOX_WBC_PLANT_AB.md).

R249 adds the Rust-owned first-order actuator realization boundary to the
failed R248 effort rows at the requested 20 ms / 50 Hz cadence. Four declared
sensitivity profiles replay 96 reset-every-sample efforts with no policy or
physics: all repeat bitwise, allocate zero Rust bytes, and stay below 5 ms
p99; the 25 Hz / 1,000 N·m/s row exercises three slew-limited coordinates. A
stricter R250 spent-state action freeze passes realized effort through fixed-effort
WBC and MuJoCo, but finds no useful safe profile, so no fresh action or
authority follows. See the [r249 realization audit](benchmarks/results/g1-actuator-realization-profile-audit-r249/G1_ACTUATOR_REALIZATION_PROFILE_AUDIT.md)
and [r250 action rejection](benchmarks/results/g1-actuator-bandwidth-action-freeze-r250/G1_ACTUATOR_BANDWIDTH_ACTION_FREEZE.md).

R251 isolates why R250's global boxes collapse to zero: candidate and baseline
plant error is strongly correlated. A policy-free replay forms paired
candidate-vs-zero velocity residuals and partitions them only by fixed causal
height, tilt, and root-rate signs. Coarse paired tubes reduce the independent
maximum span substantially, but completed labels remain offline-only and the
existing Rust selector cannot consume a paired delta, so no action or
authority is frozen. See the [r251 paired-delta audit](benchmarks/results/g1-paired-terminal-delta-audit-r251/G1_PAIRED_TERMINAL_DELTA_AUDIT.md).

R225 closes the online external-load provenance gap without conflating command,
impact evidence, and model uncertainty. A generic allocation-free Rust type
distinguishes declared continuous wrench, measured impact impulse, and
unobserved-model reserve. `/plant-ws` protocol 2 requires a source plus world
force/application-point frames and permits only `interactive_operator` and
`evaluation_harness` declared wrenches. Evidence-only impact/reserve classes
are rejected before worker command mutation; the MuJoCo worker independently
revalidates the same schema. Accepted provenance is echoed in measured plant
state, while impact impulse and model reserve remain separate explicitly
unavailable records. The retained public Cloudflare gate rejects missing
provenance and both evidence-only classes without interrupting the stream,
accepts and releases both declared sources, and observes zero MuJoCo warnings.
This does not authenticate an operator or estimate either unavailable quantity.
See the [r225 external-load provenance audit](benchmarks/results/external-load-provenance-r225/EXTERNAL_LOAD_PROVENANCE_AUDIT.md).

R226 replaces R220's constant-impedance/reference-scaling approximation with
the documented positive time-constant/damping-ratio law, the full
position-dependent impedance spline, the refsafe time-constant clamp, declared
circular/pyramidal friction sections, and a separately typed causal free point
acceleration. A second generic kernel distributes each soft velocity increment
through the complete Delassus operator with fixed forward/reverse projected
sweeps. The audit reuses the immutable rejected R221 labels and performs 96
prestate forward-dynamics queries, but takes zero physics, integration, policy,
or controller steps. Six coupled construction rows cover 48/48 samples under
both laws at useful width and a 5 ms CPU gate. A representative 32-step/free-
acceleration row fits angular/linear/joint width 0.095/0.030/6.131 and
0.449/0.065/9.320 at 0.463/0.634 ms p99. All calls repeat bitwise, allocate
nothing in timed Rust, and 192 non-timing arrays reproduce exactly. The typed
mechanisms stay, but no empirical profile, selector, or authority is promoted
and no fresh holdout is spent. The reduced integration schemes preserve the
authored law's explicit/implicit character but do not reproduce MuJoCo's
generalized RK4 or implicitfast integrators. Next, freeze a convergence/profile
rule independently of these labels before a new untouched holdout. See the
[r226 positive-reference compliance audit](benchmarks/results/g1-positive-reference-compliance-audit-r226/G1_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md).

R227 freezes the coupled construction profile without using completed contact
impulse or residual labels for selection. Its causal convergence rule chooses
the least work whose prediction changes by at most 2% of the useful-width gate
when projection sweeps double and 20% when microsteps double, while retaining a
5 ms CPU deadline. That selects 32 microsteps × 32 forward/reverse sweeps:
1.200% sweep refinement, 16.858% substep refinement, and 0.637 ms worst-law
p99. The label-free impulse cap is whole-model mass times maximum causal
closing speed plus one tick of gravity; tangent capacity comes from authored
friction. Two retained runs reproduce all 192 semantic arrays exactly. This
freezes a construction profile for a new untouched holdout only—no selector,
plant action, or authority is promoted. See the [r227 causal convergence audit](benchmarks/results/g1-coupled-positive-reference-compliance-audit-r227/G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md).

R228 freezes that entire profile before generating 96 reset-every-transition
labels at untouched offsets 90,000/100,000. The cross-factor
soft/pyramidal/Euler law passes 48/48; the stiff/elliptic/implicit-fast law
covers only 42/48 (98.9943% of components). All six misses activate additional
near-simultaneous reduced-model foot points that the reference plant never
loads. The frozen width remains 0.449/0.065/9.320 and query p99 is
0.612/0.418 ms, but fitting the stiff misses would require
1.926/0.338/46.976. All 46 non-timing arrays replay exactly. Retain the generic
coupled solver, reject the profile without retuning, and move the next
construction to contact-activation/order plus state-dependent geometry and
Delassus evolution. See the [r228 fresh holdout](benchmarks/results/g1-coupled-positive-reference-compliance-holdout-r228/G1_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT.md).

R229 uses the completed R228 labels only to localize those six rejected stiff
rows; it is explicitly ineligible for construction, selection, or authority.
All six misses predict extra loaded foot points, but predicted-only points also
occur in 8/42 covered stiff rows and 13/48 covered soft rows. Their summed
impulse happens to separate this spent corpus (0.176 N·s maximum covered versus
0.241 N·s minimum uncovered), but that observation is not promoted as a
threshold. A label-oracle mask repairs four misses and worsens the two
multi-contact cases, reaching only 46/48. The missing mechanism is therefore a
causal coupled re-solve as activation, geometry, velocity, and Delassus response
evolve—not a contact mask. A separate label-free first-impact construction uses the
authored five 1 ms ticks: it is a subset of eventual MuJoCo load in 6/6 misses
and exact in 5/6, so it can seed event evolution but cannot predict the final
contact set. All ten semantic arrays, normalized metrics, and the report replay
exactly. See the [r229 activation localization](benchmarks/results/g1-stiff-contact-activation-localization-r229/G1_STIFF_CONTACT_ACTIVATION_LOCALIZATION.md).

R230 moves the event/state loop into allocation-free Rust. The query owns the
floating state, five authored 1 ms collision ticks, explicit sphere support
radius, refreshed geometry/point velocity/convective acceleration, floating
inverse dynamics, the complete Delassus response, and evolved state output.
Collision membership is sampled once at the beginning of each state tick; one
compliant update belongs to that tick, while projection sweeps remain the only
inner convergence knob. Prediction-only double-sweep convergence selects 32
sweeps at 1.619% of the useful-width gate and about 0.70 ms p99. Scored only
after selection, the spent R228 corpus reaches 48/48 exact stiff active sets
and fitted combined width 0.168/0.023/9.893. The profile is frozen for a fresh
holdout, not authority; 106 non-timing arrays replay exactly. See the [r230 model-coupled construction audit](benchmarks/results/g1-model-coupled-positive-reference-compliance-audit-r230/G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_AUDIT.md).

R231 then spends new medium/pyramidal/implicit-fast and hard/elliptic/RK4 laws
at untouched offsets 110,000/120,000. The medium law has exact active sets
48/48 but covers 47/48 under the frozen residual box. The hard RK4 law covers
only 24/48 and misses 22 actual contact points; fitted width expands to
1.894/0.247/62.325. Timing, repeat, and zero timed allocation pass, but strict
coverage is conjunctive, so the profile is rejected without tuning. This
localizes the next mechanism to a real four-stage generalized RK4
dynamics/contact integrator; the reduced trapezoidal point update is not one.
All 60 non-timing arrays replay exactly.
See the [r231 fresh model-coupled holdout](benchmarks/results/g1-model-coupled-positive-reference-compliance-holdout-r231/G1_MODEL_COUPLED_POSITIVE_REFERENCE_COMPLIANCE_HOLDOUT.md).

R232 replaces the model path's reduced point-only RK4 proxy with four genuine
generalized state/contact stages. The authored initial acceleration is mapped
to a held generalized force; every stage refreshes floating bias/inverse
dynamics, the factored mass matrix, complete Delassus response, sphere support
geometry, point motion, and collision membership. Classical RK4 weights
advance pose, tangent, and contact impulse without hot-path storage growth.
Direct Rust tests cover analytic constant-force motion and a contact that
crosses the plane between Euler event ticks. On spent R231 data, hard-RK4 exact
active sets improve 24/48→45/48 and missed actual contacts fall 22→1.

R232's selection view contains causal predictions only. The old 32 sweeps miss
the predeclared 2% refinement gate at 2.496%; 64→128 changes only 0.600% of the
useful-width gate, so 64 sweeps freeze at 4.34 ms p99 with bitwise repeat and
zero timed Rust allocation. Completed R231 impulses and residuals are opened
only after that choice. The spent score compares the returned evolved tangent
directly with the reference final tangent, reaching 46/48 coverage and
0.669/0.125/8.262 fitted width; the obsolete final-impulse-through-initial-
response proxy had 28.584 joint width. An independent rerun reproduces all 59 non-timing
arrays exactly. See the [r232 generalized RK4 convergence audit](benchmarks/results/g1-generalized-rk4-convergence-audit-r232/G1_GENERALIZED_RK4_CONVERGENCE_AUDIT.md).

R233 then spends two new RK4 law/state families at offsets 130,000/140,000.
Medium/elliptic achieves 46/48 frozen coverage and 47/48 exact active sets;
hard/pyramidal achieves 44/48 and 44/48. Both pass repeat, zero-allocation, and
the 5 ms deadline at 3.32–3.38 ms p99, but fitted angular/linear/joint widths
remain 0.285/0.043/10.446 and 0.699/0.110/11.248. The mechanism is retained,
while the profile and authority are rejected without retuning. Because strict
coverage fails, this profile does not enter R224 terminal selection or plant
non-regression. An independent rerun reproduces all 62 non-timing arrays
exactly. See the [r233 fresh generalized RK4 holdout](benchmarks/results/g1-generalized-rk4-holdout-r233/G1_GENERALIZED_RK4_HOLDOUT.md).

R236 opens those completed labels for localization only. Direct final-tangent
error remains 10.446 rad/s on the medium law even when restricted to its 47/48
exact-active-set rows; the hard law's exact-set subset still needs 10.618
rad/s. Worst coordinates are exclusively ankle pitch/roll. The fixture and
predictor already use the same four 5 mm spheres per foot, while right-foot
normal-impulse RMSE reaches 0.730/1.957 N·s and pitch-moment RMSE reaches
0.0311/0.0802 N·m·s for medium/hard. This localizes the next construction to
stage-local force and within-foot wrench distribution/reference constraint-
solver semantics, not another scalar contact mask. R236 is ineligible for
construction selection or authority and runs zero new physics/policy/
controller/selector/plant steps. See the [r236 final-tangent localization](benchmarks/results/g1-rk4-final-tangent-localization-r236/G1_RK4_FINAL_TANGENT_LOCALIZATION.md).

R237 keeps historical generalized RK4 on model ABI id 3 and adds model-only
current-stage-force id 4. Id 4 cancels the local contact solver's extra full-
tick gap advance before every RK derivative, while retaining stage-local
geometry, dynamics, Delassus, motion, collision membership, and caller-owned
scratch. Selection sees only causal R233 arrays: 64→128 refinement is 0.801%
of the useful-width gate, freezing 64 sweeps at 4.06 ms selected p99 (4.18 ms
at 128 sweeps) with bitwise repeat and zero timed Rust allocation. A retained run reproduces 122 semantic
arrays exactly. See the [r237 current-stage-force convergence audit](benchmarks/results/g1-rk4-stage-force-convergence-audit-r237/G1_RK4_STAGE_FORCE_CONVERGENCE_AUDIT.md).

R238 spends new mixed-integrator laws and offsets 150,000/160,000 only after
that freeze. The RK4/id-4 row passes strict 48/48 coverage at
0.122/0.018/5.191 angular/linear/joint width and 2.82 ms p99. The
implicitfast/id-1 row covers 46/48 and needs 0.213/0.045/21.255, rejecting the
conjunctive profile. Both rows meet deadline/repeat/allocation gates and all 62
semantic arrays replay exactly. Retain the stage-force mechanism, admit no
authority, and isolate the remaining implicitfast transfer tail without
fitting these labels. See the [r238 fresh cross-integrator holdout](benchmarks/results/g1-rk4-stage-force-cross-integrator-holdout-r238/G1_RK4_STAGE_FORCE_CROSS_INTEGRATOR_HOLDOUT.md).

R239 then checks the reference integrator equations rather than inferring
contact semantics from an enum name. MuJoCo excludes constraint forces
`Jᵀf(v)` from implicit/implicitfast force-velocity derivatives, so local
implicit contact damping was the wrong mapping. On spent R238 rows, diagnostic
ABI 0 removes one predicted-only activation and cuts fitted joint width from
21.255 to 5.077 rad/s; one exact-set distribution miss remains. The diagnostic
uses zero new reference steps and replays 33 semantic arrays exactly. See the
[r239 constraint-RHS localization](benchmarks/results/g1-implicitfast-constraint-rhs-localization-r239/G1_IMPLICITFAST_CONSTRAINT_RHS_LOCALIZATION.md).

R240 adds a separate equation-level mapper without changing historical replay:
non-RK constraint RHS uses ABI 0 and RK4 uses current-stage ABI 4. Causal-only
128→256 refinement is 1.980% of the width gate, freezing 128 sweeps at 3.34 ms
selected p99 with bitwise repeat, zero timed Rust allocation, and 136 exact
semantic arrays. See the [r240 constraint-RHS convergence audit](benchmarks/results/g1-constraint-rhs-convergence-audit-r240/G1_CONSTRAINT_RHS_CONVERGENCE_AUDIT.md).

R241 crosses the previous cone/integrator pairing on new offsets
170,000/180,000. Elliptic RK4/id 4 covers 48/48 at
0.100/0.009/3.120 angular/linear/joint width and 3.56 ms p99. Pyramidal
implicitfast/id 0 covers 47/48 and needs 0.124/0.025/17.138 after predicting
three unloaded points in its lone miss. Both pass deadline/repeat/allocation
and all 62 semantic arrays replay exactly; the conjunctive profile and
authority remain rejected. See the [r241 fresh constraint-RHS holdout](benchmarks/results/g1-constraint-rhs-cross-integrator-holdout-r241/G1_CONSTRAINT_RHS_CROSS_INTEGRATOR_HOLDOUT.md).

The core also exposes an opt-in model-only ABI 5 (`generalized-implicit-stage-force`)
for isolating a different hypothesis: one local implicit contact update at each
current generalized-RK stage. It is covered by atomicity and positive-impulse
regressions, but has no fresh holdout and is not a production mapping; historical
implicitfast ABI 1 and all prior evidence remain unchanged.

R242 isolates the opt-in ABI 6 predicted-gap activation rule. It admits a
positive-gap contact only when the one-state-step prediction crosses the plane
while the current normal velocity is closing. On the spent R241 replay it
produces 19 extra unloaded contacts, 77.083% coverage, and 1.478/0.275/22.719
fitted angular/linear/joint width, so it is rejected. R243 freezes the first
edge-coordinate profile at 64 sweeps using causal refinement only. See the [r242 activation localization](benchmarks/results/g1-predicted-gap-activation-localization-r242/G1_PREDICTED_GAP_ACTIVATION_LOCALIZATION.md) and [r243 pyramid-edge audit](benchmarks/results/g1-pyramid-edge-convergence-audit-r243/G1_PYRAMID_EDGE_CONVERGENCE_AUDIT.md).

R244 is the historical untouched follow-up for that first edge candidate. Its
implicitfast row covers 47/48 and its RK4 row exceeds five milliseconds, so it
remains immutable failed evidence. See the [r244 pyramid-edge holdout](benchmarks/results/g1-pyramid-edge-cross-integrator-holdout-r244/G1_PYRAMID_EDGE_CROSS_INTEGRATOR_HOLDOUT.md).

R245 corrects the cross-profile selection by forcing edge coordinates for both
integrators and freezing work independently: ABI 0 selects 64 sweeps and ABI 4
selects 32. No completed labels participate. The Rust model also separates
sphere gap geometry from friction kinematics: centre-minus-radius owns the
gap, while the instantaneous surface material point includes `ω × r`; contact
`aref` uses the current collision-boundary gap.

R246 is the untouched confirmation on two new pyramidal laws and offsets
230,000/240,000. Both rows cover 48/48 with exact active sets. Implicitfast
needs 0.059/0.007/4.949 angular/linear/joint width at 0.871 ms p99; RK4 needs
0.065/0.010/2.239 at 3.399 ms. Both meet the five-millisecond,
zero-allocation, bitwise-repeat, and strict component gates; an independent
process reproduces all 62 semantic arrays. The evaluator profile is promoted,
but plant action and authority remain disabled. See the [r245 per-integrator profile audit](benchmarks/results/g1-pyramid-edge-cross-profile-audit-r245/G1_PYRAMID_EDGE_CROSS_PROFILE_AUDIT.md) and [r246 fresh surface-material holdout](benchmarks/results/g1-surface-material-cross-integrator-holdout-r246/G1_SURFACE_MATERIAL_CROSS_INTEGRATOR_HOLDOUT.md).

R199 tests a broader causal pre-step boundary against the measured impulse and
velocity-jump targets localized by r197. Features contain only current root
twist, joint position/velocity, selected action, and the center/spread of the
four support-mode acceleration predictions; physical support, post-step state
or impulse, case identity, and future dropout duration are excluded. Strict
leave-one-named-case-out rejects every tested bound over 266 samples. A global
maximum reaches 99.624% complete-sample coverage only by permitting roughly
72 rad/s of five-millisecond velocity-jump error, and still misses the
left/drop5-matched transition by 1.536 rad/s. A fixed 25% reserve still misses
normal impulse. Action, kNN, Lipschitz, and physical-support oracle rows cover
97.744%, 75.564%, 90.602%, and 93.609%. No empirical bound enters authority.
See the [r199 causal impulse holdout audit](benchmarks/results/upkie-causal-impulse-holdout-r199/UPKIE_CAUSAL_IMPULSE_HOLDOUT.md).

R200 tests the next causal witness directly: exact pre-solve MuJoCo contact
availability, signed wheel/ground distance, and signed constraint-coordinate
relative velocity at the selection tick. Frozen linear and signed-log encodings
are appended to the unchanged R196 feature, with the same six-case
train/seventh-case holdout over 266 samples. They do not help. kNN coverage
regresses from 85.338% to 84.962%; Lipschitz remains 91.353%, misses by 15.086,
and increases its p95 maximum-component bound from 37,977 to 39,752. Nineteen
queries are already in flight, and the dominant contact starts later within the
following 5 ms, so current contact state cannot expose it. The full semantic
result repeats exactly. This exact simulator field remains a candidate-sensor
oracle; no observation contract, residual bound, or authority is admitted. See
the [r200 exact contact-prestate conditioning audit](benchmarks/results/upkie-contact-prestate-conditioning-r200/UPKIE_CONTACT_PRESTATE_CONDITIONING.md).

R197 instruments the missing plant boundary without feeding simulator truth
back into authority. Python accumulates each wheel's normal/tangential contact
impulse and the generalized constraint impulse over all five 1 ms MuJoCo
substeps, while the online Rust chooser and replay state remain unchanged. On
the same 266 valid selections, measured physical support is already the
minimum-error discrete hypothesis 91.35% of the time, yet the raw four-support
qdd box covers 0/266 realized intervals. Even the best support mode has
223.257/14,718.472 p95/max qdd-error norm, and tangential impulse correlates
`+0.907` with that residual. This localizes the next model to slip/contact
impulse realization rather than adding more discrete support labels. The
measurement is noncausal, diagnostic-only, and grants no authority. See the
[r197 contact-impulse residual audit](benchmarks/results/upkie-contact-impulse-residual-audit-r197/UPKIE_CONTACT_IMPULSE_RESIDUAL_AUDIT.md).

R196 asks whether a calibrated residual can repair r195 without leaking the
plant into authority. Six models are trained on six named cases and evaluated
on the seventh. Deployable features contain only current terminal state,
bounded hip/knee posture, normalized joint velocity, action/effort, envelope
pressure, and spread across the four support hypotheses. None passes strict
holdout. A global maximum reaches 99.25% sample coverage but needs a 267.523
pressure bound; a declared 25% reserve reaches 99.62% and still misses the left
holdout. State-neighbour coverage falls to 85.34%. A Lipschitz construction
still covers only 91.35% while its p95 maximum-component bound explodes to
37,977.373. Even oracle physical-support/action conditioning covers only
94.36%. The dominant miss is left/drop5-matched tick 700: physical support
changes flight→double→right, measured wheel acceleration reaches 14,862.6
rad/s² versus 129.8 predicted, and joint-velocity pressure exceeds the global
training bound by 114.425. The next representation must be contact-impulse or
momentum aware rather than a smooth fixed-contact acceleration residual. See
the [r196 residual-conditioning audit](benchmarks/results/upkie-terminal-residual-conditioning-r196/UPKIE_TERMINAL_RESIDUAL_CONDITIONING.md).

R198 moves R197's impulse signal to the causal side of the tick boundary. The
immediately preceding 5 ms normal/tangential/generalized constraint impulse is
appended to the frozen R196 state feature using fixed linear and log scales;
the current interval and next support never enter training. Strict
leave-one-named-case-out evaluation shows no gain: kNN remains exactly 85.34%,
and Lipschitz remains exactly 91.35% while its maximum miss worsens
15.284→16.151 and its p95 bound remains about 38,000. The same left-case
flight→double→right and double→left→flight transitions dominate. Lagged force
history is therefore not a transition tube and gains no authority. See the
[r198 lagged-impulse conditioning audit](benchmarks/results/upkie-lagged-impulse-conditioning-r198/UPKIE_LAGGED_IMPULSE_CONDITIONING.md).

Reproduce it with:

```bash
./scripts/run-upkie-disturbance-envelope.sh
./scripts/run-upkie-planar-capture-ab.sh
./scripts/run-upkie-state-local-authority.sh
./scripts/run-upkie-fall-safe-contingency.sh
./scripts/run-upkie-support-contingency-admission.sh
./scripts/run-upkie-support-contingency-plant-ab.sh
./scripts/run-upkie-support-contingency-primary-preservation-ab.sh
./scripts/run-upkie-guarded-flight-contingency-plant-ab.sh
./scripts/run-upkie-contact-program-authority-plant-ab.sh
./scripts/run-upkie-contact-program-robustness-ab.sh
./scripts/run-upkie-inexact-observation-hold-ab.sh
./scripts/run-upkie-inexact-hold-authority-sweep.sh
./scripts/run-upkie-inexact-observation-duration-oracle-ab.sh
./scripts/run-upkie-inexact-hold-forecast-selector-ab.sh
./scripts/run-upkie-inexact-hold-improvement-gate-ab.sh
./scripts/run-upkie-inexact-support-free-brake-ab.sh
./scripts/run-upkie-inexact-terminal-chooser-ab.sh
./scripts/run-upkie-terminal-realization-calibration.sh
./scripts/run-upkie-inexact-zero-effort-baseline-ab.sh
./scripts/run-upkie-zero-effort-realization-calibration.sh
./scripts/run-upkie-inexact-support-hypothesis-envelope-ab.sh
./scripts/run-upkie-contact-impulse-residual-audit.sh
./scripts/run-upkie-lagged-impulse-conditioning.sh
./scripts/run-upkie-causal-impulse-holdout.sh
./scripts/run-upkie-contact-prestate-conditioning.sh
./scripts/run-upkie-contact-transition-interval-audit.sh
./scripts/run-upkie-contact-transition-response-audit.sh
./scripts/run-upkie-contact-transition-acceleration-interval-audit.sh
./scripts/run-upkie-directional-contact-transition-audit.sh
./scripts/run-upkie-grouped-acceleration-reachable-set-replay.sh
./scripts/run-g1-contact-law-transition-holdout.sh
./scripts/run-g1-kinetic-impulse-ellipsoid-holdout.sh
./scripts/run-g1-spatial-patch-transition-holdout.sh
./scripts/run-g1-causal-center-split-kinetic-replay.sh
./scripts/run-g1-coupled-contact-law-replay.sh
./scripts/run-g1-coupled-contact-law-holdout.sh
./scripts/run-g1-contact-estimator-hypothesis-replay.sh
./scripts/run-g1-substepped-compliant-contact-replay.sh
./scripts/run-g1-substepped-compliant-contact-holdout.sh
./scripts/run-g1-compliant-terminal-consequence-audit.sh
./scripts/run-g1-terminal-velocity-box-audit.sh
./scripts/run-g1-positive-reference-compliance-audit.sh
./scripts/run-g1-coupled-positive-reference-compliance-audit.sh
./scripts/run-g1-coupled-positive-reference-compliance-holdout.sh
./scripts/run-g1-stiff-contact-activation-localization.sh
./scripts/run-g1-model-coupled-positive-reference-compliance-audit.sh
./scripts/run-g1-model-coupled-positive-reference-compliance-holdout.sh
./scripts/run-g1-pyramid-edge-cross-profile-audit.sh
./scripts/run-g1-surface-material-cross-integrator-holdout.sh
./scripts/run-upkie-terminal-residual-conditioning.sh
./scripts/run-upkie-observation-delay-startup-ab.sh
./scripts/run-upkie-observation-dropout-phase-ab.sh
./scripts/run-upkie-stale-command-expiry-ab.sh
./scripts/run-upkie-contact-truth-authority.sh
./scripts/run-upkie-contact-observation-contract.sh
./scripts/run-upkie-contact-observed-controller-ab.sh
./scripts/run-upkie-reduced-support-command-gate-ab.sh
./scripts/run-upkie-contact-unload-precursor.sh
./scripts/run-upkie-contact-command-lease-ab.sh
./scripts/run-upkie-contact-command-freshness-composition.sh
./scripts/run-upkie-state-local-viability-descent.sh
./scripts/run-upkie-viability-coordinate-planner.sh
./scripts/run-upkie-viability-request-supervisor.sh
./scripts/run-upkie-viability-request-plant-ab.sh
./scripts/run-upkie-conditional-viability-planner-ab.sh
./scripts/run-upkie-budgeted-multistep-viability-ab.sh
./scripts/run-upkie-lateral-budgeted-multistep-viability-ab.sh
./scripts/run-upkie-lateral-anytime-multistep-viability-ab.sh
./scripts/run-upkie-multistep-viability-plant-ab.sh
./scripts/run-upkie-paired-multistep-viability-plant-ab.sh
./scripts/run-upkie-confirmed-multistep-viability-plant-ab.sh
./scripts/run-upkie-hybrid-guard-viability-plant-ab.sh
./scripts/run-upkie-forecast-realization-audit.sh
./scripts/run-upkie-forecast-realization-contract.sh
./scripts/run-upkie-conditioned-forecast-certificate.sh
./scripts/run-upkie-paired-state-forecast-certificate.sh
./scripts/run-upkie-execution-residual-monitor.sh
./scripts/run-upkie-execution-residual-veto-plant-ab.sh
./scripts/run-upkie-bounded-feasibility-timing-ab.sh
./scripts/run-upkie-equality-first-feasibility-plant-ab.sh
./scripts/run-upkie-planner-continuation-plant-ab.sh
./scripts/run-upkie-signed-zero-sparse-feasibility-ab.sh
./scripts/run-upkie-cross-tick-planner-cadence-ab.sh
./scripts/run-upkie-cross-session-feasibility-witness-ab.sh
./scripts/run-upkie-cross-profile-feasibility-witness-ab.sh
```

Progress against the specification is tracked in
[docs/IMPLEMENTATION_STATUS.md](docs/IMPLEMENTATION_STATUS.md).
