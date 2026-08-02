# WBC evaluation design

The evaluation strategy measures local numerical identities, solver semantics,
closed-loop behavior, determinism, and timing separately. A single attractive
animation is not evidence that a WBC is correct.

### 0. Pinocchio differential oracle

The Rust fixture executable evaluates deterministic valid states through the
normal Bonesaw compiled-model path. A separate Python process loads the same
URDF in Pinocchio 4.0, maps every coordinate by joint name and every link by
BODY-frame name, and compares:

- all body-frame translations and rotation matrices;
- all frame Jacobians after explicitly converting Pinocchio's
  `[linear; angular]` convention to Bonesaw's `[angular; linear]`;
- joint-space mass matrix;
- generalized gravity;
- recursive inverse dynamics with nonzero velocity and acceleration;
- full-system CoM, centroidal map, and centroidal momentum;
- floating-root mass matrix, bias force, inverse dynamics, and centroidal map
  after explicit tangent-order and classical/spatial-acceleration conversion.

The strict runner is:

```bash
./scripts/run-pinocchio-oracle.sh --model models/upkie/upkie.urdf --samples 50
```

The oracle is intentionally outside `bonesaw-core`; Pinocchio is a
development verifier, not a runtime dependency.

### 0b. PlaCo reference WBC

The Python evaluation layer generates one exact NumPy corpus and runs Bonesaw
and PlaCo in isolated child processes against the same toy URDF, standing pose,
target timestamps, frame names, and coordinate limits. Bonesaw executes each
chunk wholly inside Rust and writes q/v, tracked positions, status, and native
per-step timing into caller-owned arrays. PlaCo uses its independent C++ QP
kinematics solver; both end-to-end and solver-only timing are retained.

```bash
./scripts/run-reference-comparison.sh --ticks 5000 --warmup 250
```

The report includes full percentile distributions through p99.99, absolute
jitter, deadline misses, tracking error/IAE/ISE, ten execution-time windows,
RSS/USS/PSS/VMS, page faults, context switches, CPU/wall ratios, Python
`tracemalloc`, and GC collections. Raw traces and exact inputs are in
`benchmarks/results/reference-latest`.

The formulations are intentionally not identical: Bonesaw preserves strict
lexicographic levels while PlaCo uses weighted soft tasks. The comparison is a
regression instrument, not an assertion that their commands should match.

### Revision r89 fresh trace audit

R89 reruns PlaCo, Pinocchio, upstream Upkie, and Bonesaw in one session on the
same Ryzen 7 3700X host, retaining raw per-step traces rather than reusing the
r38 timing. Each fixed-base implementation receives 250 warmup ticks and 5,000
measured ticks for reach, bimanual conflict, and CMU walking retargeting.
Within that shared boundary, PlaCo/Bonesaw p50 latency is `2.75×`, `1.51×`, and
`2.36×`; peak process RSS is `2.19×`, `2.08×`, and `2.03×`. Tracking is mixed:
the PlaCo/Bonesaw RMS ratio is `1.28×`, `0.77×`, and `0.65×`, respectively.
Bonesaw therefore tracks reach better while PlaCo tracks the latter two better.
Bonesaw misses the walking foot-RMS gate (`5.549 cm > 5 cm`); PlaCo misses the
swing-clearance gate (`3.301 cm > 3 cm`).

The aligned upstream Upkie law has `0/100,000` canonical bit mismatches and the
fresh Pinocchio Upkie/G1 product oracles pass at roundoff. These prove only
their named boundaries. PlaCo's weighted position QP, Bonesaw's fixed-layout
controller, the scalar Upkie control law, and Pinocchio rigid-body products are
not collapsed into a universal WBC ranking. The responsive hosted report
renders seven latency/tracking charts; its machine-readable companion retains
600 one-second fixed-base window rows and hashes every source artifact. Rebuild
it with `scripts/run_reference_trace_report.sh` from
`benchmarks/results/reference-r89`.

### Revision r90 live query timing contract

R88's physical trace was valid, but its `solve_us` timing label was not: the
server summed four sequential 5 ms state-local queries in one streamed 50 Hz
frame and compared that sum with a single-query 5 ms deadline. R90 measures
each query end-to-end, publishes the maximum as `solve_us`, and separately
publishes `query_batch_us` plus `query_count`. Transport, JSON serialization,
and browser rendering remain outside both clocks.

The unchanged 160 mm disturbance retains the same support, stopping, effort,
hard-residual, task-count, limiting-ID, and status evidence. Maximum-query
p50/p95/p99/max is `1.923/2.819/3.076/3.223 ms`, with zero of 60 frames above
5 ms. The sum of four queries is `7.349/8.793/9.148/9.373 ms`, with zero
above 20 ms. Solver-work distributions are retained: task pseudoinverse calls
p50/p99/max are `3/10.41/11`, Jacobi sweeps `11/75.25/90`, and feasibility is
exactly one projection sweep with no polish. A simultaneous 440-sample perf
capture loses no samples and attributes 67.58% of cycles to the dense
pseudoinverse, 12.45% to dense multiply, and 9.58% to the remaining hierarchy.
No iteration, tolerance, task, constraint, or physical gate changed.

### Revision r91 CUDA StateInput boundary

R91 begins Gate 6 only after CPU concept admission. A fixed PTX kernel consumes
`q[coordinate][agent]` and `root_pose[component][agent]`, assigns one thread to
one complete agent, rejects non-finite f32 bit patterns on device, and writes
typed deterministic zeroes for invalid and padded lanes. A dynamically loaded
CUDA Driver API owns one context, JIT module, function, and five fixed device
buffers. There is no CPU fallback and no link-time `libcuda` dependency.

The paired CpuMirrorF32 stage passes 100-call byte replay, reversed-agent
permutation, invalid-agent neighbor isolation, padded-stride zeroing, and zero
allocation. The retained PTX SHA-256 is
`45652bbb5db690174ece50f579fe350527db76e15b580eecf66bbe11d93a567d`.
The local runtime probe reports `no_device`, so PTX JIT, device launch, D1, D2,
device isolation, memory scaling, timing, error injection, and direct-versus-
Graph checks are NOT RUN. `cuda_mirror_state_input_f32` is true as an
implementation capability; full `cuda_mirror_f32` and `cuda_graph_executor`
remain false. Reproduce the typed unavailable report with
`scripts/run_cuda_state_input_report.sh`.

### Revision r92 CUDA tree FK and center-of-mass boundary

R92 extends the independently manifested device implementation through generic
tree forward kinematics and mass-weighted center of mass. Construction packs
topologically ordered fixed/revolute/prismatic joints, fixed transforms, axes,
body masses, and local CoM offsets into immutable f32 constants. The executor
dynamically loads NVRTC and the CUDA Driver API, owns two modules plus fixed
state/model/output buffers, and launches retained StateInput followed by FK/CoM
in one ordered stream. One thread owns a complete agent; no atomic, block
barrier, cross-agent reduction, device allocation, or hidden CPU fallback is
allowed.

The retained audit reports source/ABI policy, CPU mirror evidence, compiler
availability, runtime availability, and device conformance independently. The
CPU witness passes 200-call complete-output byte replay, invalid-agent typing,
exact neighbor isolation, deterministic padded lanes, and the allocation-free
unit sentinel. Source and packed-model SHA-256 identities are retained. This
host reports NVRTC `library_unavailable` and CUDA `no_device`, so generated PTX,
module loading, D1, D2, device isolation, timing, memory, error injection, and
Graph equivalence are NOT RUN. `cuda_mirror_fk_com_f32` identifies implemented
stage code; full `cuda_mirror_f32` and Graph remain false. Reproduce with
`scripts/run_cuda_fk_com_report.sh`.

### Revision r93 CUDA floating-kinematics boundary

R93 extends the same no-fallback, fixed-buffer CUDA pipeline through every
floating frame-origin Jacobian and the center-of-mass Jacobian. The immutable
model pack now includes one parent-joint index per body. One device thread owns
one complete agent and walks only that body's deterministic ancestor chain;
there are no atomics, block barriers, cross-agent reductions, or device
allocations. StateInput, FK/CoM, and Jacobians launch in one ordered stream with
one final synchronization. Rows are frozen as `[angular xyz; frame-origin
linear xyz]`, and generalized tangent columns are `[root angular xyz; root
linear xyz; joints]`.

The retained CPU witness passes 200-call byte replay over the complete
FK/Jacobian output, Pinocchio and finite-difference products, invalid-agent
typing, exact neighbor isolation, deterministic padded lanes, and the
allocation sentinel. Both CUDA sources pass static source/ABI policy. Compiler
and runtime authority remain distinct: this host reports NVRTC
`library_unavailable` for FK/CoM and Jacobians and CUDA `no_device`. Generated
PTX, module loading, device D1 replay, D3 CPU-mirror tolerance, timing, memory,
fault injection, and Graph equivalence are therefore NOT RUN. Implemented
StateInput, FK/CoM, and Jacobian stage capabilities are true; full
`cuda_mirror_f32` and Graph remain false. Reproduce with
`scripts/run_cuda_kinematics_report.sh`.

### Revision r94 CUDA floating-dynamics boundary

R94 extends the same no-fallback, fixed-buffer CUDA pipeline through floating
`M(q)`, `h(q,v,g)`, and `Ag(q)`. The immutable pack adds each body's row-major
inertia about its center of mass. Fixed per-agent generalized-velocity and
gravity inputs feed a zero-acceleration spatial recursion; mass, bias, and
centroidal products are then assembled from the admitted poses and frame
Jacobians in deterministic body/coordinate order. One device thread still owns
one complete agent. StateInput, FK/CoM, Jacobians, and Dynamics launch in one
ordered stream with one final synchronization.

The retained CPU witness passes 200-call complete-pipeline byte replay,
Pinocchio and physical-identity product gates, invalid velocity typing, exact
neighbor isolation, deterministic padded lanes, and the allocation sentinel.
All three CUDA sources pass static source/ABI policy. This host reports NVRTC
`library_unavailable` for FK/CoM, Jacobians, and Dynamics and CUDA `no_device`.
Generated PTX, module loading, device D1 replay, the provisional `5e-4`
abs+rel M/h/Ag D3 gate, timing, memory, fault injection, and Graph equivalence
are therefore NOT RUN. Implemented StateInput, FK/CoM, Jacobian, and Dynamics
stage capabilities are true; full `cuda_mirror_f32` and Graph remain false.
Reproduce with `scripts/run_cuda_dynamics_report.sh`.

### Revision r95 CUDA fixed point-product boundary

R95 extends the same one-stream executor through compiler-resolved point world
position, floating point Jacobian, and zero-generalized-acceleration bias
`Jdot-v`. Query stable IDs, frame indices, and local offsets are uploaded once;
position, Jacobian, bias, and typed status storage is fixed at construction.
StateInput, FK/CoM, Jacobians, Dynamics, and PointQueries launch in order with
one terminal synchronization and no hidden CPU fallback.

The retained CPU witness passes 200-call point-product byte replay, f64
position/J/Jdot-v comparison, invalid-neighbor isolation, deterministic padded
lanes, and the allocation sentinel. All four CUDA sources pass static ABI
policy. This host reports NVRTC `library_unavailable` for all four sources and
CUDA `no_device`, so generated PTX, module loading, device D1 replay, the
provisional `2e-4` abs+rel point-product D3 gate, timing, memory, fault injection,
and Graph equivalence are NOT RUN. Implemented point queries do not imply task
emission or solve; full `cuda_mirror_f32` and Graph remain false. Reproduce with
`scripts/run_cuda_point_queries_report.sh`.

### Revision r96 CUDA row-emission boundary

R96 adds a sixth direct-launch module for fixed point-attractor and contact-lock
row emission. Immutable query-slot, bandwidth, and contact-mode metadata is
uploaded at construction; runtime activation masks and target jets are finite
checked per agent. Inactive and mode-disabled rows are exactly zero, and the
whole StateInput-through-Emission chain retains one terminal synchronization.
The retained audit passes five-source policy, 200-call complete emission-byte
replay, a zero-error independent formula oracle, malformed-mask/NaN typing,
invalid-neighbor isolation, deterministic inactive/mode-filtered/padded zeros,
and the allocation sentinel. Because this host has neither NVRTC nor a CUDA
device, generated PTX and device D1/D3 remain NOT RUN. This does not promote the
full CUDA mirror or solver. Reproduce with `scripts/run_cuda_emission_report.sh`.

### Revision r97 fixed-level CPU mirror hierarchy

R97 freezes the algorithm that a later CUDA solve must reproduce without
claiming device execution. `CpuMirrorF32` uses stable descriptor order, 64 hard
projection sweeps, 32 row-action sweeps per active priority, two restoration
sweeps after each task-row update, f32 FMA arithmetic, fixed storage, and no
early exit. It reports initial, best, and final hard residuals, per-level RMS,
priority-preservation drift, clipping, bounds, row counts, and work.

The policy-/physics-free seven-agent corpus covers compatible conflicting soft
levels, contradictory hard rows, invalid bounds, invalid emitted data, a hard
row/bound conflict, an empty problem, and an isolated compatible neighbor.
Complete output repeats byte-for-byte for 500 calls; compatible commands match
`CpuExactF64` exactly in the retained fixture, admitted hard residual and
priority drift are zero, padding is zero, and the hot path allocates zero
bytes. The contradictory hard case reports `MaxIterations` with residual
1.25→1.0→1.0, retains a finite candidate, and leaves the executable command
zero. `MaxIterations` is explicitly not a proof of primal infeasibility.
Reproduce with `scripts/run_cpu_mirror_solve_report.sh`.

### Revision r98 CUDA fixed-level solve boundary

R98 ports the R97 fixed-level hierarchy to a seventh CUDA module without
changing its authority semantics. One full agent per thread executes 64 hard
projection sweeps and 32 sweeps per active priority, with fixed global scratch,
stable descriptor order, no early exit, and no CPU fallback. Fixed outputs
retain executable command, finite best candidate, typed status, hard-residual
progress, per-level RMS and preservation drift, bounds, margins, and work.

The retained source/CPU audit passes and the conditional Rust device test is
compiled. On this host all six NVRTC probes report `library_unavailable` and
the runtime reports `no_device`; generated PTX, module loading, device D1/D3,
memory scaling, timing, isolation, and Graph equivalence are therefore NOT RUN.
This does not admit the full CUDA mirror, actuation, integration, or physical
realization. Reproduce with `scripts/run_cuda_solve_report.sh`.

### Revision r99 integrated floating CPU advance

R99 composes the strict floating `[qdd, generalized effort, point force]` query
with immutable actuation mapping, exact commanded-state splice, independently
validated primary and braking-contingency quintics, typed plan selection, and
one contiguous 20×1 ms actuator block. Observed state remains a caller input;
the transaction never self-integrates acceleration or calls the emitted
command a plant response.

The retained 500-tick Upkie corpus uses two RollingPoint wheel contacts and a
five-cycle acceleration reference. Complete semantic output bytes replay
exactly across fresh sessions; position, velocity, and acceleration splice
errors are zero; maximum hard residual is `5.15e-12`; all 500 primary and 500
contingency segments validate analytically; release p50/p99/max is
`154.477/218.618/252.717 µs`; and the Rust transaction performs zero hot-path
allocations. Observed/commanded divergence is reported explicitly because no
plant closes the loop. Reproduce with `scripts/run_dynamic_advance_report.sh`.

### Revision r100 analytic dynamic admission faults

R100 computes exact quintic position minima/maxima from polynomial roots, maps
actuator coefficients through the immutable generalized-from-actuator
transmission, and validates the resulting joint paths against authored position
limits. Fixed admission flags report solver slack, solver rejection, previous
plan expiry, actuator derivative limits, and primary/contingency joint-position
limits independently.

The policy-/physics-free Upkie corpus repeats nominal, near-left-hip-limit, and
expired-plan cases 100 times each. Nominal selects Primary with 1.260 rad
headroom. The near-limit case reports flags `0x01 | 0x10`, withholds its unsafe
primary, and admits a zero-effort braking contingency at 0.0 rad headroom. The
expired case reports `0x04` and also selects braking. Every case replays semantic
bytes exactly, remains below `5.2e-12` hard residual, allocates zero bytes inside
the transaction, and has maximum measured execution below 190 µs. Reproduce
with `scripts/run_dynamic_admission_fault_report.sh`.

### Revision r101 deterministic dynamic self-collision admission

R101 optionally sweeps both mapped joint polynomials over the compiled
conservative self-collision proxies at the exact 1 ms command grid, including
both endpoints. Collision policy is explicit: disabled validation returns no
fabricated distance evidence, and collision-enabled construction rejects
unsupported authored mesh geometry unless the caller explicitly selects the
represented-subset policy.

The policy-/physics-free one-axis fixture repeats three cases 100 times. The
safe case retains 0.200 m minimum clearance. The faulting primary crosses the
0.020 m requirement at pair 0 and 16 ms, reaches 0.010532 m, activates flag
`0x080`, and selects a zero-effort braking contingency that remains at 0.035 m.
The same command with collision explicitly disabled selects Primary and reports
`Infinity`, not a false measured clearance. Semantic bytes replay exactly,
maximum hard residual is `1.11e-16`, all transaction allocations are zero, and
the retained worst execution is 35.147 µs. Reproduce with
`scripts/run_dynamic_collision_fault_report.sh`.

### Revision r102 conservative continuous-clearance admission

R102 preserves the raw 21-point collision sweep and adds an optional
between-sample certificate over the represented sphere proxies. Construction
precomputes conservative joint-coordinate center-speed coefficients. Analytic
maximum joint velocity over each mapped quintic then bounds pair-relative
motion, and the certified lower bound subtracts half a sample interval of that
motion from the raw sampled minimum.

The adversarial primary stays above the 0.020 m requirement at every sample
(`0.020232 m`) and therefore has no raw violation pair or time. Its maximum
relative center-speed bound is `2.446812 m/s`, lowering continuously provable
clearance to `0.019008 m`. Explicit grid-only policy selects it; conservative
certificate policy emits `0x200` and selects independently certified braking
at `0.044450 m`. The three cases replay exactly for 100 resets, allocate zero
transaction bytes, retain `1.11e-16` maximum hard residual, and stay below
50.055 µs maximum on the retained run. Reproduce with
`scripts/run_dynamic_continuous_clearance_report.sh`.

### Revision r103 live command-authority stream

R103 transports the r101/r102 command evidence into the browser as three
independent authority rows: sampled represented-proxy geometry, conservative
between-sample clearance, and typed Primary/Contingency/Rejected selection with
composable flags. The dynamic adapter now reuses its already-solved WBC output
through `advance_solved_into`; it does not repeat the dense physical solve.

The retained 60-frame, 8 cm torso-pull trace carries 13 unique capability IDs.
Raw-WBC and command-admission maximum query times are 2.246 and 2.191 ms against
separate 5 ms contracts; the four-command-query batch maximum is 8.601 ms
against 20 ms. Maximum dynamics/contact residuals are `1.32e-9`/`4.41e-11`.
All commands are typed Rejected with `0x180` or `0x181`: the current single
bounding-sphere approximations for toy boxes/cylinders overlap pair 0 at offset
zero by roughly 247 mm. That is retained as a red represented-proxy limitation,
not relabeled as exact primitive collision or hidden behind the guided pose.
Reproduce with `scripts/run_live_command_authority_report.sh` while the server
is running.

### Revision r104 tight primitive command admission

R104 replaces the live one-sphere artifact with a separate command-admission
primitive table: exact spheres, conservative capsules around cylinders,
authored capsules, and oriented boxes. Point/segment distance and a
conservative separating-axis box clearance run without per-query allocation.
Primitive speed coefficients include rotational reach. Collisionless
revolute/fixed carrier chains are topology-derived physical adjacency;
prismatic carriers stay in the pair table.

The retained 60-frame, 80 mm torso pull uses a collision-aware reference arm
posture. All 21 × 1 ms grids are clear, with a 20.106 mm sampled minimum. The
continuous lower bound reaches 19.911 mm: 54 queries select Primary and six
select independently certified braking, whose minimum continuous clearance is
20.029 mm. Raw WBC, command, and four-query maxima are 3.839/1.319/5.080 ms;
maximum dynamics/contact residuals are `1.67e-9`/`4.90e-11`. Minimum body-pair
names are retained independently from the first violating pair. Reproduce with
`scripts/run_live_command_authority_report.sh` while the server is running.

### Revision r105 pair-tight continuous command authority

R105 removes a pessimistic cross-product from the continuous certificate. The
dense primitive sweep retains a minimum per stable pair in fixed Rust scratch;
each lower bound subtracts only that pair's compiled relative-speed bound over
half a 1 ms interval. A focused Rust fixture proves the result cannot mix the
closest pair with an unrelated fastest pair. The WebSocket additionally names
the continuous limiting pair and bodies and streams its speed bound.

The retained 60-frame, 80 mm guided trace records 52 Primary, five Contingency,
and three Rejected transactions. Sampled/continuous/braking minima are
19.710/19.601/19.999 mm; seven frames expose chest↔left-forearm sample
violations. This differs from r104 because less-pessimistic primary admission
changes the persistent command-state path; it is evidence that a locally
tighter certificate is not a closed-loop safety guarantee. Raw WBC, command,
and four-query maxima are 2.002/1.242/4.510 ms, with dynamics/contact residuals
at `1.67e-9`/`4.90e-11`. The browser keeps a 120-tick overlay of sampled and
continuous clearance, the requirement, and selection bands. Reproduce with
`scripts/run_live_command_authority_report.sh` against a server launched with
`BONESAW_LIVE_GUIDED=1`.

### Revision r106 bounded adaptive pair/interval clearance

R106 preserves the exact 1 ms grid and uses the r105 pair-consistent result as
a broad gate. Only globally unresolved pairs evaluate analytic velocity extrema
on individual closed intervals. Leaves whose reserve is still below 20 mm get
one deterministic midpoint primitive query and recurse to a construction-time
maximum depth of three. All storage is caller-owned.

The fixed Python depth ladder uses an unchanged 20.232 mm sampled minimum. Its
continuous proof rises 19.008→19.620→19.926→20.079 mm with exactly 0→1→2→3
midpoint pair queries; depths one and two retain one unresolved leaf and depth
three admits Primary. Across 100 command-state resets per depth, semantic bytes
are exact, timed Rust allocation calls/bytes remain zero, and the maximum hard
residual is `1.11e-16`. A separate Rust case has safe base endpoints and a
penetrating midpoint; refinement records stable pair 7 at 0.5 ms as sampled
collision rather than certifying around it.

The retained 60-frame, 80 mm live trace selects 55 Primary, four Contingency,
and one Rejected. Five sampled chest↔left-forearm failures remain visible and
hard-stop adaptive work. Clear frames in this trace pass the broad certificate
without midpoint work. Raw WBC, command, and four-query maxima are
3.701/2.336/5.396 ms; dynamics/contact residuals remain
`1.67e-9`/`4.90e-11`.

### Revision r107 shared tight-primitive avoidance

R107 removes the fixed-root representation split introduced in r104. The
supported primitive table now produces one allocation-free witness containing
signed distance, stable pair ID, deterministic normal, closest surface points,
locally selected rigid-body feature points, relative normal velocity, analytic
Jacobian, and distance quality. Fixed-root avoidance consumes that witness;
command admission consumes the same scalar routine and pair table. The former
sphere-cover emitter remains available only through explicitly named fallback
methods. Box-box continues to label its separating-axis lower bound as
conservative.

The Python-owned fixed corpus sweeps 321 states of a box–sphere fixture without
policy or physics. Tight distance matches the analytic oracle to `2.78e-17 m`,
and its Jacobian matches finite differences to `5.84e-10`. One tight pair
replaces three cover pairs, removes 131.046 mm maximum overreach, and eliminates
132 false influence plus 122 false hard-collision samples. Across 1,000 repeats,
the tight query records 0.311/0.391/6.181 µs p50/p99/max versus
0.501/0.571/0.751 µs for the fallback; both replay exactly with zero timed Rust
allocation calls/bytes.

The retained report is
`benchmarks/results/tight-primitive-avoidance-r107/TIGHT_PRIMITIVE_AVOIDANCE_AUDIT.md`.
R108 supplies the corresponding local floating barrier and live typed row; the
R107 report remains the geometry-only admission boundary.

### Revision r108 floating closest-feature collision barrier

R108 feeds the R107 tight witness into the floating CPU WBC as
`J qdd + bias + 2 ζ ω hdot + ω² h >= 0`, with `h` equal to signed distance
minus the hard margin. Python supplies 321 independent observed states and an
enabled/disabled counterfactual; Rust owns FK, floating dynamics, closest
features, hard-row assembly, solve, and typed evidence. No state is integrated
and no policy, contact response, or external physics is involved.

Seventy states activate the row. Distance, relative velocity, and required
normal acceleration match the analytic box–sphere oracle within `2.78e−17 m`,
`0`, and `5.33e−15 m/s²`; minimum post-solve barrier residual is
`−2.22e−16 m/s²`. The barrier changes inward joint acceleration by up to
`62.171 m/s²`. The retained query replays exactly and allocates zero calls or
bytes. Closest-feature switching and normal curvature are deliberately outside
this local barrier, so R106 sampled/adaptive command admission remains a
separate required gate. Rebuild with
`scripts/run_floating_collision_barrier_report.sh`; the audit is retained at
`benchmarks/results/floating-collision-barrier-r108/FLOATING_COLLISION_BARRIER_AUDIT.md`.

The separate 60-frame guided editor audit retains 13 active rows, zero
unsupported shapes, pair 12 (`left_thigh↔right_thigh`) as both closest and
limiting, 50 mm minimum clearance, 4.737 m/s² minimum barrier residual, and
2.011/3.266/3.359 ms WBC p50/p99/max. Its separately typed command certificate
reaches 19.601 mm; neither value substitutes for the other. See
`benchmarks/results/live-collision-barrier-r108/LIVE_COLLISION_BARRIER_AUDIT.md`.

### Revision r109 immutable world-SDF barrier

R109 adds a second collision authority path for finite world geometry. Rust
owns an immutable x-fastest dense grid, rotated-grid sampling, analytic
trilinear gradients, deterministic conservative sphere probes, floating body
point Jacobians and `Jdot-v`, hard-row assembly, solve, and typed evidence.
Python owns 321 observed states plus independent NumPy interpolation, gradient,
velocity, and HOCBF oracles. No policy, simulator, integration, or feedback is
present.

The retained audit activates 193 states across four probes. Maximum distance,
velocity, gradient-norm, and required-acceleration oracle errors are
`2.22e−16 m`, `5.55e−16 m/s`, `1.33e−15`, and `4.97e−14 m/s²`; minimum
post-solve residual is `−3.55e−15 m/s²`. Enabled/disabled joint acceleration
differs by up to `82.281 m/s²`. Exact replay and zero timed allocation
calls/bytes pass. `Reject` withholds outside queries. `OccupiedBoundary` makes
the finite known-volume boundary solid, reports negative distance outside, and
does not admit the deep-outside candidate. Rebuild with
`scripts/run_world_sdf_barrier_report.sh`; the audit is retained at
`benchmarks/results/world-sdf-barrier-r109/WORLD_SDF_BARRIER_AUDIT.md`.

The 60-frame live audit retains seven active probes, zero unsupported shapes,
the left hand as closest/limiting body, a trilinear unit gradient, explicit
`Reject` policy, 66.409 mm minimum clearance, 7.329 m/s² minimum residual, and
1.982/3.111/3.175 ms WBC p50/p99/max. The separately typed self-collision
command certificate reaches 19.601 mm. The visible wall and world row do not
turn the guided pose into realized response. See
`benchmarks/results/live-world-sdf-r109/LIVE_WORLD_SDF_AUDIT.md`.

The local barrier deliberately excludes the SDF Hessian and voxel-feature
switching. Swept command validation against the world field, mutable scene
epochs, dynamic obstacles, richer mesh probes, and plant response remain
separate gates.

### Revision r110 swept world-SDF command admission

R110 extends the exact policy-/physics-free dynamic transaction rather than
adding a plant. Rust owns WBC, actuation mapping, prior-command splicing,
primary/brake quintics, 21-knot SDF scans, bounded continuous certificates,
typed selection, and fixed-capacity scratch. Python independently reconstructs
both quintics and scores sampled distance, earliest violation time, analytic
rate bounds, dense continuous truth, boundary policies, timing, allocation,
and replay.

Across 321 transactions, sampled distance and continuous-lower-bound errors are
both at most `2.78e−16 m`, violation-time error is `0 ns`, and rate-bound error
is `3.11e−15 m/s`. The corpus selects 317 primary, three contingency, and one
reject. A targeted safe segment moves from a pessimistic `19.285 mm` broad
certificate to a proven `20.061 mm` with two midpoint probe samples at depth
two; dense truth is `20.319 mm`. Reject-policy field exit transfers authority
to a known-clear brake, while OccupiedBoundary yields a measured collision and
rejects both plans. The complete WBC + actuation + two-plan world transaction
measures `15.855/22.003/30.287 µs` p50/p99/max across 2,000 exact replays with
zero timed allocation calls/bytes. Rebuild with
`scripts/run_world_sdf_command_report.sh`; the report is retained at
`benchmarks/results/world-sdf-command-r110/WORLD_SDF_COMMAND_AUDIT.md`.

The 60-frame live audit exposes 17 capabilities. Local HOCBF clearance remains
`66.409 mm`; primary/brake swept-world certificates also remain at least
`66.409 mm`, with 1,840 broad-certified leaves and no unresolved intervals.
Self-collision independently produces 55 primary, four contingency, and one
reject selections. Command-admission p99 is `2.499 ms` and raw-WBC p99 is
`3.014 ms`. See
`benchmarks/results/live-world-sdf-command-r110/LIVE_WORLD_SDF_AUDIT.md`.

The current proof holds `control_world_from_root` fixed across each 20 ms joint
segment and uses conservative sphere probes. Mutable scene epochs, dynamic
obstacles, predicted root sweeps, exact mesh CCD, calibrated realization, and
measured plant response remain separate gates.

### Revision r111 floating-root swept world-SDF admission

R111 removes that fixed-root qualification without turning the free base into
an actuator. Rust pairs each primary/brake actuator segment with a fixed-size
six-DoF local tangent quintic anchored at the observed pose in smooth
`control_world`. The primary witness uses observed twist plus solved root
acceleration; the contingency witness independently brings observed twist to
rest over the same 20 ms horizon. Both are state-local collision predictions,
not commands, policy rollouts, or realized motion.

World sampling evaluates the root and joint segments at the same 21 base knots
and every adaptive midpoint before FK. The continuous proof adds analytic root
twist extrema to joint extrema; angular root motion is converted to conservative
probe speed with a compiled maximum reach over authored joint ranges. The
fixed-shape PyO3 trace exposes primary/brake endpoint translation, rotation
vector, endpoint twist, and maximum absolute twist.

The independent NumPy oracle reconstructs both root witnesses across 321
transactions. Maximum root-witness error is `1.51e−18`; sampled distance and
continuous lower-bound errors are `2.78e−16 m`; violation-time error is `0 ns`;
and rate-bound error is `4.00e−15 m/s`. A discriminating case is `23.000 mm`
clear if the root is incorrectly frozen, but the primary root prediction
reaches `17.500 mm`, first violates at 11 ms, and transfers to an independently
valid `20.250 mm` brake. Adaptive clearance proves `20.106 mm` against
`20.250 mm` dense truth with three midpoint probes. Semantic replay is exact
and the timed Rust transaction allocates zero bytes. Reproduce with
`scripts/run_world_sdf_command_report.sh`; the report is retained at
`benchmarks/results/world-sdf-root-command-r111/WORLD_SDF_COMMAND_AUDIT.md`.

The live WebSocket contract now has 18 capabilities and renders floating-root
prediction separately from local world HOCBF, sampled world geometry,
continuous world clearance, selection, and unavailable realization. See
`benchmarks/results/live-world-sdf-root-command-r111/LIVE_WORLD_SDF_AUDIT.md`.
Prediction covariance/error growth, command-history or forward-dynamics root
models, versioned moving scenes, mesh CCD, calibration, and plant response
remain separate gates.

### Revision r112 deterministic root-prediction error authority

R112 robustifies the nominal R111 root path without introducing a policy or
physics rollout. The controller receives six nonnegative worst-case bounds:
initial translation radius, translation velocity/acceleration error, initial
attitude radius, and angular velocity/acceleration error. Rust evaluates the
quadratic radii at every world-SDF knot and adaptive midpoint, converts
attitude radius through each probe's compiled root reach, and subtracts the
field-Lipschitz clearance erosion before admission. The derivative of that
erosion joins the continuous rate certificate.

The independent NumPy oracle reconstructs actuator and root quintics, both
error radii, robust sampled distance, first violation, global/local rate bounds,
and dense truth. Its error-only fixture is `21.000 mm` clear and selects Primary
with a zero envelope; the declared envelope reaches `2.124 mm`, reduces robust
clearance to `18.876 mm`, first violates at 2 ms, and rejects both otherwise
stationary plans. This is deterministic worst-case evidence, not covariance,
confidence, or a calibrated estimator claim. Reproduce with
`scripts/run_world_sdf_command_report.sh`; retained reports are
`benchmarks/results/world-sdf-root-uncertainty-r112/WORLD_SDF_COMMAND_AUDIT.md`
and
`benchmarks/results/live-world-sdf-root-uncertainty-r112/LIVE_WORLD_SDF_AUDIT.md`.

### Revision r113 versioned world-scene snapshot authority

R113 versions the immutable SDF separately from the MotionProgram and rooted
frame graph. Each command transaction checks scene epoch, source time,
valid-from/valid-until, optional maximum age, and full 20 ms horizon coverage
before either primary or brake world sweep can become authoritative. An invalid
snapshot marks both plans unknown and selects Reject; it cannot reuse a clear
distance computed against the wrong or stale field.

The policy-/physics-free Python differential isolates valid, epoch mismatch,
future source, not-yet-valid, expired-at-tick, horizon-expired, and too-old
cases, plus malformed-stamp construction. It checks stable reason codes, flags,
selection, exact replay, fixed arrays, zero timed allocations, and latency.
Reproduce with `scripts/run_world_scene_epoch_report.sh`; retained reports are
`benchmarks/results/world-scene-epoch-r113/WORLD_SCENE_EPOCH_AUDIT.md` and
`benchmarks/results/live-world-scene-epoch-r113/LIVE_WORLD_SDF_AUDIT.md`.

This milestone does not hot-swap a field during one controller instance,
interpolate epochs, or sweep moving obstacles. Scene epoch is also not a
map/odom correction: prediction remains in smooth `control_world`.

### Revision r114 observed-versus-commanded authority

R114 maps every observed joint position and velocity through the compiled
actuation transform and compares them with the exact prior-command splice.
Position and velocity retain separate limiting actuator indices and signed
contingency/reject headroom. A warning withholds primary feed-forward effort
and selects the independently validated brake. A hard breach rejects both
plans because command-space limit and collision evidence no longer adequately
describes the observed mechanism.

The policy-/physics-free differential covers nominal, position contingency,
position rejection, velocity contingency, and velocity rejection. Across
10,000 retained transactions it reproduces semantic bytes exactly, allocates
zero timed bytes, and runs at `7.143/12.063/73.249 µs` p50/p99/max. The live
21-capability trace retains 54 nominal and six braking frames during the 80 mm
torso disturbance, with `0.2732` maximum position mismatch and `1.4944`
maximum velocity mismatch. Reproduce with
`scripts/run_command_tracking_authority_report.sh`; reports are
`benchmarks/results/command-tracking-authority-r114/COMMAND_TRACKING_AUTHORITY_AUDIT.md`
and `benchmarks/results/live-command-tracking-r114/LIVE_WORLD_SDF_AUDIT.md`.

This is mismatch detection, not fault diagnosis or plant validation. Actuator
bandwidth, transport delay, contact loss, thermal state, calibration, and
mechanical failure remain separate evidence sources.

### Revision r115 robot-observation timing authority

R115 requires every dynamic transaction to retain producer time,
caller-mapped monotonic control time, stable source identity and sequence, and
synchronization uncertainty. Rust reads no clock. It derives signed age and
separate age/synchronization headroom; causality, freshness, and synchronization
validity remain independent. Any future, stale, or uncertain observation
invalidates both Primary and brake even when its q/v values match the command.

The policy-/physics-free differential covers exact and inclusive-boundary
admission, one-nanosecond stale/future/uncertain breaches, negative uncertainty,
and a combined future+uncertain sample. Across 14,000 retained transactions it
reproduces semantic evidence exactly, allocates zero timed bytes, and runs at
`6.652/10.730/80.883 µs` p50/p99/max. The live 22-capability trace retains
source `0xb015`, sequences 8–244, causal/fresh/synchronized status, 10/2 ms
age/synchronization headroom, 54 Primary plus six tracking brakes, and a
64.602 mm robust world certificate. Reproduce with
`scripts/run_robot_observation_authority_report.sh`; reports are
`benchmarks/results/robot-observation-authority-r115/ROBOT_OBSERVATION_AUTHORITY_AUDIT.md`
and `benchmarks/results/live-observation-authority-r115/LIVE_WORLD_SDF_AUDIT.md`.

This stamps an already reconstructed state. Sorted `ObservationBatch` merging,
full-controller duplicate resolution through `RobotHistory`, and conservative
reconstruction-error propagation into hard margins remain future milestones.

### Revision r116 canonical observation history

R116 implements the missing standalone history boundary in Rust. Fixed-capacity
slots, including generalized state vectors, are allocated at construction.
Online ingest validates program epoch, dimensions/finiteness/manifold, mapped
time order, source identity/sequence, causality, freshness, and synchronization
uncertainty before copying into those slots. Batches ordered by mapped time and
source ID are canonical; an unsorted batch rejects atomically. Equal timestamps
choose the lowest stable source ID independent of arrival/chunking and the
highest sequence within that source.

Queries write into caller-owned state and return exact, shortest-manifold
interpolated, bounded constant-velocity predicted, or held provenance. Source
interval, identity, sequence, age/synchronization headroom, and hard-constraint
eligibility remain explicit. Held state is diagnostic and cannot authorize a
hard row. Epoch/future/stale/uncertain/invalid-state, wraparound, excessive-gap,
and too-old gates all pass. Across 2,000 exact replays, four-record ingest runs
at `0.180/0.231 µs` p50/p99 and reconstruction at `0.090/0.140 µs`, with zero
timed allocation calls or bytes. Reproduce with
`scripts/run_robot_observation_history_report.sh`; the retained report is
`benchmarks/results/robot-observation-history-r116/ROBOT_OBSERVATION_HISTORY_AUDIT.md`.

The retained sample now includes joint q/v, root pose, and world-expressed root
twist. Interpolation is local cubic Hermite in continuous-joint and SO(3)
tangents rather than component-wise quaternion interpolation.

### Revision r117 live canonical observation boundary

R117 removes the live direct-state bypass. Every floating-WBC model, contact,
task, joint-stopping, collision, and command-admission query consumes only a
hard-eligible output of the 64-slot canonical observation history. The toy
producer remains separate and may be guided or integrated, but its state must
pass epoch/layout/manifold/timing ingest and exact/interpolated/predicted/held
reconstruction before hard rows execute. Held or invalid reconstruction stops
the query.

The 23-capability WebSocket contract adds ring occupancy/capacity, accepted,
ignored, and rejected dispositions, source interval/identity/sequence,
reconstruction provenance, age/synchronization headroom, and hard eligibility.
Across a retained 60-frame 80 mm torso pull, all 60 boundary reconstructions
were exact and hard-eligible; ring occupancy was 5–64 of 64 and stream ingest
was 60/0/0 accepted/ignored/rejected. Existing authority behavior remained 54
Primary plus six tracking brakes with 64.602 mm minimum continuous world
clearance. The report is
`benchmarks/results/live-observation-history-r117/LIVE_WORLD_SDF_AUDIT.md`.

### Revision r118 live delayed observation authority

R118 drives four deterministic transport modes through the real WebSocket and
Rust WBC boundary while retaining the guided, policy-/physics-free editor state
path. Exact emits every 5 ms. Interpolated emits every 5 ms but queries 2.5 ms
behind control time, producing a true two-sample cubic reconstruction whose
typed stamp reports the reconstructed time downstream. Predicted uses a 10 ms
producer offset by 5 ms so each 20 ms stream boundary is a genuine 5 ms
constant-velocity prediction. Stale pauses the producer entirely.

Across 12 frames per admitted mode, the retained query counts were 60 exact,
60 interpolated, and 25 exact plus 35 predicted. All admitted states remained
hard-eligible and no held state reached hard rows. Exact and interpolated frames
selected Primary 12/12. Prediction selected Primary 9/12 and the independently
certified braking contingency 3/12, exposing an authority consequence without
claiming plant response. The stale case reached 10 ms source age, exceeded the
5 ms extrapolation horizon, emitted a structured `observation_withheld` event
before WBC, advanced controller time, and recovered exact delivery on the next
tick after reconfiguration.

Reproduce with `scripts/run_live_observation_transport_report.sh`; the retained
report is
`benchmarks/results/live-observation-transport-r118/LIVE_OBSERVATION_TRANSPORT_AUDIT.md`.
Statistical jitter/drop/reordering models and conservative propagation of
reconstruction uncertainty into support, joint, and collision margins remain
later gates.

### Revision r119 reconstruction-error authority

R119 converts reconstruction exposure into typed deterministic error bounds and
passes them into the real floating WBC. Joint stopping intersects the safe
acceleration interval over all four `q ± error, v ± error` corners. Finite
support loses one CoM-position radius, self collision loses two represented-point
radii, and world-SDF collision loses one. Raw witnesses remain visible beside
their robust counterparts.

The policy-/physics-free WebSocket audit retains 20 frames for exact,
interpolated, and predicted modes. Exact exposure is zero and every raw/robust
pair is identical. At 2.5 ms, interpolation produces 0.21875 mrad q error and
erodes support/self/world margins by 0.0765625/0.25625/0.128125 mm. At 5 ms,
prediction increases these to 0.475 mrad and
0.15625/0.525/0.2625 mm. Joint stopping executes the robust corner path but
shows zero live erosion because the authored ±200 rad/s² acceleration cap is
already limiting; the Rust differential unit case proves a tighter robust
interval near a joint boundary. Stale input is withheld and exact mode recovers
in-session.

Reproduce with `scripts/run_live_observation_uncertainty_report.sh`; the retained
report is
`benchmarks/results/live-observation-uncertainty-r119/LIVE_OBSERVATION_UNCERTAINTY_AUDIT.md`.
The growth envelope is caller-authored, not covariance, estimator calibration,
random network evidence, command-trajectory propagation, or a plant-response
claim.

### Revision r120 continuous reconstruction-exposure authority

R120 fixes one tight-avoidance state and sweeps 0, 0.5, 1, 2.5, 5, 7.5, and
10 ms exposure through the same native error-growth, robust joint-stopping, and
floating collision-barrier APIs used by the live path. There is no policy,
physics engine, state integration, or output-as-plant-response feedback. Only
0–5 ms is production-eligible; the final two points show the continuing growth
curve but would be withheld live.

At 5 ms, q/v uncertainty is 0.475 mrad/0.03 rad/s, the near-limit safe joint
acceleration upper bound loses 11.3474 rad/s², the fixed raw 10 mm
self-collision margin becomes 9.475 mm, and required outward acceleration rises
from 3.0000 to 3.0525 m/s². The typed bound costs 53.12 ns/call. Across 5,000
alternating same-process A/B repeats, nominal/robust p50 is
11.823/11.853 µs and p99 is 17.333/17.373 µs. Every exposure records zero
allocator calls/bytes and bitwise-identical generalized-acceleration replay.
The fixed evidence/growth/bound values occupy 144/128/56 bytes respectively.

Reproduce with `scripts/run_observation_uncertainty_exposure_report.sh`; the
retained report is
`benchmarks/results/observation-uncertainty-exposure-r120/OBSERVATION_UNCERTAINTY_EXPOSURE_AUDIT.md`.
Timing is descriptive for this host and one-coordinate fixture. The audit does
not claim calibrated covariance, statistical transport behavior, whole-humanoid
overhead, hardware safety, or plant realization.

### Revision r121 Primary/brake reconstruction-error authority

R121 passes the exact fixed-size `RobotObservationErrorBound` used by the Rust
floating WBC into the allocation-stable Primary and braking command-admission
transaction. Raw commanded-state geometry is retained. Both sampled and
bounded-continuous self-collision witnesses subtract two body-local
represented-point radii. World admission adds root translation/rotation error
to the root-prediction initial radius, then subtracts one local point radius
scaled by the immutable field Lipschitz bound. Tracking mismatch remains a
separate observed-versus-commanded witness.

The live Python audit runs 20 guided state-local frames per exact,
interpolated, and predicted mode without a policy, rigid-body physics, or plant
integration. Self losses are exactly 0/0.25625/0.525 mm; world losses are
0/0.128125/0.2625 mm for Primary and brake, sampled and continuous. Exact,
interpolation, and prediction select Primary on 20/20, 15/20, and 12/20 frames;
the remaining cases are independent tracking contingencies rather than geometry
aggregation. Maximum retained command-admission p99 is 2.359 ms. Stale input is
withheld and exact mode recovers in-session.

The eval also exposed a server adapter mismatch: a fifth stream-boundary
reconstruction could overwrite telemetry after the fourth command result. That
query was removed. A separate twelve-frame transaction audit proves four
queries/frame (48 exact, 48 interpolated, 24 exact + 24 predicted) and equality
between streamed transport query time and command observation stamp.

Reproduce with the two Python entry points
`python/evals/live_command_observation_uncertainty_report.py` and
`python/evals/live_observation_transport_report.py`. Retained reports live under
`benchmarks/results/live-command-observation-uncertainty-r121/` and
`benchmarks/results/live-observation-transport-r121/`.

### 0c. Official Upkie wheel-controller oracle

The full reference runner also builds a tiny worker against the pinned upstream
Upkie repository. That worker links and executes `WheelBalancer.cpp` unchanged.
A second release worker evaluates the Rust law over the same 100,000 sequential
5 ms inputs; Python owns corpus construction, isolated execution, statistics,
and report rendering.

With upstream gains, limits, radius, and update order, both emitted wheel
velocity commands are canonically bitwise equal after signed-zero
canonicalization. The live Bonesaw tuning is run separately and its intentional
command delta is reported by input region and across ten execution windows.
The report also retains p99.99 latency, absolute jitter, RSS, CPU/wall, page
faults, context switches, raw commands, Rust integral state, and allocation
counts:

```bash
./scripts/run-upkie-controller-comparison.sh --ticks 100000
```

The direct report is
`benchmarks/results/reference-latest/UPKIE_CONTROLLER_COMPARISON.md`.
This proves the shared controller law, not equivalence of the surrounding
systems: Upkie writes servo velocity offsets while Bonesaw lowers the same
signal into bounded wheel-acceleration tasks inside its floating WBC.

## Implemented continuous evaluations

### 1. End-effector reference tracking

Both hands follow low-frequency, phase-shifted three-dimensional trajectories.
The report records RMS and maximum point error, joint-limit violation, solver
degradation, contingency use, and tick latency.

This catches transform direction mistakes, Jacobian sign/row-order errors,
unreachable target behavior, and an over-aggressive response profile.

### 2. Conflicting bimanual intent

Both hands request the same point while a higher-level CoM objective and a
lower-level posture objective remain active. The configuration intentionally
passes through rank loss.

The expanded version of this evaluation will snapshot each priority optimum and
assert that adding a lower level does not worsen any earlier residual beyond
`1e-9`. Unit coverage for that invariant already exists in the solver module.

### 3. Walking-motion retargeting

The data-backed corpus uses CMU Graphics Lab subject 37, trial 1 (`slow walk`,
120 Hz). A fetch manifest pins the ASF and AMC SHA-256 digests; raw files stay
in the ignored benchmark cache. The Python evaluator reconstructs named foot
and hand landmarks, extracts a 1.308 s same-phase window, applies a deterministic
six-harmonic periodic reconstruction, scales leg and arm motion independently
to the toy morphology, and time-warps it through 0.75×, 1.0×, and 1.25×
cadence blocks.

This remains a fixed-base retargeting test, not a walking physics simulation.
The target pelvis is fixed, so the common source pelvis-bob component is
removed by grounding the lower foot at every phase. Corpus construction rejects
any resulting flight phase, absence of double support, or foot without both a
stance and swing phase. The 5,000-tick reference has 0% flight, 25.66% double
support, and 606 bilateral contact-label transitions.

The predeclared gate scores:

- foot and hand RMS at `≤5 cm` and `≤3 cm`;
- stance- and swing-foot RMS at `≤6 cm`;
- swing-clearance RMS at `≤3 cm` and minimum achieved clearance at `≥−1 cm`;
- foot RMS at every cadence at `≤6 cm`;
- zero contingency or rejected ticks.

The first grounded 5,000-tick run is a useful failure. Bonesaw reaches
`5.549 cm` foot and `1.411 cm` hand RMS: all checks pass except the overall
`5 cm` foot gate. PlaCo reaches `3.701 cm` foot and `0.427 cm` hand RMS but
misses swing-clearance RMS at `3.301 cm`. Bonesaw's foot RMS degrades
monotonically from `5.276 cm` at 0.75× to `5.816 cm` at 1.25×, while retaining
zero contingency/rejected ticks. This result stays red until the implementation
or retarget architecture improves; the gate is not relaxed after observation.

The moving-root variant is now implemented separately through
`scripts/run-floating-walk-corpus.sh`. It reconstructs absolute source stride,
classifies stance from cadence-normalized moving-foot speed and height, ramps
phase from a double-support reset, and runs fixed-shape arrays through the Rust
floating inverse-dynamics WBC. Rust owns measured touchdown anchors, full
locked-point rows, a typed normal-only/release contingency ladder, velocity
viability bounds, SE(3) integration, and every timed tick.

Its first 119 nominal ticks (0.595 s) reach `0.430 cm` root RMS, `0.878 cm`
stance-foot RMS, `8.532 cm` swing-foot RMS, `14.021 cm` hand RMS, `1.780°`
maximum root rotation, and `1.59 ms` p99. The full 600-tick trace becomes
contact-contingent and then infeasible, so all whole-run tracking and
no-contingency gates remain red. Accepted ticks preserve dynamics/contact
residuals below `1e-8`; raw rejected residuals are reported separately.
Artifacts live in
`benchmarks/results/floating-walk-latest/FLOATING_WALK_CORPUS.md`.

Future variants add model-aware support viability, morphology sweeps,
missing-hand channels, and a second independently licensed clip.

### Revision r264 cross-law residual-prototype freeze

`scripts/run-g1-residual-prototype-profile-r264.sh` rebuilds the Rust/PyO3
boundary and consumes only checksum-pinned R258 and R254 artifacts. It performs
zero policy steps, physics steps, and plant actions. Python fits one complete
55-coordinate observed-state scale, computes leave-one-law-out nearest
residual centers within 16 predeclared closing-speed/tilt cells, and freezes
asymmetric cell calibration plus a maximum same-cell distance. Rust validates
the immutable flat profile once, searches at most 192 caller-owned prototypes,
fails closed beyond the distance gate, constructs all three consequence boxes,
and invokes the existing component/aggregate nonregression selector without
timed allocation.

All 192 spent rows are component- and aggregate-covered; 189 select the exact
zero baseline and three select candidate 2. All three nonzero choices are
actually strictly nonregressing and improving. Rust matches every Python
nearest identity, maximum squared-distance disagreement is 2.84e-14, and every
non-timing output repeats exactly. The recorded query is 1.23 µs p99 with zero
Rust allocations. Worst fitted component/aggregate upper extensions are broad
at 38.317/19.137, so usefulness is intentionally sparse. The profile was frozen
for exactly one untouched new-law/offset holdout; R265 has now consumed that
holdout. No refit, widening, candidate change, plant command, or authority is
implied. See the
[r264 residual-prototype profile](../benchmarks/results/g1-residual-prototype-profile-r264/G1_RESIDUAL_PROTOTYPE_PROFILE.md).

### Revision r265 exactly-once residual-prototype plant holdout

`python/evals/g1_residual_prototype_plant_holdout_r265.py` refuses to run when
either its label or result artifact already exists. Before labels, it pins the
R264 NPZ hash, model hash, complete 192-prototype profile, thresholds,
candidate family, medium elliptic/Euler and stiff elliptic/RK4 laws, and unused
330,000/340,000 offsets. The only execution produced 96 states and 1,440 fresh
MuJoCo steps under the unchanged five-by-4-ms plant cadence; it runs no policy
or plant command.

The mechanism passes: 88/96 states are inside the frozen state-distance gate,
unsupported states fail closed, all queries repeat, Rust allocations and
MuJoCo warnings are zero, profile p99 is 0.996 µs, and WBC p99 is 3.253 ms.
The profile does not transfer. Supported component/aggregate coverage is
92.045%/85.985%; worst joint-position, headroom, and aggregate misses are
27.992, 2.834, and 13.964. One medium-Euler state selects candidate 1 and
improves aggregate consequence by 0.732, but tilt and angular-rate pressure
regress by 0.0119 and 0.00349, so zero of one nonzero actions is strictly safe.
Candidate 1 was present in the frozen family but had never been selected on
spent rehearsal, identifying action-selection support as a separate necessary
gate. The holdout is consumed and may not be repeated. See the [r265 one-shot
report](../benchmarks/results/g1-residual-prototype-plant-holdout-r265/G1_RESIDUAL_PROTOTYPE_PLANT_HOLDOUT.md).

### Revision r267 floating reacquisition localization

`python/evals/g1_floating_reacquisition_localization_r267.py` composes only
completed traces. It records that per-target suppression permits target 0's
new Precontact edge at tick 428 while failed target 1 remains isolated, but the
late target never locks and releases at tick 460. Cap sweeps place the
full-contact convergence knee at 768 sweeps with a 22.9 ms maximum. Equality-
first active-set repair needs 16 iterations to retain contact through tick 329;
the native 600-tick row still records 270 contingency ticks, 23.264 ms p99,
33.291 ms maximum, and 51 twenty-millisecond misses. Combined with R165's 11
earlier plant fall boundaries, the profile is rejected and default-off. No
policy, physics, or plant step is rerun by the report. See the [r267
localization](../benchmarks/results/g1-floating-reacquisition-localization-r267/G1_FLOATING_REACQUISITION_LOCALIZATION.md).

### Revision r262 floating feasibility budget profile

`scripts/run-g1-floating-projection-budget-profile-r262.sh` compares the
unbounded floating support-transfer stress trace with explicit 8/16/32/64
Dykstra projection ceilings. This is an artifact-backed WBC timing evaluation:
it performs no policy step, physics step, or plant action. The unbounded trace
misses the 20 ms 50 Hz budget on 281/600 ticks at 276.8 ms p99. Every finite
profile has zero 20 ms misses, no failed/infeasible or contact-release tick,
and p99 below 5 ms; generic-build maxima still exceed 5 ms. The reproducible
runner then builds with `-C target-cpu=native`, pins logical CPU 4, and repeats
the eight-sweep/two-polish profile in five independent processes. That row has
0/3,000 five- and twenty-millisecond misses, 3.469 ms worst p99, 3.551 ms
observed maximum, 72/72 exact non-timing arrays, and zero Python collections.
The behavior remains functionally red: the strict timing profile exercises
354/600 normal-contact contingency ticks and retains transfer
tracking/residual failures. The finite budgets are therefore a fail-closed
execution option, not a silent default change or an authority admission. See the
[r262 floating projection-budget profile](benchmarks/results/g1-floating-projection-budget-profile-r262/G1_FLOATING_PROJECTION_BUDGET_PROFILE.md).

### Revision r263 floating contact-release recovery

`python/evals/g1_floating_contact_release_r263.py` compares the retained r262
cap8 trace with the current Rust floating session. The old trace can leave the
integrated state unchanged for 171 consecutive ticks after contact
contingency. R263 treats every unsolved contact result (including bounded
`MaxIterations`) as non-executable, retries once with normal-only rows, then
latches released targets out of hard rows until a schedule/reset edge permits
reacquisition. If no contact solve remains executable, Rust advances with a
bounded free-body damping/gravity fallback rather than repeatedly spending the
feasibility budget or freezing the state. The 600-tick comparison has zero
exact state stalls, one release contingency, zero 20 ms misses, and 4.757 ms
p99 in the latest replay. This closes the reset/freeze failure mode and preserves typed degraded
state evidence; it does not admit functional walking, physical CoM/contact
tracking, or plant authority. See the [r263 recovery report](benchmarks/results/g1-floating-contact-release-r263/G1_FLOATING_CONTACT_RELEASE_R263.md).

### Revision r260 behavior manifest

`scripts/run-wbc-benchmark-manifest-r260.sh` composes the pinned fixed-base,
floating, and Upkie artifacts into
`benchmarks/results/wbc-benchmark-manifest-r260/WBC_BENCHMARK_MANIFEST.md`.
It is intentionally artifact-only: no policy or plant is rerun while building
the manifest, and no authority is inferred. The matrix keeps end-effector
reach, bimanual priority conflict, CMU walking retargeting, measured-feedback
moving liftoff, full support-transfer stress, and the upstream Upkie law
together with RMS tracking, p50/p99/p99.9 latency, jitter, deadline misses,
process resources/GC, solver work, contact/dynamics residuals, status
transitions, and ten execution windows. Reach/conflict, moving liftoff, and
Upkie canonical parity pass; fixed-base walking remains red at 5.549 cm foot
RMS and full transfer remains red after contact-mode change. The red rows are
kept as behavior gates rather than omitted or reset.

### Revision r261 correlated complete-state exemplars

`scripts/run-g1-correlated-state-exemplar-profile-r261.sh` consumes immutable
R258 complete terminal states and the unchanged spent R254 diagnostics. It does
not execute policy, physics, or a plant action. Python constructs 3–12 complete
residual exemplars from the same causal group; Rust keeps baseline/candidate
pairs correlated, retains clearance and vertical velocity, performs
support-free propagation and exact consequence scoring, envelopes at most 16
hypotheses, and conservatively selects without timed allocation. R258 source
coverage is 100%/100% with five strictly safe improving selections at 28.10 µs
p99. R254 rehearsal coverage is 85.301%/80.208%, with two selected component
regressions and one aggregate regression, so the profile and authority remain
closed.

### 4. Deterministic replay

The same initial state, targets, tick bytes, program, and CPU backend are
evaluated twice. Every commanded `f64` is compared by bit pattern. This is the
D1 sentinel for the current process and build.

Future variants change observation batch chunking, prefill scratch with
different bytes, permute source arrival before canonical sorting, and run a
30-minute replay.

### 5. Fast frame queries

The benchmark evaluates root-to-frame pose and relative twist repeatedly from
one already-evaluated model cache. It exercises the interaction pattern needed
by the rig editor and high-rate query consumers.

The report separately measures the timestamped historical contract:
authoritative state reconstruction, external-slot interpolation, FK, rooted
atlas evaluation, and pairwise query. This keeps cache-hot current queries from
being confused with the more expensive historical path.

### 6. Rigid-body dynamics identities

Deterministically randomized valid states exercise the mass matrix, recursive
inverse dynamics, and forward dynamics. The report records mass-matrix
symmetry, minimum eigenvalue, inverse/forward round-trip error, and p99 runtime.
This catches gravity-sign, joint-axis, inertial-origin, and parent/child
recursion mistakes without depending on a second dynamics package.

### 7. Controller-path allocation sentinel

A counting global allocator is sampled immediately before and after the
borrowed `advance_into_reusable` transition. Calls and bytes per tick are
reported after warmup. Input, state, task, constraint, solver, collision,
trajectory, and output buffers survive across ticks. The sentinel currently
measures zero calls and zero bytes for reach, conflicting priorities, walking
retarget, and collision-enabled operation across all compiled self-pairs.
The PlaCo comparison report embeds these native results beside process-level
RSS, CPU, fault, context-switch, `tracemalloc`, and Python GC measurements.

### 7a. Compiled signal-jet graph

A flat 13-node fixture exercises scalar/vector inputs and constants, add,
scale, analytic blend derivatives, deadband, clamp, low-pass, and critically
damped spring nodes. Two independent explicit memory/scratch streams receive
identical inputs and must produce bitwise-identical outputs and state. The
fixture also measures the counting allocator around both evaluations.

The graph compiler rejects duplicate stable IDs, forward dependencies,
type-invalid edges, non-finite parameters, and missing outputs. Deserialization
recomputes the canonical layout and rejects forged inferred types, input
counts, or memory slots. A workspace from a same-size graph with different
state types returns a typed layout error rather than panicking.

The 5,000-step full reference run currently reports 13 nodes, two outputs,
three state slots, p50 `0.260 µs`, p99 `0.271 µs`, bitwise repeatability, and zero
allocation calls/bytes per evaluation. This covers the signal runtime itself;
signal-output-to-WBC-task lowering is covered separately below.

### 7b. Compiled signal-to-task controller

Two vector input jets and one rotation input jet pass through low-pass and
critically damped spring nodes whose outputs are resolved at compile time into
one point task, one CoM task, and one orientation task on an independent frame.
Rotation state evolves on SO(3), stays normalized, and exposes world angular
velocity and acceleration verified against small-step finite differences. The
measured transition includes signal evaluation, explicit
double-buffered signal-memory commit, FK and Jacobians, task emission, strict
hierarchical solve, quintic synthesis, sample-block generation, and next-state
output. Two independently constructed state/scratch/output streams receive
identical inputs and are compared bitwise.

Over 5,000 ticks in the full reference run this path reports `0.9214 cm` point
tracking RMS and `0.1929°` orientation tracking RMS, p50 `103.9 µs`, p99
`139.9 µs`, max `167.1 µs`, bitwise repeatability, and zero allocation
calls/bytes per transition. A separate three-axis regression compiles a vector
signal into a direction-aim task and proves that its projected angular Jacobian
retains both tangent directions while the controlled-axis roll column is
exactly zero. The remaining lowering surface is pose, posture, pole-vector,
and inverse-dynamics acceleration/wrench tasks.

### 8. Collision-distance differential

Randomized valid postures scan every compiled conservative self-collision pair.
For the nearest pair, each coordinate's analytic normal-distance Jacobian is
compared with a central finite difference. The report also records pair count,
minimum proxy distance, and p99 full-scan time. Negative proxy distance is not
treated as proof of geometry penetration because non-spherical primitives use
conservative bounding spheres; exact and swept geometry remain a separate gate.

### 9. Hard-constraint conformance

A coupled task is solved against an active two-sided linear row and must recover
the unconstrained residual by re-optimizing in the remaining space. A second
case supplies contradictory half-spaces and must return
`PrimalInfeasible`. The report records maximum feasible violation, active task
residual, contradiction classification, and p99 solve time.

### 10. Unified fixed/floating inverse-dynamics WBC

The native fixture solves 5,000 deterministic steps for both fixed-base and
floating-base profiles. The floating vector prepends six root accelerations,
contains no root-torque variables, and enforces all six free-body equilibrium
rows. Both point-lock and wheel-style rolling contact semantics are covered by
unit tests. The fixed-base long-run corpus deliberately requests tangential
load beyond the friction pyramid on part of the trajectory, so the inequality
becomes active rather than remaining a decorative bound.

The report records:

- maximum rigid-body dynamics equality residual;
- maximum `J q̈ + dJ v` contact-acceleration residual;
- minimum friction and torque margins;
- friction-active and infeasible tick counts;
- bitwise repeatability;
- mean, p50, p99, and maximum solve time;
- native allocation calls and bytes per solve.

The Upkie floating corpus now scores unactuated base equilibrium, contact
acceleration, load/friction margins, determinism, allocation traffic, and
latency. A physics-level regression also reconstructs centroidal
angular-momentum rate from solved external contact moments and checks it against
the requested task. The next expansion must score integrated momentum over a
long reference trace, contact load distribution against an independent
reference, and compiled support-mode transitions.

### 10.1 G1 support-policy matrix

`python/evals/g1_support_policy_sweep.py` runs five predeclared 600-tick cases
against the same official-G1/CMU-derived input: baseline, two centroidal damping
weights, support-preview CoM, and the combined policy. It retains full per-case
NPZ, JSON, and Markdown artifacts and applies the unchanged floating-walk gate.
Avoiding infeasibility is not sufficient when touchdown remains normal-only or
tracking diverges.

The current combined case extends the nominal window from 330 to 532 ticks and
has zero infeasible ticks, but reaches a 104-tick normal-only touchdown with
45.616/46.114/70.212 cm root/stance/swing RMS. All five cases therefore remain
red. The canonical controller policy is unchanged until a whole case passes.

### 10.2 G1 synthetic-step acceptance and CMU stress

`scripts/run-g1-synthetic-step.sh` runs a deterministic 160-tick official-G1
toe-step through the same Rust floating WBC and fixed NumPy boundary as the CMU
corpus. Python predeclares the one-centimetre forward/clearance reference and
contact edge. Rust owns the typed per-target support state, sole-centre landing
shaping, frame-angular task, whole-patch tangential-speed gate, solve,
integration, and transition telemetry.

Acceptance is reported as two explicit sub-gates. The functional gate covers
feasibility, contingency, touchdown duration, root/foot tracking, attitude,
joint speed, and hard residuals. The real-time gate retains the predeclared
5 ms p99 deadline; combined acceptance requires both. Revision r30's latest
toe-step is combined green at 4.282 ms p99; five cached-kernel repeats have a
4.364 ms median p99, with four of five below 5 ms and one retained scheduler
tail at 5.926 ms. The independent 260-tick moving-liftoff profile is combined
green at 3.904 ms p99. The CMU-derived 600-tick full transfer remains the
separate red stress oracle; the r30 support-preview artifact is retained under
`benchmarks/results/g1-cmu-transfer-r30-final` with its full phase,
speed, solver-work, tracking, latency, memory, and contingency arrays.

Revision r31 adds a non-vacuous touchdown-admission gate. The authored phase is
only a request: Rust requires measured sole proximity plus bounded normal and
whole-patch tangential velocity before entering `TouchdownNormal`, and retains
the outgoing support until the replacement locks. The synthetic toe-step still
passes every gate with zero admission delay. The CMU stress records a 172-tick
denial and therefore fails `touchdown_admission_completes_within_8_ticks`; it no
longer presents a far, fast, mid-air foot as a successful contact. See
`benchmarks/results/g1-touchdown-admission-r31/MEASURED_TOUCHDOWN_ADMISSION.md`.

Revision r32 adds measured world-frame CoM position to the fixed-shape trace
without a Python callback or per-tick object allocation. A 250-tick causal A/B
shows that the current support-centre position reference is the first sustained
transfer defect: the baseline root/CoM/swing RMS is
`2.323/3.550/8.921 cm`, while the identical trace with only the CoM task
disabled records `0.400/1.160/0.360 cm`. The control later loses balance, so
the result rejects both the static support-centre servo and the no-CoM policy.
The replacement experiment is predeclared as measured CoM position/velocity
and capture-point feedback with a virtual ZMP clipped to the measured support
polygon. Full methodology, rejected hypotheses, raw artifact paths, and the
unchanged default regressions are in
`benchmarks/results/g1-com-reference-r32/COM_REFERENCE_CAUSAL_AB.md`.

Revision r33 adds the predeclared allocation-free Rust DCM/virtual-ZMP law and
a Python backward-DCM reference oracle. The online controller builds and erodes
the measured support convex hull in fixed storage, clips virtual ZMP, emits a
horizontal CoM acceleration task, and streams every DCM/ZMP/height diagnostic
through fixed-shape arrays. The true full-horizon first 250 ticks reduce swing
RMS from `9.613 cm` under the static support-centre servo to `1.821 cm`, with
`0.966 cm` DCM RMS. The 600-tick case remains red and the policy is not
canonical: 172 touchdown requests are denied, 173 ticks enter normal-contact
contingency, and p99 reaches `48.250 ms`. The default toe-step and liftoff
regressions remain combined green. See
`benchmarks/results/g1-dcm-zmp-r33/DCM_ZMP_EXPERIMENT.md`.

Revision r34 falsifies a simple liftoff-delay explanation and measures the
task-authority trade. Whole-posture Intent, a low-weight protected upper-body
Viability task, and a new smooth joint-velocity-envelope task each postpone the
first contingency and improve DCM or swing tracking, while none preserves root
attitude through 600 ticks. The envelope law is Rust-owned, allocation-free,
uses the effective 8 rad/s controller cap, and activates through a cubic
smoothstep; Python only selects experiment parameters and reports fixed arrays.
It is opt-in because the best no-rejection setting still reaches 179.895° root
rotation. Full matrix and negative controls:
`benchmarks/results/g1-task-authority-r34/TASK_AUTHORITY_EXPERIMENT.md`.

Revision r35 adds a typed, allocation-free measured-contact authority schedule
and fixed telemetry for phase, applied scale, and active envelope coordinates.
The matrix rejects hard switching, symmetric ramps, and static root-task
reweighting. Immediate engagement with a 200-tick release cuts full-run DCM RMS
to `10.831 cm` and retains the always-on envelope's swing accuracy, but the
root-angular residual grows during Precontact before any contact fallback. The
next predeclared experiment is feedback scheduling from DCM support margin,
attitude residual, precontact reachability, and joint utilization. Full data:
`benchmarks/results/g1-phase-authority-r35/PHASE_AUTHORITY_EXPERIMENT.md`.

Revision r36 adds signed measured DCM margin from the exact eroded support hull
and an allocation-free feedback authority law over capture margin, root
attitude, and precontact age. Fixed telemetry records target and applied scale.
The 150-tick feedback release delays contingency to tick `472` without rejected
solves, but signed margin reaches `-85.855 cm`; the landing/reference remains
outside the recoverable set. Full matrix and raw paths:
`benchmarks/results/g1-feedback-authority-r36/FEEDBACK_AUTHORITY_EXPERIMENT.md`.

Revision r37 adds a Rust-owned capture landing planner and reports applied
sole-center anchor, capture scale, authored offset, future-root reach,
projection/slew/freeze flags, touchdown position error, whole-patch tangential
speed, and normal speed as fixed arrays. The stable 8 cm / 0.50 m/s case moves
first contingency from tick `472` to `482` and cuts DCM RMS from `33.520 cm` to
`18.131 cm`, with zero rejected solves. The incoming foot nevertheless has a
minimum `19.857 cm` position error and `0.678 m/s` tangential speed, so zero of
172 delayed-admission observations pass the unchanged physical gate. Full
spatial, commitment, and authority sensitivity matrices:
`benchmarks/results/g1-capture-landing-r37/CAPTURE_LANDING_EXPERIMENT.md`.

Revision r38 fixes two evaluation-oracle horizon leaks before further policy
tuning. `dcm-backward-preview` now performs a fixed receding-horizon recurrence
instead of seeding once from the caller's final array sample, and floating CMU
lateral-root calibration uses the pinned canonical gait cycle instead of the
requested runtime duration. Three regression tests enforce extension-invariant
prefixes and bounded future influence. The corrected 200-tick preview candidate
repeats bit-for-bit across three behavioral traces, reaches first contingency at
tick `480`, and has no rejected solve, but remains functionally red at `9.73 cm`
minimum touchdown distance and `1.626 m/s` minimum tangential speed. Full audit:
`benchmarks/results/g1-bounded-preview-r38/BOUNDED_PREVIEW_CAUSAL_REPORT.md`.

### 11. Integrated floating balance and squat

A 200 Hz Upkie sentinel ramps a 12 cm root-height request, builds a planar
wheel-contact reference with reusable damped Gauss-Newton IK, places the target
CoM over the wheel axle, and solves the floating WBC with rolling contacts.
Three typed signal inputs lower through resolved root-attitude,
root-translation, and CoM acceleration task slots with archived response
parameters. The browser and sentinel call the same policy compiler.
`RollingPoint` is a free-tangent skate/caster abstraction; `RollingWheel`
instead enforces wheel-center no-slip by coupling center and wheel angular
acceleration, with bounded velocity-level stabilization. The raw balance
adapter retains Upkie's reference PI structure, but active leg articulation
requires an axle-to-full-model-CoM virtual pitch rather than torso pitch alone.
The raw regression advances only through SE(3) integration of solver output.
The browser now defaults to the same raw dynamic path and declares
`raw_dynamic` in its WebSocket Hello message. A browser-driven 18-second squat
hold completed without transport or solver errors. Set
`BONESAW_LIVE_GUIDED=1` to restore the separately labelled diagnostic preview,
which solves every WBC substep and then adopts the contact-consistent IK state.

The report separates constrained lateral/normal slip from physically permitted
wheel-axis travel and includes root/CoM tracking, rotation, acceleration,
torque, force, dynamics/contact residuals, friction/torque margins,
degraded/infeasible ticks, latency, and allocation traffic. Over 5,000 steps the
canonical row reaches the full 12 cm command with zero infeasible steps,
`5.93e-6 m` maximum constrained slip, and 1.73 cm permitted wheel travel. A
1 Hz critically damped root-translation spring in the shared compiled policy
shapes the exact linear browser command profile. The latest full path runs at
p50 `262.2 µs`, p99 `277.8 µs`, and max `585.6 µs`, with zero allocation calls
and zero allocated bytes inside the measured steps.

The focused acceptance command is:

```bash
python3 python/evals/rolling_balance_corpus.py --ticks 500 --repeats 2
```

Python defines nominal, training, and held-out initial ground-velocity cases.
For each nonzero case, root and wheel rates satisfy both no-slip rows at tick
zero. All nine cases from -5 to +5 cm/s must pass twice; every non-timing
metric must compare exactly. The current corpus passes 9/9 with zero
infeasible ticks. The browser runs four 5 ms WBC substeps per 50 Hz streamed
state before its guided pose handoff. Raw integration remains available as an
engineering mode until the new controller is promoted into that adapter.

Revision r28 attributes feasibility work with fixed Rust-owned arrays for
cyclic projection sweeps, attempted halfspace projections, active-set polish
iterations, polish pseudoinverses, and polish Jacobi sweeps. On the canonical
160-tick G1 toe-step, normal ticks use one projection sweep while the locked
tail reaches 755 sweeps / 176,670 halfspace projections. Caching immutable row
norms and omitting exactly zero transpose updates keeps all 37 shared
non-timing arrays byte-identical. Five pinned hardware-counter runs reduce
median instructions 6.729%, cycles 12.714%, and task-clock 11.341%. The
regenerated latest trace records 21.661 ms p99, while five additional optimized
repeats span 20.991–30.503 ms; the timing gate therefore remains red without a
single-run speedup claim.
Earlier active-set handoff thresholds are retained only as rejected experiment
artifacts: they reach 7.119 ms p99 but materially change joint trajectories.

Revision r29 replaces the unbounded slow tail with a hybrid seed: the first
eight Dykstra sweeps retain canonical row order, then a preallocated
immutable-origin primal/dual active set computes the Euclidean projection. The
active set is capped at 64 iterations and every accepted point is rechecked
against the original bounds and linear rows at `1e-8`. Five toe-step repeats
reduce p99 median from 24.162 to 6.349 ms and thread-CPU median from 806.149 to
475.791 ms; all 37 r29 non-timing arrays repeat byte-for-byte. A same-revision
moving-liftoff control falls from 11.175 to 4.122 ms p99 while keeping every
functional gate green. On the CMU stress, p99 falls from 177.348 to 13.547 ms
and primal-infeasible ticks fall from 126 to zero, but fallback/release and
metre-scale tracking remain red. Active-set-only, two-sweep, and four-sweep
thresholds are explicitly rejected for behavioral-gate failure or cycling.
See `benchmarks/results/g1-feasibility-r29/HYBRID_FEASIBILITY_AB.md` for state
deltas, jitter, CPU/memory, execution-over-time links, and all caveats.

Revision r30 targets the dominant one-sided-Jacobi pseudoinverse kernel. Column
energies are cached and updated algebraically inside each sweep, then
recomputed exactly in canonical summation order at the sweep boundary. Random,
rectangular, and rank-deficient matrices match the independent established
kernel within `1e-10` relative error. An isolated uncached toe-step records
8.449 ms p99 and 617.2 ms thread CPU; cached five-run medians are 4.364 ms and
411.1 ms. Because the within-sweep floating-point order changes later active
boundaries, this is an algorithmic reacceptance, not a parity claim. The same
campaign repaired active-set failure semantics: repeated working sets are
detected in fixed storage, and any active-set miss restores the exact Dykstra
state and consumes the remaining bounded projection budget instead of
declaring false primal infeasibility. See
`benchmarks/results/g1-jacobi-r30/JACOBI_ENERGY_CACHE_AB.md` for the full A/B,
reference comparison, memory, jitter, temporal, and tracking evidence.

Revision r39 adds a Rust-owned measured touchdown phase cursor. The pure core
policy estimates conservative remaining landing time from sole error,
whole-patch tangential speed, normal speed, acceleration capacity, and the
unchanged contact-admission envelope. The PyO3 batch path samples root, CoM,
all endpoint jets, and contact intent from that one cursor, including the full
velocity/acceleration time-warp chain rule. A deterministic 6 cm step compressed
into 100 ms fails the nominal touchdown-transition gate at 10 ticks and passes
with retiming at 3 ticks. Three retimed repeats are behaviorally bit-for-bit
exact and all pass the 5 ms p99 gate (4.698 ms median). Disabled and already-
viable paths are exact no-ops. The evaluator also requires the first authored
post-liftoff touchdown to be reached; the unsolved CMU H=200 transfer therefore
remains honestly red when its 600/800-tick cursors stop at source ticks
420.872/425.992 before touchdown tick 428. See
`benchmarks/results/g1-phase-retiming-r39/TOUCHDOWN_PHASE_RETIMING_ABLATION.md`.

Revision r40 audits the complete coupled-reference consumer path and tests a
second, measured-balance phase signal. Every floating-WBC consumer uses the
same Rust-owned source tick, next tick, and fraction; the DCM law consumes the
already time-warped CoM jet. A pure allocation-free core policy maps signed
DCM support margin through a C1 rate taper. Disabled behavior is exact across
71 shared non-timing arrays. Immediate engagement is rejected because it
injects `56.1 s^-1` phase acceleration and shortens the clean prefix to 268
ticks. A 50-tick engagement bounds that term to `4.0 s^-1`; adding a `0.5x`
rate floor improves the clean prefix from 482 to 490 and swing RMS from
81.2 cm to 46.3 cm. It is still not a viable landing: at the authored contact
edge the sole is at least 0.639 m away and moving tangentially at 6.044 m/s.
An 800-tick extension diverges instead of converging. The mechanism remains an
opt-in research control, disabled by default. See
`benchmarks/results/g1-balance-phase-r40/G1_BALANCE_PHASE_CAUSAL_REPORT.md`.

The r40 reference-contract suite moves the causal boundary earlier again. It
reads only authored root, CoM, foot, and stance arrays and explicitly forbids
effective/retimed targets, tracked state, controller status, contact forces,
solver outputs, timing, and physics integration. Under a zero-angular-momentum
centroidal assumption it scores positive normal specific force, friction demand,
CoP/DCM margin to the 1 cm-eroded sole hull, contact continuity, and reach. The
0.35× CMU/DCM reference is already red at this layer: `85.37 m/s²` maximum CoM
acceleration, `8.276` maximum friction ratio, and `1.683 m/s` maximum pre-edge
normal foot speed. Static support preview reaches `3822.17 m/s²` and is rejected
even more decisively. See
`benchmarks/results/g1-reference-contract-r40/OPEN_LOOP_REFERENCE_CONTRACT.md`.

Revision r41 runs PlaCo 0.9.23's `WalkPatternGenerator` as a second standalone
reference source. It matches the G1 initial CoM, initial sole poses, first
touchdown pose, 199-tick starting double support, and 229-tick first swing.
`WalkTasks`, IK, WBC, integration, and simulation are never constructed. PlaCo
reduces maximum CoM acceleration from `85.37` to `1.53 m/s²`, jerk from
`17387.0` to `344.6 m/s³`, and friction ratio from `8.276` to `0.156`; its
touchdown reaches `0.008/0.007 m/s` tangential/normal speed. The strict gate
still rejects two transition samples: after the right sole becomes the only
support, the conditional zero-momentum CoP misses by `6.612 cm` at tick 199 and
`0.014 mm` at tick 200. All later CoP samples are inside. The generator is
bitwise repeatable, and raw inputs plus process/runtime metadata are retained.
See `benchmarks/results/g1-reference-contract-r41/OPEN_LOOP_REFERENCE_CONTRACT.md`.

Revision r42 adds Bonesaw's own Rust-native two-stage LIPM boundary planner to
the identical standalone contract. It analytically joins the measured initial
horizontal CoM state to a zero-velocity equilibrium above the future right
sole using two exact constant-CoP arcs. A fixed 91-candidate switch-time search
accepts a plan only when the first CoP is inside the opening double-support
hull and the second and terminal CoPs are inside the 1 cm-eroded future sole.
No policy, IK, WBC, state integration, or physics runs. The resulting trace
passes all `9/9` gates: minimum CoP margin `0.891 cm`, maximum CoM acceleration
`2.167 m/s²`, friction ratio `0.221`, and touchdown tangential/normal speed
`0.000175/0.000151 m/s`. The retained release run plans in `29.3 µs` and
samples 600 ticks in `99.5 µs`; both measured Rust regions make zero allocation
calls and reproduce bit-for-bit. Its `697.2 m/s³` sampled peak jerk is retained
as an explicit multi-step smoothing target rather than hidden by a policy.
See `benchmarks/results/g1-reference-contract-r42/OPEN_LOOP_REFERENCE_CONTRACT.md`.

Revision r43 adds the two missing robot-level bounds discovered by the first
downstream admission. The Rust planner replaces its instantaneous internal CoP
switch with an exact `100 ms` linear-CoP arc, keeping CoM acceleration
continuous and reducing sampled peak jerk from `697.2` to `31.8 m/s³`. It also
projects the matched landing horizontally from `0.871` to `0.800 m`
pelvis-to-ankle reach while preserving authored height. The policy-free,
physics-free result remains `9/9` green with `0.880 cm` minimum CoP margin,
`1.988 m/s²` peak CoM acceleration, bitwise repeat, and zero measured planner
or sampler allocations. See
`benchmarks/results/g1-reference-contract-r43/OPEN_LOOP_REFERENCE_CONTRACT.md`.

Revision r44 makes the pelvis/root motion an explicit morphology variable
instead of equating it with CoM motion. The canonical reference follows 75% of
the horizontal CoM displacement and limits the authored landing to `0.760 m`;
the resulting root-to-foot maximum is `0.777 m`. The standalone reference
remains `9/9` green with no policy, IK, WBC, integration, or physics. See
`benchmarks/results/g1-reference-contract-r44/OPEN_LOOP_REFERENCE_CONTRACT.md`.

The new oracle-state admission then adds the missing morphology certificate in
two explicit Rust stages. Allocation-free whole-body IK preserves both foot
positions and orientations while matching root and CoM geometry; all `600/600`
ticks converge with `4.99 mm` maximum foot error, `0.796°` maximum foot
orientation error, and `27.28 mm` maximum CoM error. An analytic `Jv` / `Jq̈ +
J̇v` solve projects authored velocity and acceleration jets without finite
differencing the independently solved postures. Its maximum residuals are
`0.17%/12.96%` of authored foot velocity/acceleration and `7.69%/25.54%` of
authored CoM velocity/acceleration.

Finally, every floating inverse-dynamics WBC tick runs independently at that
oracle state using the acceleration tasks induced by the projected generalized
jet. There is still no policy, state integration, or physics simulator. All
`600/600` ticks solve, the hard dynamics/contact residuals remain
`1.22e-9`/`4.88e-11`, p99 latency is about `4.3 ms`, all three measured Rust
loops make zero allocation calls, and physical WBC outputs reproduce
bit-for-bit. The full result passes `26/26` predeclared gates. See
`benchmarks/results/g1-oracle-wbc-admission-r44/ORACLE_WBC_ADMISSION.md`.

Revision r45 strengthens the same stateless boundary with hard finite-foot
center-of-pressure constraints. Rust derives a deterministic convex hull from
each four-point sole and emits one linear normal-force inequality per edge,
eroded by the declared margin. The default 5 mm corpus passes `27/27` gates;
the measured minimum geometric CoP margin is 5 mm, p99 WBC latency is about
`3.8 ms`, hard residuals are unchanged, and the hot loop still allocates zero
bytes. The separately retained 10 mm run solves all ticks and respects its hard
margin, but exceeds the predeclared swing-effector tracking gate. See
`benchmarks/results/g1-oracle-wbc-admission-r45/ORACLE_WBC_ADMISSION.md` and
`benchmarks/results/g1-oracle-wbc-admission-r45-margin10mm/ORACLE_WBC_ADMISSION.md`.

The same immutable NPZ can also enter `floating_walk_corpus.py` through
`--reference-inputs`. That path requires an exact tick count and rejects
eval-side anchoring, finite-difference reconstruction, reach projection, and
phase retiming. The first WBC admission with viability-priority CoM tracking
and centroidal damping remains red: it has zero primal-infeasible ticks and a
265-tick nominal prefix, then accumulates contact contingency and tips. This
separates the next missing certificate cleanly: root, CoM, feet, and contact
intent are centroidally and reach-feasible, but the trace does not yet carry a
time-varying whole-body posture/IK witness. See
`benchmarks/results/g1-bonesaw-lipm-wbc-r43/FLOATING_WALK_CORPUS.md`.

Revision r53 extends the stateless boundary to four alternating 62 mm steps,
2,317 oracle states, and eight contact edges. Each Rust LIPM transfer starts
with a 300 ms linear CoP entry from the measured initial CoM projection, so
authored acceleration begins at zero; a second 300 ms ramp replaces the
internal CoP discontinuity. The reference repeats bit-for-bit and analytical
sampling allocates zero bytes. The task solver retains exact undamped hard
feasibility but uses a `1e-8` Tikhonov term for task pseudoinverses, with a
near-singular regression proving bounded response without changing numerical
rank. This removes the prior isolated swing spike: linear/angular effector task
residuals peak at `0.001981 m/s²` / `0.002860 rad/s²`, and CoM peaks at
`0.891961 m/s²`.

Hard admission is green: all 2,317 states solve or use typed slack, dynamics and
contact residuals peak at `1.71e-9` / `6.66e-11`, finite support retains the
declared 5 mm erosion, peak URDF-bounded effort is `67.07%`, physical outputs
repeat bit-for-bit, and all measured Rust hot loops allocate zero bytes. WBC
p50/p99/max is `3.563/6.079/9.595 ms`, with no 20 ms misses. Tracking remains
red only for root angular and height. Their maxima are `2.298717 rad/s²` and
`1.296678 m/s²`; violations occupy 107 and 12 ticks, with longest contiguous
runs of 105 ms and 15 ms. The artifact also retains per-task p99, time-RMS,
integrated residual, violation fraction, and per-step/contact-window evidence,
so a brief excursion cannot be confused with persistent inability. See
`benchmarks/results/g1-multistep-oracle-r53/ORACLE_WBC_ADMISSION.md` and
`benchmarks/results/g1-multistep-reference-r53/MULTISTEP_REFERENCE.md`.

Revision r54 keeps every r53 input byte and threshold fixed, but corrects the
stateless adapter's priority map to match the integrated floating profile:
root attitude and root height are structural `Invariant` tasks; horizontal root
and CoM transfer remain `Viability`. A double-support regression supplies a
deliberately conflicting Viability CoM request and proves that it cannot perturb
the root angular/height invariant optimum. This is a semantic correction, not
weight tuning: the r53 sensitivity runs showed that 10×/100× weights, zero CoP
erosion, 10× friction, exact endpoint projection, and a zero centroidal-momentum
task did not remove the failure.

The unchanged 2,317-state corpus now passes `43/43` gates. Root angular/height
RMS maxima are `0.001849 rad/s²` / `0.000775 m/s²`; root-horizontal, CoM,
linear-effector, and angular-effector maxima are `0.697476 m/s²`,
`0.811208 m/s²`, `0.001421 m/s²`, and `0.002052 rad/s²`. Hard residuals remain
`1.71e-9` / `6.66e-11`, support retains 5 mm, peak effort improves to 53.13%,
and p50/p99/max WBC time is `3.511/5.876/7.832 ms`. All hot loops allocate zero
bytes and physical outputs repeat bit-for-bit. `1,447` ticks still carry typed
Preference/Style slack, which is reported explicitly rather than conflated with
physical tracking failure. The synthetic coupled-actuation variant passes
`45/45`. See `benchmarks/results/g1-multistep-oracle-r54/ORACLE_WBC_ADMISSION.md`
and `benchmarks/results/g1-multistep-oracle-r54-coupled-actuation/ORACLE_WBC_ADMISSION.md`.

## Revision r55 CPU reference audit

`python/evals/cpu_reference_report.py` is a report-only Python evaluator over
immutable raw artifacts. It combines the current r54 G1 admission with the
retained r38 PlaCo, Pinocchio, and upstream Upkie traces without retiming or
mixing their semantic boundaries. Rust still owns every measured Bonesaw hot
loop; Python owns aggregation, checksums, distributions, CSVs, and Markdown.

The G1 section includes every requested performance family: p50/p95/p99/p99.9/
max latency, absolute inter-tick jitter, deadline counts at 1/2/5/10/20 ms,
process CPU/wall, throughput, RSS growth, Python trace memory, native allocation
sentinels, pose/CoM/orientation tracking, physical feasibility margins, all
fourteen task/nullspace residuals, solver-work correlations, and 200-tick
execution windows. Raw per-step NPZ files remain at checksum-addressed source
paths; lossless summary CSVs cover windows, task residuals, and fixed-base
comparison rows.

The interpretation is intentionally narrow. Pinocchio proves rigid-body product
agreement, upstream Upkie proves exact shared controller-law behavior, PlaCo is
an independent fixed-base task comparator, and the G1 trace proves stateless WBC
admission without policy, integration, simulator, or physics. See
`benchmarks/results/cpu-reference-comparison-r55/CPU_REFERENCE_COMPARISON.md`.

## Revision r56 continuous authority over time

`python/evals/authority_over_time.py` consumes the same immutable r54 metrics
and raw NPZ rather than adding a rollout. It constructs eleven independent,
unit-aware pressure traces for hard constraints, finite support, root Invariant,
Viability transfer/effectors, joint position, actuator effort, nominal 5 ms and
declared 20 ms compute boundaries, and Preference/Style clipping. It never
combines these rows into a health score.

Each signal retains raw distributions, warning and critical episodes, longest
runs, dwell curves at 5/10/20/100/500 ms, and leaky exposures with 0.25/1/5 s
time constants. This answers whether an excursion is isolated or persistent
without allowing time to excuse a hard-row violation. The baseline has zero
hard or 20 ms critical ticks. Viability transfer enters its warning band for one
5 ms tick. Nominal 5 ms compute overruns occupy 76 ticks and a longest 40 ms
episode. Preference and Style clipping have longest runs of 990 and 575 ms.
The report, full per-tick CSV, JSON, and public responsive heatmap live under
`benchmarks/results/g1-authority-over-time-r56/` and
`web/AUTHORITY_OVER_TIME_R56.html`.

## Revision r57 CPU-tail stability

The r56 nominal-5 ms pressure triggered a four-run release audit: the retained
baseline and three `taskset` runs on logical CPUs 2, 4, and 6. The evaluator
requires exact byte parity for every non-timing NPZ array before comparing
timing. All 56 fields match, including output acceleration/torque/contact force,
hard margins and residuals, task residual/clipping, status, allocation
sentinels, and solver-work counters.

Timing nevertheless varies materially. P99 ranges from 5.746 to 7.742 ms,
maximum from 7.622 to 12.021 ms, over-5 ms ticks from 49 to 558, and the longest
episode from 20 to 320 ms. Pairwise overrun-set Jaccard falls as low as 0.088.
Dense work remains correlated with latency inside a run, but identical work does
not produce a stable wall-time tail across CPU placement/frequency/contention.
Consequently r57 rejects a clock-triggered core exit and requires an isolated
fixed-governor cycles/instructions experiment before deterministic Jacobi or
clipping budgets are changed. Raw traces, pairwise statistics, CSV, JSON, and
the report are retained under `benchmarks/results/g1-cpu-tail-r57/`.

## Revision r58 CPU-counter stability

Five further release processes run under `perf stat`, each pinned to logical
CPU 4 and each solving the same 2,317-state G1 admission corpus. Every one
passes 43/43 gates, and all 56 non-timing arrays remain byte-exact against r54.
Retired instructions have a 239.807401 billion median and only 0.001081%
relative span. In contrast, task-clock ranges from 15.181 to 19.451 seconds and
cycles from 62.530 to 79.415 billion. The cold/contended first trial reaches a
5.021 ms WBC median while retaining the same semantic and instruction work.

This is strong evidence that the current algorithm has deterministic work and
that host execution rate shapes the observed tail. It is not per-tick solver
attribution: the counters include Python startup, reference loading, IK/jet
witnesses, repeatability passes, report generation, and serialization. R58
therefore preserves the fixed-work core and promotes a native WBC-only counter
sentinel as the next measurement before any Jacobi or clipping budget changes.
Raw perf CSVs, five complete traces, normalized CSV, JSON, Markdown, and the web
report are retained under `benchmarks/results/g1-cpu-counters-r58/`.

## Revision r59 native WBC counter sentinel

The focused native path invokes only the release `bonesaw-eval` binary, the
official 23-DOF G1 URDF, and one fixed-shape floating WBC loop. Each of five
long trials performs 2,000 solves with 58 decision variables, two locked ankle
contacts, and one active Cartesian Preference task. An adjacent one-tick
process is subtracted from each long process before dividing by the 1,999
additional ticks; trial order alternates short/long and long/short.

Marginal retired instructions have a 21.991005 million median and 0.000041%
relative span. Cycles range from 5.884 to 8.980 million and task-clock from
1.439 to 2.187 ms per additional tick, again exposing execution-rate variation
with effectively fixed instruction work. Native p99 latency has a 2.107 ms
trial median. Every long run has an identical non-timing semantic report, zero
infeasible ticks, sub-1e-9 dynamics residual, bitwise final-repeat agreement,
and zero allocations/bytes inside the measured solve.

The subtraction removes most model-load, startup, and serialization work, but
the marginal value still includes the timer and semantic aggregation around
each solve. It is therefore an upper bound, not direct per-solve PMU sampling.
The sentinel is also simpler than r54: it omits finite patches and the complete
five-level task stack. Optimization must keep both the r59 semantic report and
the r54 byte-exact corpus green. Raw short/long counter CSVs and JSON, normalized
trials, metrics, Markdown, and web report are retained under
`benchmarks/results/g1-native-wbc-counter-r59/`.

## Revision r60 discarded-column Jacobi A/B

The first r59-guided dense-kernel experiment moves the cached-energy discard
test before the Jacobi coupling dot. This is semantics-neutral: when either
cached column energy is already below the floor, the original path computes
the dot and then rejects the pair without consuming it. A direct Rust test
confirms bit-exact inverse, orthogonal columns, right vectors, singular values,
rank, and sweep count between paths.

Five native control and five experimental 2,000-tick trials retain identical
non-timing reports. The experiment nevertheless increases the median marginal
instruction count from 21.991005 to 22.083438 million per tick: +92,433 or
+0.420%. Discarded pairs are too sparse to amortize the branch paid on every
pair. Cycle and latency blocks are host-sensitive and do not override this
stable instruction result. Production therefore keeps the original coupling
dot, while the rejected path remains disabled behind the measurement-only
`jacobi-discarded-pair-experiment` feature. The restored default separately
passes r54 43/43 and keeps all 56 non-timing arrays byte-exact. Raw A/B traces,
metrics, CSV, Markdown, and web report live under
`benchmarks/results/g1-jacobi-discarded-r60/`.

## Revision r61 fused Jacobi energy-scan A/B

The second r59-guided experiment combines the initial Frobenius reduction with
the per-column energy scan. The fused loop computes each square once and adds
it to both accumulators in the same column-major order. A direct Rust witness
confirms bit-exact Frobenius squared norm and every column energy.

Five 2,000-tick native control and five experimental trials retain identical
semantic reports, but the fusion increases median instructions from 21.991006
to 22.049923 million per tick: +58,916 or +0.268%. The likely machine-level
cause is lost reduction/vectorization efficiency from two dependent accumulators;
removing a source pass does not reduce retired work. Production keeps the
original separate scans. The fusion remains disabled behind
`jacobi-energy-scan-experiment`. The unchanged default passes r54 43/43 and
keeps all 56 non-timing arrays byte-exact. Raw A/B traces, metrics, CSV,
Markdown, and web report live under
`benchmarks/results/g1-jacobi-energy-scan-r61/`.

## Revision r62 physics-free actuator realization

`g1_actuator_realization.py` consumes the immutable r54 WBC effort and oracle
joint-velocity arrays. Rust advances only realized actuator effort through an
exact first-order bandwidth response, an independent effort-slew constraint,
and an instantaneous available-effort clamp. There is no robot policy, robot
state integration, rigid-body physics, or contact simulation. The stateless
r54 certificate remains authoritative and unchanged.

Six profiles form a capability curve. Infinite bandwidth/rate reproduces all
53,291 requests bit-exactly. Synthetic 20, 10, and 5 Hz cases have 3.083%,
3.954%, and 4.861% p99 error normalized by authored effort limit; worst
contact-edge recovery is 10, 25, and 55 ms. The 5 Hz / 250 Nm/s case has 108
slew-limited ticks. At 50% availability only the two source ticks already above
50% utilization clip. At 25%, 1,623 ticks clip, one edge fails to return below
5% before the next phase, and the longest >5% episode is 2.145 s.

All profiles repeat bit-exactly, respect declared availability, and allocate
zero bytes in measured Rust steps. Mechanical power is retained as
`effort × oracle velocity`; calibrated electrical/thermal/reliability and
constrained acceleration realization remain separate future boundaries. Raw
NPZ, full JSON, compact CSV, Markdown, and web report live under
`benchmarks/results/g1-actuator-realization-r62/`.

## Revision r63 continuous fixed-effort acceleration consequence

`g1_constrained_acceleration_realization.py` replays the r54 WBC once to recover
the full contact-force basis byte-exactly on every retained field, then consumes
the six r62 realized-effort traces at the same 2,317 immutable states. Rust uses
a separately preallocated compact layout containing generalized acceleration
and contact force only. Realized generalized effort moves into the floating
dynamics right-hand side; dynamics and active locked-contact acceleration are
exact equalities. The query has no policy, state integration, contact
simulation, or rigid-body rollout.

The generic hard-inequality projector is deliberately not reused here. An
initial lower-equals-upper effort formulation produced 324 false infeasible
ticks even for ideal control, revealing numerical degeneracy rather than lost
physical authority. R63 instead returns the continuous equality consequence
and scores acceleration-bound excess, unilateral normal force, friction, and
the 5 mm finite-patch support margin as four independent traces. R54 remains
the separate full hard-inequality admission certificate.

Ideal effort solves all ticks, reconstructs admitted acceleration within
`2.43e-9`, reproduces effort exactly, violates none of the four scored
inequalities, and allocates zero bytes. Synthetic 20/10/5 Hz profiles have
`4.58/7.30/10.73` generalized-acceleration RMS error, `15.59/30.60/63.09%`
p99 pressure against the 200-unit acceleration bound, and `75/105/510 ms`
maximum recovery below 1% after contact edges. The 25%-availability slow case
has `11.06` RMS error, a `5.655 s` longest >1% episode, and six edges that do
not recover before the next transition. Per-profile acceleration, normal-force,
friction, and support violation counts remain separate; no aggregate health
score is formed.

Eight mechanism gates pass. Every profile repeats bit-exactly outside timing
on a declared 128-tick witness, all equality residuals remain below `1e-7`, and
every measured Rust step is allocation-free. The observed r63 p99 solve range
is `1.66–2.16 ms` on this unpinned offline run; it is not a replacement for the
pinned r57–r59 realtime evidence. Raw NPZ, full JSON, compact CSV, Markdown, and
web report live under `benchmarks/results/g1-constrained-acceleration-r63/`.

## Revision r64 independent fixed-effort differential oracle

`g1_pinocchio_fixed_effort_reference.py` validates the r63 state-local query
without consuming Bonesaw model products or a Bonesaw solver workspace. It
selects 48 immutable r54 states, retaining all three ticks around each of the
eight contact edges and filling the remainder with evenly distributed and
adverse acceleration-pressure samples. Four effort profiles cover ideal,
synthetic 20 Hz, synthetic 5 Hz, and the deliberately adverse 25%-availability
case.

Pinocchio 4.0 independently evaluates the floating mass matrix, bias force,
eight sole-point Jacobians, ankle-frame angular/linear Jacobians, CoM Jacobian,
and centered finite-difference point `Jdot-v`. The adapter explicitly converts
Pinocchio's `[linear; angular]` free-flyer tangent order to Bonesaw's
`[angular; linear]` order, sorts the 23 scalar joints by Bonesaw coordinate,
and packs active contact forces in the same semantic order while deriving that
order independently from contact state. A separate NumPy SVD implementation
normalizes equality rows, computes the equality nullspace, then freezes the
Invariant, Viability, Preference, and Style optima in sequence with the
declared `1e-12` rank tolerance and `1e-8` soft-task damping.

All six gates pass. Direct evaluation of Rust outputs under Pinocchio products
has `6.30e-9` maximum floating-dynamics residual and `4.38e-10` maximum active
contact-acceleration residual. The independently solved generalized
acceleration differs from Rust by `1.03e-10` RMS or `1.26e-9` maximum across
the worst profile; contact force differs by `2.57e-8 N` RMS or `3.05e-7 N`
maximum. The source artifacts remain checksum-identical and NumPy repeats
bit-exactly on two declared states per profile.

Reference product/solve timings are retained for transparency but are unpinned
Python-process observations, not a speed comparison with Rust. The evaluator
runs no policy, state integration, contact simulation, or rigid-body rollout.
It validates r63 equations and optimizer semantics; calibrated actuator
response, estimator/delay effects, compliance, and closed-loop stability remain
separate required boundaries. JSON, Markdown, and public HTML live under
`benchmarks/results/g1-pinocchio-fixed-effort-r64/`.

## Revisions r65–r66 dense task-row compaction

R65 first tests the narrowest possible change: deleting the identically zero
vertical row from each horizontal-only CoM task before the Viability
pseudoinverse. Five alternating complete admission processes are pinned to one
logical CPU and measured with hardware counters. All 56 non-timing arrays are
bit-exact, but median retired instructions move only `-0.00007%`; paired deltas
range from `+0.00034%` to `-0.00027%`. Host time improves, but the stable-work
signal does not. The candidate is rejected and remains disabled.

R66 evaluates the complete feasible-set compaction. Exact-zero rows may be
removed at any priority. A nonzero row may be removed only at terminal Style
when every coefficient belongs to a finite lower-equals-upper coordinate and
its fixed value exactly equals the row target. Because Style is terminal, no
lower layer can observe a changed projector; because the row is constant on
the feasible set, deleting it does not change the optimization problem.
Declared task shapes, provenance, and residual-row counts remain unchanged.

The retained G1 four-step corpus avoids 20,176 dense row-instances: 8,992 zero
CoM row-instances across repeated Viability solves and 11,184 unused
contact-force row-instances across terminal Style solves. Five alternating
pinned process pairs reduce retired instructions in every pair, tightly between
`-3.86069%` and `-3.86139%`; the median is `-3.86106%`. Process CPU falls
`3.61%`. P50 improves `0.29%`, while p99 changes `+0.12%`, so promotion is
based on stable instructions rather than a host-tail claim.

Production passes all 43 admission gates. Status, task clipping,
pseudoinverse-count, active-set, feasibility, and allocation arrays are exact,
as are all double-support physical arrays. Across 916 single-support ticks,
worst generalized-acceleration, effort, normal-force, and task-RMS deltas are
`1.50e-7`, `1.46e-8`, `5.69e-7`, and `3.23e-8`; all remain inside explicit
field contracts. R63 passes 8/8: all six consequence accelerations remain
within `5.42e-20`, effort and bound pressure are bit-exact, and redundant force
distribution changes by at most `2.76e-6 N`. The independent r64
Pinocchio/NumPy check passes 6/6. The production behavior is default; the
`bonesaw-core/resolved-task-row-compaction-control` feature retains the A/B
control. Full counters, raw paired corpora, JSON, Markdown, and web reports live
under `benchmarks/results/g1-resolved-task-row-compaction-r66/`.

## Revision r67 coincident task-step limits

R67 tests whether multiple hard limits reached by one correction can be frozen
together before rebuilding the projected task pseudoinverse. A deterministic
two-coordinate witness proves the mechanism: two equal upper bounds reduce
three inverse evaluations to two with the same bounded solution.

The complete 2,317-state G1 admission is workload-negative. Control and
candidate both execute exactly 16,809 task pseudoinverses and 10,604 clipped
steps. All physical, decision, residual, authority, clipping, rank,
feasibility, and allocation arrays remain bit-exact. Two Jacobi-sweep
diagnostic samples differ by at most two sweeps and the aggregate increases
from 122,031 to 122,032. Both builds pass all 43 admission gates and allocate
zero bytes in the Rust hot loop.

The candidate is rejected without hardware-counter timing: it eliminates zero
target dense operations, already falsifying the optimization claim, and timing
the added boundary scan would invite a noise-based promotion. The feature
remains an explicit unit-level experiment. The next CPU work must address
sequential non-coincident Viability clipping. JSON, Markdown, retained raw
arrays, and public HTML live under
`benchmarks/results/g1-coincident-step-limit-r67/`.

## Revisions r68–r69 sequential clipping shortcuts

R68 attempts projected-gradient reoptimization after the first limit hit. The
approximate direction selects a different monotonic active set and is rejected
before corpus timing: two established dynamic-WBC tests become
`NumericalFailure`, with maximum original-constraint violations of `1.24e-7`
and `1.69e-7` against the unchanged `1e-8` contract. The code path is removed;
no tolerance or test is weakened.

R69 analytically redirects the remaining correction through the exact current
task nullspace. It carries the correction only when the new limit can be
satisfied without changing the achieved task optimum and otherwise falls back
to the established projected SVD. A redundant two-coordinate witness preserves
the exact bounded solution while reducing two pseudoinverses to one, and all
117 core tests pass under the feature.

The 2,317-state G1 corpus is an exact no-op: every sequential hit changes the
current task optimum. Both builds execute 16,809 task pseudoinverses, 122,031
Jacobi sweeps, and 10,604 clipped steps. All 56 non-timing arrays are bit-exact,
both builds pass 43/43 admission gates, and both allocate zero bytes in the Rust
hot loop. Hardware counters are skipped because the target-work gate already
fails. R69 remains opt-in; production stays r66. Retained arrays, JSON,
Markdown, and public HTML live under
`benchmarks/results/g1-task-nullspace-repair-r69/`.

## Revisions r70–r71 projected-solve factorization

R70 tests the algebraically attractive wide-task identity
`A⁺ = Aᵀ(AAᵀ + λ²I)⁻¹`. A conservative unregularized Cholesky-pivot guard
admits only full-row-rank matrices far from the SVD truncation and Gram
cancellation floors; all other matrices fall back to the established one-sided
Jacobi SVD. Equality and feasibility pseudoinverses are never routed through
the experiment.

The candidate passes all 119 feature-build core tests, 43/43 G1 admission
gates, zero allocation, and the independent r64 Pinocchio/NumPy oracle 6/6.
It reduces reported task Jacobi sweeps from 122,031 to 74,113 (`-39.27%`). It
is nevertheless a semantic rejection: different floating-point order changes
the monotonic limit-selection path on hundreds of states, changes 22/56
non-timing fields, and fails 18 strict r63 replay fields. Maximum deltas include
125.52 in generalized acceleration, 18.68 Nm in effort, and 70.81 N in normal
force. Independent state-local optimizer validity therefore does not imply
equivalence to the established bounded active-set trajectory.

R71 narrows the experiment to a one-row task projection. It preserves the
ordinary norm accumulation, singular threshold, damped inverse factor, and
output arithmetic bit-for-bit while omitting transpose storage, an unused
Frobenius scan, identity construction, and the vacuous Jacobi driver. All 120
feature-build core tests pass, all 56 non-timing arrays remain exact, and five
alternating pinned process pairs remain exact. The retained workload does not
benefit: median instructions change `+0.000103%`; paired changes range from
`-0.000654%` to `+0.000360%`. R71 is rejected at the stable-work gate.

Neither feature is production and no gate is weakened. JSON, retained arrays,
pinned counter CSV, Markdown, and public HTML live under
`benchmarks/results/g1-projected-factorization-r71/` and the source variant
directories `g1-row-gram-r70/` / `g1-rank-one-pseudoinverse-r71/`.

## Revision r72 slice-addressed Jacobi columns

R72 retains the established one-sided-Jacobi factorization and changes only
how its active column pairs are addressed. Each pair is split once into two
prevalidated contiguous slices. Pair traversal, scalar coupling accumulation,
cached-energy decisions, rotation arithmetic, right-vector updates,
sweep-boundary energy re-anchoring, singular truncation, and downstream
hard-limit comparisons stay in the same order. A dedicated kernel test checks
every matrix/right-vector/energy bit across dense, exactly rank-deficient, and
zero-column shapes.

The production and flat-index control both pass all 121 core tests. The
2,317-state four-step G1 corpus passes 43/43 admission with zero allocation;
all 56 non-timing arrays are bit-exact, including physical outputs, residuals,
active sets, ranks, and work counters. Five alternating complete-process pairs
pinned to one logical CPU are exact and every production process retires
30.492% fewer instructions. Median process CPU falls 6.28%, p50 falls 6.28%,
and p99 falls 6.98%; RSS is flat.

A second native 58-variable G1 WBC boundary runs five 2,000-tick trials per
variant with adjacent one-tick setup subtraction. Marginal instructions fall
from 22.022 M to 15.807 M/tick (`-28.22%`), cycles fall 5.18%, task-clock falls
5.37%, p50 falls 5.39%, and median p99 falls 5.62%. Both variants have identical
semantic reports, zero infeasible ticks, bitwise repeat, and zero allocations.

R72 is production default. `bonesaw-core/jacobi-column-slice-control` restores
the flat-index control, while `jacobi-column-slice-experiment` explicitly opts
the slice path back in when composing A/B features. Exact r54 outputs inherit
the established r63 8/8 consequence and r64 6/6 independent-oracle evidence.
Retained raw arrays, five-pair process counters, ten native trials, JSON,
Markdown, CSV, and public HTML live under
`benchmarks/results/g1-jacobi-column-slice-r72/`.

## Revisions r73–r74 post-r72 address/layout audit

R73 begins with a fresh native profile rather than assuming the r72 hotspot
distribution. A 1,999 Hz cycle sample over 2,000 fixed-shape, CPU-pinned,
58-variable G1 solves retains 5,324 samples with none lost. Self attribution is
81.09% in the pseudoinverse symbol, including 16.34% at the Jacobi coupling
line, while the general dense multiply accounts for 8.21%.

Paired slice iterators are the narrow r73 candidate. They retain scalar
coupling and rotation order and produce an identical native semantic report,
but LLVM already lowers indexed validated slices equivalently: five pinned
instruction changes have mixed signs and a `+0.0000014%` median. R73 is
rejected at the stable-work gate.

R74 moves the slice boundary to general dense products. It validates each
left/output row and shared right row once while retaining output zeroing,
row/shared/column traversal, exact-zero left-value skipping, and every scalar
multiply-add expression. A dedicated dense/sparse witness checks every output
bit through a 58 × 58 product. Default production and the explicit flat-index
control each pass all 123 core tests.

The 2,317-state four-step G1 corpus passes 43/43 admission in control,
experiment, and production-default builds. All 56 non-timing arrays are exact,
including 16,809 task pseudoinverses, 122,031 Jacobi sweeps, 10,604 clipped
steps, physical outputs, active sets, and allocation counters. All five
alternating complete-process pairs retire fewer instructions; median
instructions fall 4.104%, process CPU 2.23%, p50 2.19%, and p99 1.94%, with RSS
flat.

The separate native marginal sentinel retains identical semantic reports,
zero infeasible ticks, bitwise repeat, and zero allocation. Instructions fall
from 15.807 M to 15.317 M/tick (`-3.098%`), cycles 2.17%, task-clock 2.30%, p50
2.28%, and median p99 2.17%. Exact source traces inherit r63 8/8 consequence
and r64 6/6 independent-oracle evidence. R74 is production default;
`bonesaw-core/dense-multiply-row-slice-control` restores the flat-index path.
Retained profile attribution, r73 negative counters, raw arrays, five paired
process runs, ten native trials, JSON, Markdown, CSV, and public HTML live under
`benchmarks/results/g1-dense-multiply-row-slice-r74/`.

## Revision r75 fixed-layout batch mirror admission

R75 freezes the first future-CUDA stage boundary without requiring policy,
physics, integration, or a device. `bonesaw-cuda` compiles a fixed SoA
StateInput/FK/CoM descriptor and a preallocated `CpuMirrorF32` executor. The
Python evaluator constructs immutable batches, calls the Rust executor, and
uses Pinocchio 4.0 as an independent f64 frame/CoM oracle.

Across 16 toy-humanoid and 16 Upkie states, maximum frame
translation/rotation/CoM errors are respectively
`1.54e-7 m / 5.91e-8 rad / 1.56e-7 m` and
`7.94e-8 m / 2.62e-8 rad / 7.15e-8 m`, well inside the frozen D3 `5e-5`
contracts. Rotation uses an atan2(skew, trace) comparison and separately
bounds f32 orthogonality, avoiding the false angle amplification of trace/acos.
Repeat output bytes, agent permutation, aligned-versus-compact stride, 7+10
chunk reassembly, and unaffected neighbors around one NaN agent are exact.
Every measured Rust execution reports zero allocation calls/bytes.

For Upkie over 300 calls per batch size, kernel-only execute p50/p99 is
`6.732/8.577 µs` at one agent, `191.101/211.130 µs` at 32, and
`1.985/2.648 ms` at 256. NumPy↔SoA marshalling is intentionally excluded and
must not be described as end-to-end latency. CudaMirrorF32 and
CudaThroughputF32 are unavailable; device D1/D2/D3 are NOT RUN, not passed.
JSON, raw NPZ, Markdown, and public HTML live under
`benchmarks/results/cuda-batch-abi-r75/` and reproduce via
`scripts/run-cuda-batch-abi-audit.sh`.

## Revision r76 fixed-layout Jacobian mirror admission

R76 adds a separate allocation-free Jacobian stage over completed r75 FK
output. Every body emits a six-row world spatial Jacobian—angular then
frame-origin linear—over tangent columns `[root angular; root linear; joints]`;
the system CoM emits a three-row Jacobian with the same columns. Storage is
fixed SoA over the agent dimension and covered by the kernel descriptor.

Pinocchio independently supplies LOCAL_WORLD_ALIGNED joint columns while the
evaluator reconstructs world root columns analytically. Over 16 toy and 16
Upkie states, maximum frame/CoM absolute errors are
`1.53e-7/1.79e-7` and `9.30e-8/5.17e-8`. The worst individual comparison uses
`0.00345×` of the declared `2e-5 + 2e-4·scale` tolerance. A second property
uses central differences after left-multiplying root orientation by
`Exp(±εω)`, translating by `±εv`, and perturbing every joint by `±εqdot`.
Frame-origin and CoM `J·v` pass on both models without policy or physics.

Repeat bytes, permutation, aligned-versus-compact stride, 7+10 chunking, and
NaN-agent isolation are exact across poses, CoM, mass, frame Jacobians, CoM
Jacobians, and status. The invalid agent's Jacobians are zero and every
neighbor is bit-exact. The binding detaches the GIL while both Rust stages run.
Across 300 Upkie calls per size, FK+Jacobian combined p50/p99 is
`20.238/26.782 µs` for one agent, `492.825/662.543 µs` for 32, and
`4.186/5.949 ms` for 256, with zero measured Rust allocations. The
256-agent trace retains a real scheduler tail rather than presenting its
quieter median as a throughput result. These are stage timings only;
Python/NumPy marshalling is excluded. Raw NPZ, JSON,
Markdown, and HTML live under `benchmarks/results/cuda-jacobian-abi-r76/` and
reproduce via `scripts/run-cuda-jacobian-abi-audit.sh`. CUDA remains NOT RUN.

## Revision r77 fixed-layout dynamics mirror admission

R77 adds allocation-free floating mass matrix, bias force, and centroidal-map
products after the admitted FK and Jacobian stages. Layouts are
`M[g][g][agent]`, `h[g][agent]`, and `Ag[6][g][agent]`, with
`g = 6 + dof` and tangent order `[root angular; root linear; joints]` in
`control_world`. The PyO3 boundary detaches the GIL and reports FK, Jacobian,
and dynamics timers separately; NumPy↔SoA copies remain outside them.

The independent oracle uses Pinocchio's free-flyer CRBA, RNEA, and CCRBA. It
does not compare arrays before resolving conventions: Pinocchio's root tangent
is body-local and linear-first, while Bonesaw's is world-expressed and
angular-first. Bias comparison additionally supplies the local linear
acceleration `-Rᵀ(ω×v)` induced by zero world root acceleration. Across 16 toy
and 16 Upkie states with nonzero root/joint velocity and non-axis-aligned
per-agent gravity, maximum absolute M/h/Ag errors are
`8.40e-6/3.75e-5/6.56e-6` and `5.55e-7/5.69e-6/3.24e-7`. All elementwise D3
budgets pass; mass symmetry is bit-exact and every sampled mass matrix is
positive definite.

Repeat, permutation, aligned-versus-compact stride, 7+10 chunking, and
malformed-velocity isolation are exact across M, h, Ag, and status. The bad
agent is typed invalid and zeroed while all neighbors remain bit-exact. Across
200 Upkie calls per size, full FK+Jacobian+dynamics p50/p99 is
`49.784/57.391 µs`, `1.379/1.727 ms`, and `11.417/11.873 ms` for 1, 32, and
256 agents. All raw timing samples, including scheduler tails, are retained.
Raw NPZ, JSON, Markdown, and HTML live under
`benchmarks/results/cuda-dynamics-abi-r77/` and reproduce via
`scripts/run-cuda-dynamics-abi-audit.sh`. CUDA dynamics D1/D2/D3 remain NOT
RUN.

## Revision r78 compiler-resolved point-query admission

R78 freezes arbitrary point sites as compile-time batch slots rather than
accepting runtime frame names or per-agent topology. A canonical point task
automatically contributes its stable ID, body frame, and local offset to the
descriptor; the exact f32 offset bits participate in the kernel fingerprint.
The output layouts are `point[slot][3][agent]`,
`point_jacobian[slot][3][6+dof][agent]`, and
`point_bias[slot][3][agent]`. The final field is the kinematic acceleration
bias `Jdot-v` for zero generalized acceleration, which task/contact emission
subtracts from a desired physical acceleration.

Pinocchio independently evaluates four non-origin sites on each model using
BODY placements and LOCAL_WORLD_ALIGNED spatial Jacobians. The evaluator
rotates each local offset and explicitly converts Pinocchio's body-local,
linear-first free-flyer tangent into the Bonesaw world, angular-first tangent.
A separate f64 central difference left-multiplies root orientation by
`Exp(±dt·omega)`, translates the root, and advances every joint at constant
velocity; its first and second derivatives check `Jv` and `Jdot-v` without a
policy or physics rollout. Across 16 states, worst absolute
position/J/Jv/bias errors are `1.71e-7/1.51e-7/4.97e-8/2.65e-8` for toy and
`8.57e-8/1.14e-7/5.74e-8/2.50e-8` for Upkie. The worst declared tolerance
score is `0.003483×`.

Repeat, reverse-interleaved permutation, aligned-versus-compact stride, 7+10
chunking, and malformed-velocity isolation are bit-exact across all point
outputs and status. The invalid agent is zeroed and cannot perturb a neighbor.
Four Upkie slots add point-stage p50/p99 of `1.182/1.224 µs`,
`25.098/30.250 µs`, and `213.178/251.488 µs` at 1/32/256 agents over 200
untrimmed calls. The complete pipeline allocates zero bytes. Raw NPZ, JSON,
Markdown, and HTML live under `benchmarks/results/cuda-point-query-abi-r78/`
and reproduce via `scripts/run-cuda-point-query-abi-audit.sh`. CUDA remains
NOT RUN.

## Required numerical/property suite

Before dynamic hardware use, CI should contain:

1. FK placements and Jacobians against Pinocchio across randomized valid
   configurations.
2. `Jv` against finite-difference point velocity, including singular and
   joint-limit states.
3. Frame identity, inverse, and three-frame composition.
4. Mass-matrix symmetry/positive definiteness and body-energy equivalence.
5. Gravity generalized force against the potential-energy gradient.
6. Hard-bound feasibility, contradictory constraints, KKT residual, and typed
   infeasibility.
7. Lower-priority invariance after every solver level.
8. Quintic endpoints, derivative continuity, analytic extrema, and exact integer
   sample timestamps.
9. NaN, stale observation, missing frame, history-gap, and impossible-target
   fault injection.
10. Zero allocation after warmup and release p99.999 latency on the target CPU.

## Report policy

Evaluation output is JSON-capable so CI can retain full distributions instead
of screenshots. A benchmark baseline must include the program/model hash,
compiler profile, CPU model/ISA, core revision, tick count, and whether the
process was pinned. Debug-build timings are never used as acceptance evidence.

## Revision r128 matched Upkie external-plant boundary

R128 adds a closed-loop consequence test after, and separate from, the
policy-/physics-free WBC certificates. The evaluator inserts a floating root
into the pinned Upkie URDF before MuJoCo parses it and disables URDF static-body
fusion. This retains rotated fixed-link inertias and restores agreement with
Pinocchio's CoM and sagittal mass matrix. The plant adds only a ground plane and
URDF-limited torque motors; it does not invent armature or joint damping.

A persistent Rust `UpkieBalanceSession` owns balanced-standing morphology
projection, axle-to-CoM virtual pitch, reference PI state/update order, clamps,
and signed wheel coordinates. Python owns MuJoCo soft contact, 1 ms integration,
the 5 ms experiment clock, external wrench, statistics, and artifacts. A
rejected WBC candidate remains diagnostic only; the adapter holds the last
admitted command and records the status instead of resetting the plant.

The 4.5 s nominal case peaks at 0.0063 degrees. A 4 N forward torso force for
100 ms recovers in 1.440 s after a 24.920 degree peak and at most one 5 ms
contactless interval. Its controller p50/p99 is 130.7/164.1 microseconds and
full-loop p99 is 415.7 microseconds, with zero 5 ms overruns, zero timed Rust
allocations, zero Python GC collections, and maximum admitted
dynamics/contact residuals of 7.105e-9/5.207e-9. Four startup candidates are
rejected fail-closed through 20 ms; no later tick is rejected. The otherwise
identical 6 N probe falls and produces sustained MaxIterations/timing loss,
bracketing this one-direction configuration at 0.4–0.6 N·s rather than implying
unbounded robustness. Raw CSV, JSON, Markdown, and HTML live under
`benchmarks/results/upkie-mujoco-plant-r128/` and reproduce with
`scripts/run-upkie-mujoco-plant-report.sh`.

## Revision r129/r130 rooted capture, station authority, and plant envelope

R129 adds an immutable, policy-component and physics-free rooted corpus before
the plant. A persistent Rust session computes the full sagittal DCM
`com_x + com_velocity_x / sqrt(g / height)`, exposes its continuous pressure,
fades a lower-priority odom station anchor with a C1 curve, and evaluates the
compiled `control_world`/`odom`/`map` atlas without allocating. A 10 m
`map ← odom` jump changes global reporting by exactly 10 m while wheel commands
and every non-map control diagnostic remain bit-exact. Ten thousand calls are
exact on replay, produce zero GC, and pass the 1 ms boundary gate. This corpus
has no integration, contact response, learned policy, or WBC solve and does not
claim body response. Reproduce it with
`scripts/run-upkie-rooted-capture-report.sh`.

R130 sweeps actuator-facing DCM velocity fractions
`0.00/0.05/0.10/0.20/0.40/0.60/1.00` through fresh 10-second MuJoCo plants.
The full DCM always remains the viability-pressure witness. Fractions
`0.05–0.40` qualify; `0.00` loses its station by 1.743 m, and `0.60/1.00` fall
with later solver rejection. The frozen `0.20` value has the fastest qualified
station re-entry at 1.870 s; selecting it is therefore an observed tradeoff,
not an unreported constant.

The retained 10-second r130 differential stands nominally, recovers from the
same `4 N × 100 ms` forward torso push after a 24.907° peak in 1.450 s,
re-enters the odom station envelope in 1.870 s, restores authority to 1.0, and
finishes at `−6.9 µm` station error. Controller p50/p99 is
`125.2/166.0 µs`, full-loop p99 is `446.9 µs`, and the recovery case has zero
5 ms overruns, zero Rust timed allocations, zero Python GC collections, and no
rejected tick after the five-tick fail-closed startup. The 6 N overload falls
and loses its timing/admission envelope; that red case is the declared
boundary, not a performance sample to average into the recovered run.

Artifacts are retained under
`benchmarks/results/upkie-rooted-capture-r129/`,
`benchmarks/results/upkie-capture-fraction-sweep-r130/`, and
`benchmarks/results/upkie-rooted-capture-plant-r130/`.

## Revision r131 live typed plant gateway

R131 adds a separately admitted browser-protocol consequence boundary without
changing the policy-/physics-free WBC gates. TARGET remains a guided Cartesian
query on `/ws`. Entering PUSH lazily opens `/plant-ws`; Rust validates finite
wrench vectors, enforces the `8 N` limit and `140 ms` dead-man expiry, correlates
release/reset/error responses, supervises one Python/MuJoCo child, and kills it
when the session closes. Python integrates the pinned floating Upkie at 1 kHz;
persistent Rust capture and WBC sessions run at 200 Hz and measured transforms
return at 50 Hz.

Five fresh-session trials pass physical response, full capture-pressure
exercise, release, missing-release expiry, recovery, invalid-force survival,
correlated reset, and fresh-worker reconnect. Every trial reaches `107.6 mm`
root travel and `25.09°` peak tilt, then finishes the scored recovery at
`0.150°` tilt and `42.17 mm` station error. Maximum retained p99 is
`458.3 µs` for one Rust control tick, `4.236 ms` for one four-tick worker step,
and `41.5 ms` for the 50 Hz stream; numerical reset count is zero. Reproduce
against the single managed 8777 server with
`scripts/run-live-upkie-plant-gateway-report.sh`.

This admits only one sagittal body-center force under ideal observation. R132
separately adds a bounded point differential; lateral recovery, general body
surfaces/directions, friction/slope robustness, delay/noise tolerance, thermal
or hardware safety, and browser frame time remain outside this gate.
Visual/touch/mobile acceptance is NOT RUN in the current CLI environment.

## Revision r132 bounded application-point wrench

R132 prevents a finite but remote point from turning the `8 N` force cap into
an unbounded moment. Rust admits a push only when its body exists in the latest
plant snapshot and its world point lies within `750 mm` of the streamed body
origin. The MuJoCo worker independently checks the exact body-COM lever before
calling `mj_applyFT`. It streams the current lever, the world moment
`(point−COM)×force`, and the maximum moment for the 20 ms step. A worker-side
error clears the active lease rather than repeating until TTL or closing the
socket.

The retained audit uses five fresh sessions for each of three conditions. The
same `2 N` world-X force acts for `60 ms` at base points −200, 0, and +200 mm
in world Z. Final streamed pitch moments are
`−0.400765/0.000289/+0.401712 N·m`; the observed `0.802477 N·m` high-to-low
difference is within `0.002477 N·m` of the exact `0.4 m × 2 N` prediction.
Final signed pitch is `−0.021522/0.030826/0.083378 rad` and pitch rate is
`−0.734179/0.960618/2.576460 rad/s`. Every physical value repeats bit-exactly,
no bounded condition falls, and a 751 mm request is rejected and cleared while
the stream continues. Maximum controller and four-tick worker times are
`325.5 µs` and `2.071 ms`. Reproduce with
`scripts/run-live-wrench-application-report.sh`.

This is not a general wrench or recovery envelope. Other bodies, mesh material
points, arbitrary 3D directions, persistent body-local point attachment,
repeated/simultaneous forces, friction/slope/contact variation, estimator and
transport faults, calibrated resources, hardware, and browser frame time
remain open.

## Revision r133 first-boundary 3D disturbance envelope

R133 replaces the earlier post-fall rollout with a 20-case, first-event
consequence matrix. Python owns MuJoCo contact/integration, exact body-point
wrench application, case construction, friction variation, termination, and
artifacts. Persistent Rust sessions own balanced standing, rooted capture and
station state, reference PI state, floating WBC, torque, status, solve timing,
and allocation counters. This remains a learned-policy-free external-plant
audit; the policy-/physics-free state-local contact replay stays underneath it.

The frozen rows cover ±sagittal, ±lateral, diagonal, ±vertical, magnitude,
equal-impulse duration, three repeated sagittal pulses, base-versus-handle
application, and μ=1.00/0.10/0.03. Ten rows qualify and ten are red. The
canonical 4 N row reaches 107.6 mm and 24.91° and recovers in 1.950 s. Up/down
4 N rows recover in 0.335/0.380 s. Three 2 N × 100 ms pulses one second apart
recover 1.690 s after the final pulse. The 6 N sagittal overload, every lateral
and diagonal row, the 4 N handle row, and μ=0.03 fall; μ=0.10 recovers
physically but records eight later WBC non-admissions.

Every rollout terminates at the first root-height-below-350 mm or tilt-above-45°
fall boundary, or at the first pre-boundary numeric fault. The ±2 N lateral
rows cross at 1.985/1.990 s (0.251% timing difference), while their 48.59%
first-boundary path difference is retained as asymmetry. All 20 traces are
finite, no MuJoCo warning occurs, every failure stops at its first boundary,
canonical semantic replay is exact, and Rust reports zero timed allocation.
This removes the prior post-fall BADQACC/28 m artifact and prevents failed-state
integration from contaminating per-step latency. Red rows still expose genuine
8–12 ms controller tails and non-admission during authority collapse.

Artifacts live under
`benchmarks/results/upkie-disturbance-envelope-r133/`; reproduce with
`scripts/run-upkie-disturbance-envelope.sh`. Slopes, estimator delay/noise,
simultaneous/contact-coupled disturbances, persistent material-point leases,
calibrated actuator/resource limits, lateral recovery logic, and hardware remain
outside this admission.

## Revision r135 policy-free state-local task authority

R135 adds a response-surface evaluation below the closed-loop plant gate. It
contains no policy, state rollout, simulator, or physics engine. Python authors
196 immutable rows: one baseline per contact mode plus every combination of
LockedPoint/NormalPoint/RollingPoint/RollingWheel, six root axes, both signs,
and request magnitudes 1/10/50/250 in native angular or linear acceleration
units. One persistent Rust `FloatingWbcSession` owns model products, hard
contact and rolling rows, dynamics, hierarchy, effort limits, task residuals,
timing, and allocation counting.

All 196 rows are hard-feasible: 22 return `Solved` and 174 return
`SolvedWithSlack`. Maximum hard/dynamics/contact residual is
`1.712e-12/1.712e-12/2.461e-13`, and all timed Rust calls allocate zero bytes.
For RollingWheel, the worse ±1 signed response fraction is `0.8068` for roll
and `0.1933` for lateral translation; their maximum directional responses on
the declared curve are `15.2984 rad/s²` and `8.8103 m/s²`. Full JSON retains
both signs, orthogonal leakage, the first four task-layer RMS/clipping rows,
friction margin, effort utilization, hard residual, and solver work rather
than rolling them into one health score.

This state-local evidence does not imply recovery. The r134 planar-capture A/B
still rejects promotion after 0/3 lateral recoveries, and r133 remains the
external-plant consequence boundary. Reproduce with
`scripts/run-upkie-state-local-authority.sh`; artifacts are retained under
`benchmarks/results/upkie-state-local-authority-r135/`.

## Revision r137 live stale-command expiry

R137 evaluates command freshness separately from r136's rejected damping
blend. A current `MaxIterations` candidate remains non-executable. Rust grants
only the last admitted torque a five-tick lease after non-admission and fades
that authority to zero by tick twelve; valid primary acceleration objectives
are untouched.

The retained six-case baseline/candidate/replay matrix keeps nominal and the
qualified 4 N sagittal recovery bit-exact. The left 1 N, left 2 N, right 2 N,
and 6 N overload boundaries move by `+0.210/+0.365/+0.025/+0.205 s`, so none
regress. Maximum consecutive stale-command ages fall `48→17`, `26→22`,
`20→11`, and `41→28` ticks. Fresh authority engages in every adverse row,
candidate replay is exact, traces are finite, and timed Rust allocation is
zero. Terminal kinetic energy remains reported but is not used as a freshness
gate; this revision makes no recovery claim.

Reproduce with `scripts/run-upkie-stale-command-expiry-ab.sh`; artifacts are
retained under `benchmarks/results/upkie-stale-command-expiry-ab-r137/`.

## Revision r138 frozen measured-contact authority

R138 sources sixteen observations at 500/250/100/50 ms before the four frozen
adverse first boundaries, then stops the plant. The subsequent 96 WBC queries
contain no policy, integration, or physics step. Each frozen q/v/root state is
queried with roll-only, lateral-only, and coupled 250 ms rate-arrest requests
under both the controller's permanent double-RollingWheel declaration and the
exact named wheel-subtree-to-world contact mask.

The measured masks contain `11`, `10`, and `00`; 12/16 snapshots disagree with
the permanent declaration. All 48 measured-mask queries are hard-feasible with
maximum violation `5.954e-12`. The stale declaration returns `MaxIterations`
for 30/48 queries and reaches hard violation `216.339`. Exact semantic replay,
finite output, discriminating authority, and zero timed Rust allocation pass.
Signed projection gain remains separate and can be negative or exceed one; it
is not an aggregate health score.

This admits an observation boundary only. The source contact is simulator
truth, not a causal estimator, and a feasible one-/zero-contact local equation
does not provide a transition or recovery action. Reproduce with
`scripts/run-upkie-contact-truth-authority.sh`; artifacts are retained under
`benchmarks/results/upkie-contact-truth-authority-r138/`.

## Revisions r139–r145 causal contact and negative support-action gates

R139 is policy-, WBC-, plant-, integration-, and clock-free. Python authors
immutable observations; generic `bonesaw-core` validates caller tick time,
mapped time, age, source identity, sequence, and synchronization uncertainty.
It exposes Exact/Held/Unavailable provenance, raw contact, debounced mode,
hard eligibility, pending counts, transitions, and typed flags. Three exact
samples activate a new contact; exact absence immediately removes its hard row;
two samples change stable mode. Mirrored 10/01, activation/loss chatter,
missing/expired, future, stale, uncertain, wrong-source, duplicate-sequence,
and reordered-time cases pass all nine gates with atomic faults and exact
replay.

R140 directly executes the measured-mask WBC over the frozen 20-case plant
matrix. Raw non-admission falls `711→43`, but 9/10 previously qualified rows
become falls. R141 keeps every reduced-support result diagnostic-only and
expires the last admitted double-support command through r137. All ten green
physical outcomes remain green, but they acquire later non-admissions and all
nine baseline falls happen `0.345–3.455 s` earlier. Both deployments are
rejected despite exact replay, finite traces, and zero timed Rust allocation.

R142 asks whether a continuous WBC load witness can act earlier. Positive
per-wheel normal-force share is checked at fixed 10/20/30% thresholds for two
ticks inside a 500 ms pre-loss window. Coverage is zero at all thresholds.
Transient contact loss also occurs in multiple green recoveries, so the binary
edge alone does not classify failure. This is retained negative evidence, not
threshold tuning permission.

R143 moves command retention itself into generic allocation-free Rust. A fresh
six-actuator command can enter only from an admitted solve with exact evidence;
the state stores its authoring support mask and explicit tick. Replay is allowed
only while the exact hard mask differs and only inside a configured bound.
Missing/rejected evidence, returned authoring support without a new solve,
non-monotonic ticks, non-finite commands, and expiry are typed non-executable
states. The policy-/plant-/WBC-free mirrored/zero-support corpus passes 10/10
gates with exact replay and zero hot-path allocation.

R144 makes that five-tick lease the sole post-loss actuator authority. Bounded
lifetime, zero post-expiry torque, suppressed stale contact-force witness,
exact replay, finite state, and zero Rust allocation pass. Deployment is
rejected because 7/10 green qualifications are lost and 7/9 existing falls
move `0.065–3.505 s` earlier. R145 then tests the alternative continuous
composition: r143 contact grace followed by r137 freshness fade. Its zero-tick
row is execution-bit-exact with r141; every 0/2/4/8/16 row preserves all ten
green physical outcomes. All nine r137 falls remain earlier, however, and
nonzero grace monotonically worsens aggregate fall time from `−12.675 s` to
`−12.730/−12.775/−12.895/−13.015 s`. Neither strict expiry nor sequential fade
composition is a viability action.

Reproduce with `scripts/run-upkie-contact-observation-contract.sh`,
`scripts/run-upkie-contact-observed-controller-ab.sh`,
`scripts/run-upkie-reduced-support-command-gate-ab.sh`, and
`scripts/run-upkie-contact-unload-precursor.sh`, and
`scripts/run-upkie-contact-command-lease-ab.sh`, and
`scripts/run-upkie-contact-command-freshness-composition.sh`.

## Revisions r146–r150 bounded viability request and plant rejection

R146 freezes 16 pre-fall states and asks a policy- and plant-step-free question:
which of 245 bounded roll/lateral/yaw WBC requests best reduces a declared
250 ms constant-acceleration capture-pressure forecast? Fifteen states admit
strict descent, but only 9 finish inside the unit boundary and 15 choose a
lattice edge. R147's two-pass coordinate selector uses 40 exact queries,
activates on 12/16 states, strictly descends every activated state, reaches a
1.011 median pressure ratio to the oracle, and records 3.808 ms p99 summed Rust
work with exact replay and zero allocation.

R148 tests execution independently of selection. Generic `bonesaw-core` owns
request hysteresis, per-coordinate bounds and slew, explicit planner sequence,
four-tick age, Fresh/Held/Releasing provenance, and immediate revocation on
missing exact evidence or a failed update. Python owns only the bounded search
orchestration. An executable request cannot become torque until a current exact
WBC solve admits it.

R149 evaluates r137, a measured-contact control, the supervised candidate, and
exact replay over all 20 retained MuJoCo cases. All 11 mechanism gates pass,
including bounded 40-query searches, strict admitted descent, exact replay,
zero timed Rust allocation, and zero Python GC. Deployment fails: 11/19 falling
measured-control rows cross earlier (worst `−1.490 s`) and 395 planner ticks
miss the 5 ms Python-loop budget.

R150 isolates support composition by leaving the r137 double-support program
unchanged until the supervisor has a fresh executable request. All ten r137
green rows are preserved. Across 13,249 exact planner queries, replay is exact,
state remains finite, and timed Rust allocation stays zero. No adverse row
recovers; eight cross earlier, with a worst delta of `−1.065 s`, and the worst
per-case p99 controller step is `16.239 ms`. The mechanism evaluation passes,
deployment is rejected, and r137 remains live. Reproduce with
`scripts/run-upkie-state-local-viability-descent.sh`,
`scripts/run-upkie-viability-coordinate-planner.sh`,
`scripts/run-upkie-viability-request-supervisor.sh`,
`scripts/run-upkie-viability-request-plant-ab.sh`, and
`scripts/run-upkie-conditional-viability-planner-ab.sh`.

## Revisions r151–r156 fixed-budget multi-step viability

R151 changes the policy- and plant-step-free selector objective rather than its
execution authority. Rust scores eight fixed future knots over 240 ms and
reports capture, sagittal, terminal rate, yaw, support-closing direction,
actuator, joint, action, and action-change pressures separately. Python issues
at most four planner WBC queries per control tick: one zero baseline plus one
three-point local coordinate question. Exact replay, zero timed Rust
allocation, zero Python GC, finite-state, bounded-query, and strict-descent
gates pass. The broad wake policy nevertheless loses five retained green rows
and moves the worst boundary `−4.360 s`, so it is negative physical evidence.

R152 restricts wake authority to current roll/lateral capture. The remaining
pressures continue to veto proposals but cannot independently wake request
coordinates that cannot correct them. A 40/40/20 one-tick trust region, pitch
path non-regression, and a minimum-improvement rule preserve all ten r137 green
rows and avoid new falls. No adverse row recovers; one boundary moves
`−0.085 s` earlier, 660 controller ticks exceed 5 ms, and the worst per-case
p99 is `20.158 ms`. A planner-only eight-iteration feasibility cap produces no
material timing or consequence change, showing that the active-set projection
is not the dominant tail.

R154 reruns the budgeted planner with four causal arms: retained r137 sentinel,
measured-contact control without a planner, measured-contact candidate, and an
exact candidate replay. The candidate passes all 11 mechanism gates. Relative
to the measured-contact control, 4/19 falling boundaries are earlier, 5/19 are
later, and 10/19 are neutral; the worst and best deltas are `−0.250 s` and
`+0.560 s`. Compared with r149, earlier boundaries contract 11/19→4/19 and
loop overruns 395→63, but aggregate exact queries increase 19,649→20,957 and
summed within-run RSS increases 1.38→2.00 MiB. Physical promotion therefore
fails. This comparison also prevents the planner from receiving credit for
the measured-contact controller's independent consequences.

R155 replaces the three-point coordinate batch with one Rust-owned signed
proposal after the mandatory same-state zero baseline. The fixed phase,
axis/sign, clipping witness, and invalid-input atomicity are allocation-free.
The 20-case causal matrix passes all 11 mechanism gates and lowers total exact
planner queries from 20,957 to 12,140. Physical promotion still fails: 5/19
falling boundaries are earlier, the worst is `−0.315 s`, and loop overruns
only change 63→61. The query reduction is real; the tail is not proportional
to aggregate poll count.

R156 inserts two-update same-support confirmation between forecast selection
and request supervision. Across the four-arm matrix, 19 ticks are shadowing,
26 are confirmed executable, and one raw support change revokes confirmation.
All 15 mechanism gates pass, including exact replay, strict descent, no
confirmation bypass, bounded queries, finite traces, and zero timed Rust
allocation. Of 19 falling measured-control rows, 15 are neutral, two later,
and two earlier; the worst regression is `−0.450 s`. Planner and final-WBC
worst per-case p99 are independently 6.08 and 6.10 ms, while Python GC remains
zero. Confirmation is retained as a valid mechanism and rejected as a
physical deployment.

Reproduce with
`scripts/run-upkie-budgeted-multistep-viability-ab.sh`,
`scripts/run-upkie-lateral-budgeted-multistep-viability-ab.sh`,
`scripts/run-upkie-lateral-anytime-multistep-viability-ab.sh`,
`scripts/run-upkie-multistep-viability-plant-ab.sh`,
`scripts/run-upkie-paired-multistep-viability-plant-ab.sh`, and
`scripts/run-upkie-confirmed-multistep-viability-plant-ab.sh`.

## Revisions r157–r159 hybrid guard, realization, and certificate rejection

R157 exposes the scorer's exact eight `[time, roll/rate, pitch/rate,
lateral/rate, yaw/rate]` knots in caller-owned storage and joins each confirmed
proposal path to later 200 Hz plant state. Exact replay and zero allocation
hold. Only 25/173 complete comparisons preserve origin support; 148 cross a raw
support transition, separating hybrid-contact error from same-support model
error rather than hiding both in one percentile.

R158 inserts a generic Rust hybrid-support guard before confirmation. It
requires four exact same-support ticks, rejects roll that opens a missing wheel,
and requires material load on both wheels under double support. The four-arm
matrix exercises 20 admissions, nine dwell rejections, and four direction
rejections. Seventeen of 18 mechanism gates pass; the retained plant rows do
not exercise support-change revocation. Four of five physical gates pass: no
fall boundary is earlier, one is 10 ms later, and 18 are neutral, but 62 loop
deadline overruns reject promotion.

R159 repeats the join for every fresh final exact-WBC command, after command
admission rather than proposal selection. A frozen 10-case calibration / 10-case
holdout split yields 11,034 origins and 63,308 same-support comparisons. All
nine measurement gates pass, replay is exact, Rust allocations are zero, and
path emission is 0.311/0.511 µs p50/p99. The proposed calibration maximum plus
5% reserve is rejected: joint holdout coverage is 98.231% overall, falls to
95.681% at 240 ms, and misses by as much as 14.275×. A separately retained
force-quiescent slice reaches 99.063% but also fails. No online certificate or
authority is created. Reproduce with
`scripts/run-upkie-hybrid-guard-viability-plant-ab.sh`,
`scripts/run-upkie-forecast-realization-audit.sh`, and
`scripts/run-upkie-forecast-realization-contract.sh`.

## Revisions r175–r181 observed-support action and guarded consequence

R175 is the policy- and physics-free state-local action corpus. Rust derives
current CoM and the active support centroid from the Upkie model, authors
double-/single-support braking or exact ballistic flight, and submits the
request to an independent current-support floating WBC. All 256 frozen rows
are admitted with `2.05e-11` maximum hard violation, `124.453 µs` WBC p99,
exact reordered/fresh-session replay, mirrored single-support evidence, and
zero timed allocation/GC. This proves request construction and instantaneous
admission, not recovery.

R176 supplies the causal plant consequence and rejects direct selection. Its
non-executing shadow is exact and every one of 1,703 selections follows exact
double-support arming, observed support loss, and typed candidate admission.
Nine of ten r137 green rows physically fall, all ten lose qualification when
timing is included, and eight existing fall boundaries move earlier.

R177 separates that action failure from contact enable/reacquisition. Keeping
the established primary outside unconfirmed contact activation makes the
shadow bit-exact to r137 and restores all 10/10 green rows, but the preserved
primary has no current observation authority and the selected action advances
seven red boundaries. R178 therefore adds generic allocation-free Rust
`ContactProgramAuthority`: fresh primary/current-support commands require exact
raw=stable=hard masks, transitions may execute only a prior admitted command
inside a fixed lease, and startup/evidence/mask/sequence/expiry faults withhold.

R179 closes the orthogonal action-conditioning gate inside r177's exact
scaffold. Only the first exact mask-0 candidate is evaluated, and the lease is
consumed on accept or reject. Selection requires typed WBC admission, at least
`0.10` Rust eight-knot score improvement over inertial continuation, and
achieved root angular acceleration within `40 rad/s²`. The complete five-arm
20-case run has 17 queries, 11 transfers, and six rejections. All 10 r137 green
rows survive; no fall boundary is earlier; diagonal, low-friction forward, and
handle-forward boundaries are later by `0.200/0.055/0.035 s`. Shadow and
selected replay are exact, timed Rust allocation and Python GC are zero,
single-query WBC/forecast maxima are `143.902/0.521 µs`, green loop maximum is
1.113 ms, and total candidate overruns change `642→592`. The conditioned action
passes, but the controller remains rejected because r178 current-observation
authority is deliberately disabled in this scaffold.

Reproduce with
`scripts/run-upkie-support-contingency-admission.sh`,
`scripts/run-upkie-support-contingency-plant-ab.sh`,
`scripts/run-upkie-support-contingency-primary-preservation-ab.sh`, and
`scripts/run-upkie-guarded-flight-contingency-plant-ab.sh`.

R180 composes exact measured contact with generic Rust
`ContactProgramAuthority` and a fixed-primary-effort current-support WBC. Its
20-case five-arm matrix selects current support 1,227 times, including 381
debounce ticks, while a zero-tick retained-command profile avoids stale
extension. All r137 execution traces, 10 green rows, and 10 fall boundaries
remain exact; replay and sparse-row semantics pass. The mechanism is admitted,
but its retained ordinary-process profile has nine 5 ms loop misses and one
5.036 ms controller call, so synchronous and hardware promotion remain red.

R181 adds the required ablation and freshness check. Its raw current-support
transfer changes a terminal boundary or qualification in 16/20 rows. Fixed
effort realization instead preserves the effective r137 torque exactly on all
18,209 ticks, including 1,227 current-support selections, with no withheld,
lease, or fallback tick. All plant/command traces and terminal boundaries are
bit-exact; replay is exact; maximum hard violation is `2.77e-9`; worst per-case
proof-WBC p99 and absolute maximum are `108.011/128.713 µs`; and allocation/GC
remain zero. The fall-safe supervisor continues to derive primary freshness
from the raw primary solve, never from the current-support realization. This
admits semantic current-observation composition, but not the ordinary
synchronous profile (10 red-tail misses, 5.639 ms controller maximum) or a
hardware controller. Reproduce with
`scripts/run-upkie-contact-program-authority-plant-ab.sh` and
`scripts/run-upkie-current-support-realization-plant-ab.sh`.

## Revisions r182–r191 observation fault consequence

R182 executes exact, 5/20 ms latency-onset, periodic single/burst dropout,
left/right bit chatter, and mirrored ±1 N lateral profiles over seven cases.
Every unavailable tick fails closed and all exact-green cases survive, but
seven existing fall boundaries move earlier. R183 separates monotonic steady
20 ms age from abrupt latency onset: the steady stream is plant/command exact,
whereas the onset correctly produces backwards-timestamp rejection. R184 then
defers dropout until primary authority has run for a complete period and proves
zero torque remains consequence-unsafe, with ten earlier case/profile
boundaries and a `−1.245 s` established-handle shift.

R185 tests an explicit alternative in the Rust authority state machine. The
new `RetainedInexactObservation` provenance has a separate one/two-tick budget;
it never presents as a fresh WBC solve, reproduces the preceding admitted
effective effort bit-for-bit, emits no contact-force witness, and expires to
withheld zero torque. Dormant exact hold-0/1/2 traces are identical and all 274
selected holds pass provenance, burst-count, replay, finite-output, allocation,
and GC gates. No unconditional policy passes strict plant non-regression. The
one-tick 5 ms profile is closest: right and handle boundaries improve by
`+0.515/+0.570 s`, but the left boundary is still `−0.040 s` earlier. The
10 ms one-tick profile reaches `−1.655 s`; the run also has 81 loop misses and
a 6.845 ms controller maximum. Reproduce with
`scripts/run-upkie-contact-program-robustness-ab.sh`,
`scripts/run-upkie-observation-delay-startup-ab.sh`,
`scripts/run-upkie-observation-dropout-phase-ab.sh`, and
`scripts/run-upkie-inexact-observation-hold-ab.sh`.

R186 makes the first unavailable-tick authority continuous rather than adding
another fixed hold duration. The Rust state stores a Q15 fraction, scales the
cached admitted effort exactly without compounding, and treats zero as disabled
retention; the zero traces match the r184 withheld control exactly. A single
causal fraction is used before the evaluator knows whether loss lasts one or
two samples. Across seven cases, 13 profiles, and paired replay, all mechanism,
provenance, finite-output, allocation, and GC gates pass. None of
`0/0.25/0.5/0.75/1` passes strict consequence: every fraction advances an
existing fall boundary or creates a new fall, and `0.5` turns the recovering
forward-4 N reference/5 ms row into a fall at 2.495 s. The retained rerun
records 131 loop misses and a 6.955 ms controller maximum. Reproduce with
`scripts/run-upkie-inexact-hold-authority-sweep.sh`.

R187 is the denser duration-oracle negative control. Q15 gains `0.45` and
`0.95` pass strict consequence only when the known 10 ms arms are evaluated
separately; no tested gain passes the 5 ms arms and none passes both. The first
unavailable tick is identical in the two fault classes, so duration-specific
selection cannot be causal. Exact dormancy, scaled-effort equality, withheld
zero semantics, replay, finite output, allocation, and GC gates pass. Global
policy and synchronous admission remain rejected; 196 loop misses and a
7.182 ms controller maximum are retained. Reproduce with
`scripts/run-upkie-inexact-observation-duration-oracle-ab.sh`.

R188 replaces duration knowledge with a causal support-free Rust forecast
selector. On each unavailable tick it scores Q15 fractions
`0/0.25/0.5/0.75/1` using only current reduced root state and the previous
admitted command witness; support is forced unknown and exact ties select less
authority. Exact/dormant/replay/finite/allocation gates pass, and matched 5/10
ms first-loss prefixes are identical. Across two independent full-process
repetitions, the selector maximum is `3.015–6.192 µs`, but
strict plant consequence rejects the score minimum: backward-4 N/5 ms becomes
a new fall at 3.000 s and six existing boundaries advance, worst `−1.995 s`.
The repetitions retain 100–104 loop misses, 6.122–6.247 ms worst loops, and
5.697–5.866 ms controller maxima.
Reproduce with `scripts/run-upkie-inexact-hold-forecast-selector-ab.sh`.

R189 sweeps a causal minimum forecast-improvement gate through a
withheld-equivalent endpoint. Every mechanism gate passes and no threshold
passes both 5/10 ms consequence. Margin `0.001` removes R188's green fall but
advances left by 0.620 s; larger values converge toward hold-0. The run records
192–235 misses and 5.699–6.645 ms controller maxima across three repetitions.
Reproduce with
`scripts/run-upkie-inexact-hold-improvement-gate-ab.sh`.

R190 evaluates a distinct support-free brake rather than another retained
torque gate. Rust authors ballistic gravity plus attitude/joint damping, a
separate zero-contact WBC admits it, and typed selection 5 executes without a
contact-force witness or Primary refresh. Mechanism passes; author/WBC maxima
span 3.486–9.778/154.372–178.638 µs across three repetitions. Plant
consequence rejects it: left improves, but two
green sagittal rows fall and right/handle regress, worst −1.935 s. The run has
113–132 misses and 5.779–6.261 ms controller maxima. Reproduce with
`scripts/run-upkie-inexact-support-free-brake-ab.sh`.

R191 evaluates a conservative three-way terminal-impact chooser over withhold,
the independently bounded retained command, and the fresh support-free WBC
candidate. Rust predicts ballistic time and vertical specific impact energy at
a declared `0.225 m` Upkie root-impact plane, while terminal tilt/rate, joint
headroom/speed, effort, and admission remain separate non-aggregated harm
components. A non-baseline action must improve at least one component without
regressing any. Mechanism passes across 62,817 first-run ticks, 77 paired
profiles, and 481 terminal queries per run: exact configuration is dormant,
5/10 ms first-loss evidence is identical, typed choices map to execution,
replay is exact, an independent re-audit reproduces every diagnostic/action,
and Rust allocation/Python GC remain zero. Three complete process runs reproduce
the same mechanism and consequence result. Plant
consequence remains rejected: sagittal green rows survive and handle/5 ms moves
1.340 s later, but left/right boundaries move up to 0.680 s earlier. The run
set records 115–126 deadline misses, 6.620–6.777 ms controller maxima, and
6.950–7.117 ms loop maxima. Reproduce with
`scripts/run-upkie-inexact-terminal-chooser-ab.sh`.

R192 audits that chooser at the model-to-plant boundary. For all 481 selected
actions it compares exact candidate root-angular/joint qdd with the following
5 ms measured MuJoCo velocity difference. Leave-one-named-case-out
componentwise maxima cover only 96.881% of complete samples; a 5% reserve
reaches 97.089% and still misses by 25.234×. The global realization envelope is
rejected. Reproduce with
`scripts/run-upkie-terminal-realization-calibration.sh`.

R193 replaces the fictitious zero-qdd withhold prediction with a separate
no-contact fixed-zero-effort floating-WBC solve. Its state-local mechanism,
replay, independent re-audit, and allocation gates pass, but plant consequence
rejects the unchanged Pareto ordering: handle/5 ms falls 1.575 s before exact
control. Reproduce with
`scripts/run-upkie-inexact-zero-effort-baseline-ab.sh`.

R194 audits that corrected zero-effort candidate on 179 actual withhold
intervals. Physical contact remains double on 169 samples; terminal-pressure
error reaches 28.200, and leave-one-case-out bounds cover only 93.3% of
mirrored-right and 86.4% of handle. No-contact remains one named hypothesis,
not an unknown-contact substitute. Reproduce with
`scripts/run-upkie-zero-effort-realization-calibration.sh`.

R195 explicitly solves each withhold/retained/support-free effort under
none/left/right/double support and aggregates the four Rust terminal-score
hypotheses componentwise. Across 40,982 first-run ticks, every missing-evidence
tick is attempted: 266 envelopes are valid and four invalid baselines fail
closed to authority 0 and exact zero torque. Mechanism gates pass, but only
23/266 following plant intervals are fully bracketed; maximum component
exceedance is 267.523. Handle/5 ms is 1.245 s earlier than exact and the run
misses 70 deadlines (6.578/5.793 ms loop/controller maxima). The support set is
diagnostic-only. Reproduce with
`scripts/run-upkie-inexact-support-hypothesis-envelope-ab.sh`.

R197 records physical wheel normal/tangential impulse and generalized
constraint impulse across each complete 5 ms plant interval, without changing
the controller or replay state. Physical support is the best of the four rigid
qdd hypotheses on 91.35% of 266 selected intervals, but their componentwise qdd
range covers 0/266. Best-mode qdd-error norm reaches 14,718.472 and tangential
impulse correlates `+0.907` with it. This identifies soft-contact/slip impulse
realization as the dominant missing state, but the completed-interval impulse
is noncausal and is not installed as a bound. Reproduce with
`scripts/run-upkie-contact-impulse-residual-audit.sh`.

R198 appends only the immediately previous interval's normal, tangential and
generalized constraint impulse to the frozen R196 state feature. Linear and log
encodings are fixed before strict leave-one-case-out testing. Neither helps:
kNN remains 85.34%, Lipschitz remains 91.35%, and maximum exceedance worsens
15.284→16.151. This rejects lagged force history as a conservative transition
state while preserving it as a possible observation diagnostic. Reproduce
with `scripts/run-upkie-lagged-impulse-conditioning.sh`.

R199 tests the broader causal pre-step state/model family against completed-
interval normal/tangential impulse and two five-millisecond velocity-jump
targets. No strict leave-one-named-case-out row passes. Global maxima reach
99.624% complete-sample coverage only with about 72 rad/s of allowance and
still miss; action, kNN, Lipschitz and physical-support oracle grouping cover
97.744%, 75.564%, 90.602% and 93.609%. Reproduce with
`scripts/run-upkie-causal-impulse-holdout.sh`.

R200 adds exact current pre-solve MuJoCo contact availability, signed distance,
and three signed constraint-coordinate relative velocities to the frozen R196
feature. Across 266 strict held-out samples, linear/log kNN coverage regresses
85.338→84.962%; Lipschitz remains 91.353%, misses by 15.086 and expands its p95
maximum-component bound to 39,751.621. The 247 contact-present and 19 flight
rows, complete metrics tree, and rejection repeat exactly. These plant fields
remain candidate-sensor oracle evidence, not online authority. Reproduce with
`scripts/run-upkie-contact-prestate-conditioning.sh`.

R201 replaces empirical residual fitting with a fixed-size Rust physical outer
bound over uncertain impact time, restitution, per-contact closing speed,
effective mass, sustained normal load, friction, candidate acceleration, and a
caller-owned `M⁻¹Jᵀ`. All six declared profiles cover both wheels' measured
normal/tangential impulse on 266/266 retained intervals. The tight row has a
0.703 N·s p95 normal bound at 18.0% p95 utilization; the primary 100 m/s²
reserve has 6.042 N·s at 1.7%. Rust p99 is 0.148 µs with zero allocation, and
the semantic result repeats exactly. This admits the mechanism only: the first
audit passes zero acceleration/response and exact prospective MuJoCo geometry,
so model-derived generalized velocity-jump coverage, useful width, fresh
morphology/friction/timing holdout, consequence, and authority remain open.
Reproduce with `scripts/run-upkie-contact-transition-interval-audit.sh`.

R189 sweeps a causal Rust minimum-improvement gate at
`0/0.001/0.005/0.025/0.1/0.25/1` plus an all-withheld endpoint. The gate
compares the selected reduced score only against the zero-authority score;
insufficient improvement installs zero authority. Across 154 profiles, paired
replay, 127,865 first-run ticks, and 1,792 selector queries, exact dormancy,
unavailable-only queries, matched 5/10 ms first-loss evidence, exact retained
effort, bounded expiry, withheld-endpoint equivalence, finite execution, and
zero allocation/GC all pass. No margin passes both plant slices. `0.001`
removes r188's green fall but advances one 5 ms and three 10 ms boundaries;
the withheld endpoint advances five. Two process repetitions retain 192–210
deadline misses, 8.847–9.448 µs selector maxima, 6.134–7.039 ms loop maxima,
and 5.699–6.645 ms controller maxima. Reproduce with
`scripts/run-upkie-inexact-hold-improvement-gate-ab.sh`.
### Revision r269 bounded contact continuation

`python/evals/g1_bounded_contact_continuation_r269.py` compares the retained
r268 low-gain native-reference trace with two regenerated r269 traces. It uses
no policy or physics simulator. The first trace enables exact-prefix reuse,
eight feasibility sweeps per solver query, and a 12-tick non-integrating hold.
The second independently exercises localized handoff under a centroidal
control profile. Both explicitly predeclare target 1/right foot as the first
normal-only fallback target; this is a causal diagnostic input rather than an
inferred contact-fault label.

The evaluator checks that status-8 ticks preserve q, v, root pose, tracked
points, and CoM bit-for-bit; that observable aggregate work remains within two
eight-sweep solver queries per WBC tick; that status 9 retains nonzero support
force with dynamics/contact residuals below 1e-8; and that every global release
clears rejected residual diagnostics. The low-gain case delays first global
release from 869 to 888 after retaining the locked left foot at tick 869. The
mechanism passes, but first NormalFallback is unchanged at 863 and full-run
root/attitude behavior remains red. Defaults and authority are unchanged.

Run `scripts/run-g1-bounded-contact-continuation-r269.sh` to rebuild the PyO3
extension, regenerate both traces, and render the Markdown/HTML decision
report. The retained artifact is
[`G1_BOUNDED_CONTACT_CONTINUATION.md`](../benchmarks/results/g1-bounded-contact-continuation-r269/G1_BOUNDED_CONTACT_CONTINUATION.md).

### Revision r272 NormalFallback point-task scale causal split

`python/evals/g1_normal_fallback_task_scale_r272.py` compares the retained R270
trace with scale 0, 0.25, and 0.5 controls on the existing viability point task
that remains after measured `NormalFallback`. The scale is default-preserving,
adds no rows or solver work, and uses no policy or physics simulator.

The mechanism is partial but rejected. The dormant scale-1 replay is bitwise
equal to R270 on 72 non-timing arrays, and every reduced-scale trace is bitwise
equal to baseline through fallback tick 875. All profiles therefore hit the
knee limit at tick 874 with the same −4.097 rad/s critical-window minimum; the
different later full-trace minima are downstream consequences, not causal
improvements. Scale 0.25 stays under 5 ms p99 but worsens root RMS
to 17.030 m; scale 0.5 improves root RMS to 14.691 m but crosses p99 at
5.162 ms. No scale is admitted.

Run `scripts/run-g1-normal-fallback-task-scale-r272.sh` to rebuild the PyO3
extension, regenerate the dormant and three scale traces, and render the
Markdown/HTML decision report. The retained artifact is
[`G1_NORMAL_FALLBACK_TASK_SCALE_R272.md`](../benchmarks/results/g1-normal-fallback-task-scale-r272/G1_NORMAL_FALLBACK_TASK_SCALE_R272.md).

### Revision r271 aggregate support-load floor falsifier

`python/evals/g1_support_load_floor_r271.py` compares the retained R270
lower-body hard-envelope trace with default-off aggregate support-load floors
of 0.5%, 10%, and 25% of supported weight, divided evenly over active patches.
Rust emits one hard linear inequality over the sum of each patch's normal-force
slots, so a four-point sole may redistribute load without silently accepting an
underloaded support. A separate zero-floor trace must reproduce all non-timing
R270 arrays bit-for-bit. The evaluator decodes compact force slots in active
target order and checks every non-release/non-hold patch row within 1e-6 N. The
replay uses no policy and no physics simulator.

The generic mechanism passes—72/72 dormant semantic arrays match and every
enabled row has zero violations—but the global walking profile is rejected:
first fallback moves 875→505→424→308 as the floor increases, with support
release immediately following the floor contingency. Full root RMS reaches
24.382–28.960 m. This makes the prior
42–58 N status-4 load trade explicit and fail-closed, but does not restore useful
walking authority. The API defaults to zero, and no authority is admitted.

Run `scripts/run-g1-support-load-floor-r271.sh` to rebuild the PyO3 extension,
regenerate the dormant A/B and three floor traces, and render the Markdown/HTML decision report.
The retained artifact is
[`G1_SUPPORT_LOAD_FLOOR_R271.md`](../benchmarks/results/g1-support-load-floor-r271/G1_SUPPORT_LOAD_FLOOR_R271.md).

### Revision r270 lower-body velocity-envelope causal split

`python/evals/g1_lower_body_velocity_envelope_r270.py` compares the retained
r268 and r269 traces with dormant, soft-only, hard-only, soft+hard, and r269
composition controls. It uses no policy and no physics simulator. The Rust
session derives a lower-body-only velocity-envelope mask once from the pinned
URDF names and can independently apply the soft viability task and a hard
directional braking bound. A conflicting hard interval is carried to the solver
to fail closed rather than silently skipped. The default session and authority
boundary are unchanged.

The causal mechanism checks pass: no early envelope activity appears;
hard-only keeps the right-knee velocity above -8 rad/s and moves first fallback
863→888; soft-only moves it only to 864. The much longer first-release delay to
tick 1108 appears only in the soft+hard interaction. The combined standalone
timing distribution is under the 5 ms p99 gate, but composition with r269
releases at tick 897 and exceeds the timing gate. The walking profile is
deliberately rejected: full root RMS remains 15.514 m, the knee reaches its
position limit at tick 874, and support is released at the next transition.
This isolates continuous position-limit/support-transition recovery without
treating extra solver work as a fix.

Run `scripts/run-g1-lower-body-velocity-envelope-r270.sh` to rebuild the PyO3
extension, regenerate the candidate trace, and render the Markdown/HTML report.
The retained artifact is
[`G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md`](../benchmarks/results/g1-lower-body-velocity-envelope-r270/G1_LOWER_BODY_VELOCITY_ENVELOPE_R270.md).

### Revision r268 native-reference integrated bridge

`python/evals/g1_native_reference_integration_r268.py` is a report-only,
policy-free and physics-free comparison over three retained 2,317-tick
integrated traces. All consume the same immutable r53 Rust-LIPM reference and
the first state of the r54 morphology witness; future controller state is
integrated by Rust. The optional posture row allowlists only q/v/qdd witness
arrays and never consumes oracle WBC force, status, or solved acceleration.

Initialization-only extends the previous native-reference clean prefix from
265 to 501 ticks and is exact at liftoff, but remains touchdown-red. The R54
task-stack row fails at 188 and the morphology-posture row at 439. The report
pins both source hashes, accepted hard residuals, status/support transitions,
latency/deadline distributions, finite-state evidence, and explicit zero
policy/physics counts. It also checks that every status-5 release/fallback tick
has zero contact/dynamics residual witnesses after the rejected solve is
cleared; this is diagnostic hygiene, not a physical-support claim. A retained
low-gain morphology posture row (weight `0.05`) carries the first transfer
through touchdown to tick 863 with 1.84 cm root RMS, but remains red later and
is not promoted. Run
`scripts/run-g1-native-reference-integration-r268.sh` to rebuild all four
traces and the hosted report.
