# CPU concept baseline

Captured 2026-07-31 with:

- AMD Ryzen 7 3700X, 8 cores / 16 threads;
- x86-64;
- Rust 1.95.0;
- Cargo `--release`;
- 5,000 ticks per scenario after 250 warmup ticks;
- process not CPU-pinned.

| Scenario | RMS error | Max error | p50 tick | p99 tick | Contingency |
|---|---:|---:|---:|---:|---:|
| Moving bimanual reach | 0.63 cm | 24.40 cm | 40.2 µs | 49.9 µs | 0 / 5,000 |
| Bimanual priority conflict | 0.48 cm | 17.86 cm | 90.1 µs | 124.0 µs | 4 / 5,000 |
| CMU walking-motion retarget sentinel | 4.05 cm | 5.55 cm | 67.1 µs | 76.4 µs | 0 / 5,000 |

Additional sentinels:

- capture-aware G1 landing experiment: the allocation-free Rust policy bounds
  the sole-center anchor by authored offset, future-root reach, and slew before
  freezing it for measured touchdown admission. The stable 8 cm case extends
  the clean CMU-transfer prefix from 472 to 482 ticks and cuts DCM RMS from
  33.520 cm to 18.131 cm, but the physical touchdown gate correctly remains
  closed. The planner is disabled by default; both short floating regressions
  remain combined green.

- isolated data-backed CMU 37/01 walking retarget: fixed pelvis, 5,000 ticks,
  0.75×/1.0×/1.25× cadence, 0% reference flight and 25.66% double support.
  Bonesaw p50/p99 is `67.1/76.4 µs` with `5.549 cm` foot and `1.411 cm`
  hand RMS, zero contingency/rejected ticks, and zero native controller
  allocations. It fails only the predeclared `≤5 cm` overall foot gate.
  PlaCo records `3.701 cm` foot RMS but fails the `≤3 cm` swing-clearance RMS
  gate at `3.301 cm`. See the raw per-step comparison artifacts for cadence,
  jitter, memory, CPU, GC, and transition distributions.

- joint-position limit violation: `0` in all three scenarios;
- replay: bitwise identical for 500 ticks;
- current cached frame queries: 2.36 million queries/second over 500,000
  queries.
- historical rooted-atlas queries: 821 thousand queries/second over 50,000
  queries, including interpolation, FK, atlas evaluation, and pairwise query;
- randomized dynamics oracle: inverse/forward maximum error `4.06e-14`,
  minimum mass-matrix eigenvalue `1.62e-2`, p99 `43.9 µs` over 200 states;
- collision differential: 15 conservative sphere proxies, 97 self-pairs,
  analytic/finite-difference Jacobian max error `5.12e-10`, p99 full scan
  `37.3 µs`;
- hard constraints: zero measured violation, zero active-task residual,
  contradictory half-spaces typed `PrimalInfeasible`, p99 `1.17 µs`;
- compiled signal-jet graph: 13 scalar/vector nodes, two outputs, three
  explicit state slots, bitwise-repeatable over 5,000 steps, p50 `0.261 µs`,
  p99 `0.330 µs`, max `3.076 µs`, and `0.0` allocation calls/bytes per
  evaluation;
- compiled signal-to-task rig: six signal nodes, three resolved
  point/CoM/orientation task slots, complete controller and trajectory
  transition over 5,000 ticks, `0.9214 cm` point and `0.1929°` orientation
  tracking RMS, bitwise repeatability, p50 `74.0 µs`, p99 `102.4 µs`, max
  `258.9 µs`, and `0.0` allocation calls/bytes per tick;
- Pinocchio 4.0 differential oracle: 50 states each for Upkie and the official
  23-DOF G1, covering 2,050 + 1,500 body-frame samples and every fixed/floating
  rigid-body product. Maximum errors are `2.132e-14` / `1.705e-13`, all below
  strict thresholds;
- controller-path allocation sentinel: `0.0` calls and `0.0` bytes per tick
  for reach, conflict, walking, and collision-enabled control across all 97
  compiled self-pairs.
- unified inverse-dynamics WBC: 42 variables (`18 q̈ + 18 τ + 2×3 contact
  forces`) over 5,000 ticks, zero infeasible ticks, bitwise repeatability,
  maximum dynamics residual `1.04e-13`, contact-acceleration residual
  `5.13e-18`, friction active on 4,674 ticks, p50/p99/max
  `687.1/714.0/934.3 µs`, and `0.0` allocation calls/bytes per tick.
- Upkie floating inverse-dynamics WBC: 24 variables
  (`6 root q̈ + 6 joint q̈ + 6 τ + 2×3 contact forces`) over 5,000 ticks,
  zero infeasible ticks, bitwise repeatability, maximum dynamics residual
  `5.14e-12`, contact-acceleration residual `1.82e-14`, p50 `104.5 µs`, p99
  `132.4 µs`, max `149.3 µs`, and `0.0` allocation calls/bytes per tick.
- raw-integrated Upkie squat: 5,000 steps at 200 Hz, the full `12 cm` root
  command, maximum constrained rolling-contact slip `5.93e-6 m`, 1.73 cm
  permitted wheel-axis travel, zero infeasible ticks, p50 `198.0 µs`, p99
  `230.0 µs`, max `375.5 µs`, and zero allocation calls/bytes across IK
  reference generation, balance cascade, centroidal refinement, contact
  stabilization, compiled four-node/three-task floating policy, WBC, and SE(3)
  integration. A compiled 1 Hz critically damped root-translation spring shapes
  the exact linear editor command profile. Compiled root/CoM acceleration
  matches the direct formula oracle to `5.55e-17`. The default browser path
  now runs that same raw controller and advances only through WBC output and
  SE(3) integration. `BONESAW_LIVE_GUIDED=1` restores the contact-consistent
  pose handoff as an explicitly labelled diagnostic preview. The floating
  solve publishes twelve fixed task-diagnostic slots (eight semantic base plus
  four Cartesian point slots) with physical-unit L2/RMS residuals, activity,
  priority, and clipped-level membership.
- official Upkie controller oracle: 100,000 sequential 5 ms samples through
  the pinned upstream `WheelBalancer.cpp` class and the Rust typed law. With
  upstream parameters, both wheel commands have zero canonical bit mismatches.
  The live-tuned law reports 0.1177 m/s RMS delta and 0.9951 correlation,
  p50/p99 `0.030/0.031 µs`, and zero hot-loop allocations; the official-parameter
  Rust law is `0.050/0.060 µs`, while upstream C++ dictionary read/write reports
  p50/p99 `1.372/1.643 µs`.
- fixed/floating Pinocchio oracle: CoM, centroidal map/momentum, floating mass,
  bias, and inverse dynamics all pass across 50 Upkie states; maximum floating
  error is `2.132e-14`.
- 18-DOF toy floating-contact profile: 48 variables, zero infeasible ticks,
  acceleration-tracking RMS `1.06e-2`, maximum acceleration `3.72e-2`,
  dynamics/contact residuals `5.14e-10` / `5.19e-11`, p50 `0.650 ms`, p99
  `0.669 ms`, max `0.990 ms`, bitwise repeatability, and zero allocation
  calls/bytes.
- official-G1 moving liftoff: 260 strict 5 ms ticks, combined PASS with
  root/stance/swing RMS `0.748/0.000/0.173 cm`, zero fallback/release/rejected
  ticks, and dynamics/contact residuals `1.22e-9` / `5.16e-11`. Revision r30's
  cached-energy Jacobi kernel records p50/p99 `2.320/3.904 ms`. State deltas and
  the intentional floating-point ordering change are reported as an
  algorithmic reacceptance rather than described as byte parity.
  The detailed report includes p99.99, jitter, deadlines, per-status timing,
  ten windows, RSS/CPU/fault/context-switch deltas, Python trace memory, and GC.
- official-G1 support-preview transfer stress: revision r30 has zero fallback,
  release, infeasible, or failed ticks and passes the CPU gate at `4.808 ms`
  p99, but root/stance/swing RMS `23.670/26.328/28.588 cm` and `9.339°` root
  rotation keep the full 600-tick trace behaviorally red. The default transfer
  policy remains still worse. This is a controller/reference-design failure,
  not a feasibility or deadline pass.
- revision r31 measured-touchdown control: the unchanged toe-step remains
  combined green at `4.040 ms` p99. On the 600-tick CMU stress, the incoming
  foot never satisfies the `2.5 cm` / `0.20 m/s` position/tangential/normal
  admission envelope; 172 requested-contact ticks remain Precontact and the
  outgoing right support remains Locked. There are zero fallback, release,
  infeasible, failed, or GC events, and p99 is `4.763 ms`. This is the intended
  safe failure: the trace fails its new admission-delay gate instead of
  manufacturing contact at a mid-air measured pose.
- revision r32 actual-CoM observation and causal control: the default toe-step
  and moving-liftoff profiles remain combined green at `4.125 ms` and
  `3.852 ms` p99. The 600-tick CMU support-preview stress remains CPU-green at
  `4.795 ms` p99 but records root/CoM/stance/swing RMS
  `20.026/26.808/28.940/27.249 cm`. Over its first 250 ticks, removing only the
  invalid support-centre CoM task reduces root/CoM/swing RMS from
  `2.323/3.550/8.921 cm` to `0.400/1.160/0.360 cm` and p99 from `4.737 ms` to
  `2.590 ms`. The no-CoM full trace later loses balance; this is a causal
  controller diagnosis, not an accepted replacement.
- revision r33 DCM/virtual-ZMP experiment: the fixed-capacity Rust hull,
  erosion, projection, DCM feedback, and added fixed-shape diagnostics leave
  the default toe-step and moving-liftoff combined green at `4.020 ms` and
  `3.844 ms` p99. On the full-horizon first 250 CMU ticks, swing RMS falls from
  `9.613 cm` under the r32 static servo to `1.821 cm`, but the full 600-tick
  r33 trace reaches 173 normal-fallback ticks and `48.250 ms` p99. This is an
  explicit rejected-policy result; the CPU deadline failure comes with the
  later clipped/contingency tail, not the dormant hull kernel.
- revision r34 task-authority ablation: the new dormant velocity-envelope path
  leaves defaults unchanged. At Viability priority, weight `0.10` postpones the
  first contingency from tick `427` to `467`, cuts DCM RMS from `43.355 cm` to
  `17.235 cm`, and has zero rejected ticks, but its contingency tail raises p99
  to `102.162 ms` and root attitude reaches `179.895°`; it is not a new CPU
  baseline or accepted controller policy.
- revision r35 measured-phase authority remains dormant by default. The
  deterministic toe-step and moving-liftoff regressions are combined green at
  `4.100 ms` and `3.973 ms` p99. The best experimental 200-tick release reaches
  `10.831 cm` DCM RMS but `112.259 ms` p99 in its contingency tail and is not an
  accepted controller or CPU baseline.
- revision r36 feedback authority and signed DCM-margin telemetry are dormant
  by default. Toe-step and moving-liftoff remain combined green at `4.052 ms`
  and `3.906 ms` p99. Experimental feedback tails remain deadline-red and are
  not promoted into the CPU baseline.
- revision r38 changes only Python evaluation-reference semantics: bounded DCM
  preview and canonical-cycle lateral registration. It adds no hot-loop Rust
  allocation or new controller task. The corrected 200-tick full-transfer
  stress is explicitly deadline-red in its contingency tail (181.5–183.9 ms
  p99 across three behaviorally exact repeats). Moving liftoff remains combined
  green at 4.125 ms p99. The synthetic toe-step remains functionally green in
  five repeats, with 4/5 under 5 ms and a 4.623 ms median p99; the retained
  scheduler-tail run reaches 5.904 ms p99.
- revision r39 adds scalar Rust phase/cadence policy work plus fixed-shape
  coupled-reference sampling. On the isolated 240-tick 6 cm / 100 ms touchdown
  stress, three retimed runs are behaviorally bit-for-bit exact and record p99
  4.698/4.688/4.944 ms, thread CPU 630.1/630.2/633.7 ms, zero Python GC
  collections, and 1,424 B Python traced-call peak. The mechanism reduces the
  touchdown transition from a failing 10 ticks to a passing 3 ticks. These are
  acceptance results for the isolated phase policy, not a new sustained-CMU
  baseline: the H=200 transfer remains behaviorally and deadline red.
- revision r40 adds an opt-in scalar DCM-margin phase-rate law and no promoted
  CPU baseline. The disabled path is exact across 71 shared non-timing arrays.
  Smooth engagement improves the clean CMU/G1 prefix from 482 to 490 ticks and
  swing RMS from 81.2 cm to 46.3 cm, but the best contact-edge state is still
  0.639 m / 6.044 m/s and an 800-tick extension diverges. Contingency tails are
  deadline-red (the 0.5×-floor 600-tick p99 is 208.715 ms), so the policy remains disabled.
  The causal report retains per-case latency, jitter, thread CPU, RSS delta,
  GC, solver work, transition timing, and raw traces.
- the r40 open-loop reference contract is deliberately outside the CPU baseline:
  it executes neither Bonesaw policy nor physics. It shows the 0.35× CMU/DCM
  reference is already red at 85.37 m/s² peak CoM acceleration, friction ratio
  8.276, and 1.683 m/s pre-edge normal foot speed. These are reference-generation
  defects to resolve before another WBC timing comparison.
- the r41 PlaCo WPG oracle is also upstream of the WBC CPU baseline. Its matched
  600-tick first-step reference is bitwise repeatable and records approximately
  0.9 s one-shot planning plus 18 ms sampling in the isolated Python/PlaCo/
  Pinocchio process. Peak process RSS is about 220 MiB and includes import,
  model, and setup. These are reference-generator measurements, not Rust servo
  costs. Eight of nine open-loop gates pass; the exact liftoff CoP boundary is
  still red.
- the r42 Rust LIPM planner is the first reference admitted by that upstream
  gate. The retained release worker plans the matched boundary in 29.3 µs and
  samples 600 ticks in 99.5 µs, with zero allocation calls in both measured
  regions and bitwise repeat. These are reference-generation costs, not WBC
  servo costs; closed-loop tracking and physics remain separate downstream
  suites.
- r43 smooths the internal CoP transfer over 100 ms and bounds the official-G1
  landing reach at 0.800 m. The retained open-loop trace remains 9/9 green,
  reduces sampled jerk to 31.8 m/s³, and still reports zero allocation calls.
  Its first immutable WBC admission is intentionally recorded separately and
  remains red after a 265-tick nominal prefix; contingency-tail timing is not a
  reference-planner CPU cost.
- r44 adds a separate policy-free, physics-free morphology and algebraic WBC
  admission baseline. Whole-body position IK is p50/p99 about 1.3/19.4 µs,
  analytic velocity/acceleration projection is about 28.2/33.3 µs, and the
  23-DOF floating inverse-dynamics solve is about 2.39/4.32 ms across 600
  independent oracle states. Each measured Rust region reports zero allocation
  calls. All states solve; maximum hard dynamics/contact residuals are
  1.22e-9/4.88e-11 and physical WBC outputs repeat bit-for-bit. This is an
  algebraic admission cost, not a claim about closed-loop physics robustness.
- r45 adds allocation-free finite-support CoP rows to that stateless solve.
  The 5 mm eroded-sole contract passes 27/27 gates over all 600 oracle states:
  p50/p99 is about 2.44/3.81 ms, hard dynamics/contact residuals remain
  1.22e-9/4.88e-11, and the WBC hot loop still allocates zero bytes. A retained
  10 mm sensitivity remains feasible but reaches 0.597/0.313 frame-angular/
  point RMS, deliberately documenting the current support-margin/tracking
  frontier rather than relaxing the tracking gate.
- r72 and r74 are exact-order address/layout promotions over the later
  four-step G1 CPU admission. R72 reduces the 58-variable native marginal
  sentinel from 22.022 M to 15.807 M instructions/tick by slicing established
  Jacobi columns. After re-profiling, r74 slices general dense-product rows and
  reduces it again to 15.317 M instructions/tick. The r74 complete-process
  corpus retains 56/56 exact non-timing fields, 43/43 admission, zero
  allocation, and unchanged 16,809 inverse / 122,031 sweep / 10,604 clip work;
  five pinned pairs reduce instructions by 4.104% and process p50/p99 by
  2.19%/1.94%. These are algebraic CPU costs, not physics-rollout claims.
- r75 adds a separate batch-mirror CPU baseline for the first future-CUDA
  manifest; it does not change or replace the dynamic-WBC baseline above.
  Preallocated Rust StateInput/FK/CoM execution on Upkie records p50/p99
  `6.732/8.577 µs` at one agent, `191.101/211.130 µs` at 32, and
  `1.985/2.648 ms` at 256 over 300 calls per size, with zero execute
  allocations. These timings exclude Python/NumPy↔SoA copy and websocket or
  device transport. The CPU mirror passes independent Pinocchio f64 and exact
  reindexing/isolation gates; no CUDA timing or correctness baseline exists.
- r76 extends the separate mirror baseline through floating frame-origin and
  CoM Jacobians without changing the deployed f64 dynamic-WBC baseline. Upkie
  FK+Jacobian combined p50/p99 is `20.238/26.782 µs` at one agent,
  `492.825/662.543 µs` at 32, and `4.186/5.949 ms` at 256 over 300 calls per
  size. The binding detaches the GIL around both Rust stages. Rust execution
  allocates zero bytes; NumPy marshalling remains outside the timer, and the
  retained 256-agent scheduler tail is not a throughput claim. Pinocchio and finite-difference `J·v` gates pass for toy
  and Upkie, with exact batch reindexing and malformed-agent isolation. There
  is still no CUDA baseline.
- r77 extends the mirror baseline through floating M/h/Ag without changing the
  deployed f64 dynamic-WBC baseline. Upkie FK+Jacobian+dynamics combined
  p50/p99 is `49.784/57.391 µs` at one agent, `1.379/1.727 ms` at 32, and
  `11.417/11.873 ms` at 256 over 200 calls. The complete Rust pipeline
  allocates zero bytes and the untrimmed p99 tail is retained. Independent
  Pinocchio CRBA/RNEA/CCRBA checks, exact batch reindexing/isolation, mass
  symmetry, and positive-definiteness gates pass for toy and Upkie. This is a
  CPU mirror baseline only; CUDA execution remains unavailable and NOT RUN.
- r78 adds four fixed Upkie point sites after the admitted model products.
  Their point-only p50/p99 is `1.182/1.224 µs`, `25.098/30.250 µs`, and
  `213.178/251.488 µs` at 1/32/256 agents; full
  FK+Jacobian+dynamics+points p50/p99 is `52.539/60.353 µs`,
  `1.416/1.470 ms`, and `11.720/11.952 ms`. These are 200-call untrimmed Rust
  stage traces excluding NumPy copies. Pinocchio placement/Jacobian and f64
  central-difference `Jv`/`Jdot-v` gates pass on toy and Upkie, all batch
  invariants are exact, and allocation remains zero. CUDA is still NOT RUN.
- r92 does not change these CPU timing baselines. It adds a generic packed-tree
  StateInput→FK/CoM CUDA executor and a 200-call CPU semantic witness. The local
  NVRTC library is unavailable and the CUDA runtime reports no device, so no
  device latency, memory, D1, or D2 number is substituted into this section.
- r93 also leaves these CPU timing baselines unchanged. It extends the CUDA
  source/executor boundary through floating frame-origin and CoM Jacobians and
  adds a 200-call complete-pipeline CPU semantic witness. Both NVRTC probes are
  unavailable and the runtime reports no device, so no device latency, memory,
  D1, or D3 number is substituted into this section.
- r94 likewise leaves the measured CPU timing baselines unchanged. It extends
  the CUDA source/executor boundary through floating M/h/Ag and adds a 200-call
  complete-product CPU semantic witness. All three NVRTC probes are unavailable
  and the runtime reports no device, so no device latency, memory, D1, or D3
  number is substituted into this section.
- r95 also leaves the measured CPU timing baselines unchanged. It extends the
  one-stream CUDA source/executor boundary through four fixed point
  position/J/Jdot-v sites and adds a 200-call complete point-product CPU
  witness plus the retained f64 oracle. All four NVRTC probes are unavailable
  and runtime reports no device, so device latency, memory, D1, and D3 remain
  unmeasured rather than inferred.
- r96 retains those CPU product timings and adds fixed point-task/contact row
  emission after the point products. Its 200-call emitted-row replay is byte
  exact and an independent formula reconstruction has zero maximum error;
  malformed inputs, neighbors, modes, inactive rows, and padding pass. All five
  NVRTC probes remain unavailable and runtime reports no device, so no device
  latency or D1/D3 value is substituted.

Program fingerprint:
`0d247d0c1e76788ba8b78f55b78ed2722d9fa87710c7ab694805b51981730404`.

The maximum reach errors include the deliberate initial step from the standing
pose to the moving target. “Degraded” ticks include any solver level whose step
is clipped by a bound; they are not equivalent to hard-constraint failure. The
raw browser now reports clipped priorities with the largest physical RMS at
each level separately from the largest task residual across all twelve named
floating-base task slots.

These figures validate ample CPU headroom for the small 18-DOF concept. They are
not worst-case execution-time claims. Fixed-capacity task/constraint storage,
flat row-major solver matrices, reusable collision Jacobians, analytic extrema,
and flat trajectory sample blocks remove measured heap traffic from the
controller transition. The in-place Jacobi pseudoinverse is checked
differentially against the established matrix kernel and nalgebra's SVD, plus
all four Moore–Penrose identities on rank-deficient cases.
