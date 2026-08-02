# G1 DCM / clipped virtual-ZMP experiment — r33

## Decision

Revision r33 implements the dynamically consistent balance experiment proposed
by r32, but does **not** make it the default walking policy. The Rust law fixes
the first support-centre/CoM category error and materially reduces the first
swing divergence. It does not yet complete the 600-tick CMU transfer: the
measured DCM eventually leaves the recoverable support region, the incoming
foot misses measured touchdown, and the remaining support enters typed
normal-contact contingency. The experiment is retained as a stronger base for
the next controller/reference iteration, not presented as sustained walking.

The unchanged deterministic toe-step and moving-liftoff regressions remain
combined green. CUDA remains deferred.

## CPU implementation

`bonesaw-core::dcm_balance_acceleration` is a pure, allocation-free tick
operation. It accepts measured CoM position/velocity, a desired DCM jet, up to
sixteen established material support points, and explicit configuration. It:

1. derives the LIPM natural frequency from measured CoM height,
   `omega = sqrt(g / h)`;
2. observes DCM as `xi = com + com_velocity / omega`;
3. closes desired DCM velocity with
   `xi_dot_cmd = xi_dot_ref - k (xi - xi_ref)`;
4. computes virtual ZMP as `zmp = xi - xi_dot_cmd / omega`;
5. constructs the convex hull of measured support anchors in fixed stack
   storage, erodes every edge by the configured metric margin, and projects the
   virtual ZMP to that polygon;
6. emits horizontal CoM acceleration
   `com_acceleration = omega^2 (com - clipped_zmp)` with a declared norm cap.

DCM control uses only the two horizontal CoM Jacobian rows. Root height remains
owned by the existing invariant root-height task, avoiding the redundant
vertical CoM row found in the first smoke experiment. Invalid margins and
degenerate polygons are rejected. Fallen traces remain observable: measured
height is explicitly reported and the natural-frequency calculation uses a
declared floor rather than aborting the batch.

Four unit tests cover an interior stabilizing command, metric polygon erosion
and projection, an empty eroded set, and height-floor failure telemetry.

## Fixed-shape telemetry

The single Rust `run_trace` call now writes all of the following into
caller-owned arrays without per-tick Python callbacks or objects:

- measured CoM position and finite-difference velocity;
- measured and target DCM;
- raw and support-clipped virtual ZMP;
- commanded CoM acceleration;
- measured-height natural frequency and measured CoM height;
- support-hull vertex count, ZMP-clipped flag, and height-floor flag.

The NPZ archive retains every array. JSON and Markdown report DCM RMS/p95, ZMP
clip fraction and distance, natural-frequency range, height-floor ticks,
horizontal command percentiles, and support-hull size.

## Reference construction

Python eval land retains the reference-policy experiments. Two strategies were
predeclared:

- `support-centroid-preview`: the r32 future-support average, reinterpreted as
  a DCM target rather than a CoM target;
- `dcm-backward-preview`: a support-centre ZMP sequence integrated backward
  through the unstable DCM dynamics from the terminal support state.

For the backward policy, each discrete reference satisfies
`xi[k] = zmp[k] + exp(-omega[k] dt) (xi[k+1] - zmp[k])`, and its velocity is
`omega[k] (xi[k] - zmp[k])`. Rust still owns the online measured feedback,
support polygon, clipping, WBC task, solve, integration, and diagnostics.

## Full-horizon first 250 ticks

These values are sliced from the full retained traces, so future precontact
lookahead is identical to the eventual 600-tick run. This is stricter than a
standalone 250-tick execution, which cannot see the authored contact edge 428
ticks into the source.

| Policy | Root RMS | stance RMS | swing RMS | max root rotation | p99 |
|---|---:|---:|---:|---:|---:|
| r32 static support-centre CoM servo | 2.162 cm | 0.0003 cm | 9.613 cm | 0.975° | 5.280 ms |
| r32 no-CoM causal control | 0.381 cm | 0.0001 cm | 1.683 cm | 2.842° | 2.834 ms |
| r33 backward DCM + clipped ZMP | 2.773 cm | 0.0002 cm | 1.821 cm | 2.001° | 5.454 ms |

The r33 DCM trace records `0.966 cm` DCM RMS over this window. ZMP clipping is
active on `7.6%` of ticks with `0.571 cm` clip-distance RMS. There are no
fallback, infeasible, or failed ticks; the final 22 ticks are the expected
measured precontact state. This proves that the new law removes most of the
first swing divergence while preserving honest support semantics. It does not
prove the full transfer.

A standalone 250-tick support-centroid DCM diagnostic records `0.075 cm`
swing RMS and `3.192 ms` p99. It is retained for controller-kernel inspection
only: truncation removes the future contact edge and therefore the precontact
task. It is deliberately excluded from acceptance claims.

## Full 600-tick result

| Metric | r32 static servo | r33 backward DCM |
|---|---:|---:|
| functional gate | FAIL | FAIL |
| 5 ms p99 gate | PASS · 4.795 ms | FAIL · 48.250 ms |
| root RMS | 20.026 cm | 30.346 cm |
| stance-foot RMS | 28.940 cm | 51.837 cm |
| swing-foot RMS | 27.249 cm | 47.778 cm |
| max root rotation | 13.921° | 64.219° |
| denied touchdown ticks | 172 | 172 |
| normal-contact contingency ticks | 0 | 173 |
| rejected / infeasible ticks | 0 | 0 |

The r33 full trace records DCM RMS/p95 `43.355/87.305 cm`, ZMP clipping on
`61.5%` of ticks, `67.502 cm` clip-distance RMS, a `3.573–5.134 rad/s`
natural-frequency range, and zero height-floor ticks. Horizontal command p95
and maximum are `11.449/14.175 m/s²`. The right sole remains the measured
support; the left enters Precontact at tick 228 but never satisfies touchdown
admission, and the right transitions to NormalFallback at tick 427.

The dense latency tail is a consequence of prolonged clipped semantic work and
contingency, not the support-hull kernel. Default green regressions below show
that adding the dormant fixed-shape path does not change the accepted CPU path.

## Default regressions

| Corpus | Combined | Root / CoM-observation / stance / swing RMS | p99 | status |
|---|---:|---:|---:|---|
| deterministic 1 cm G1 toe-step | PASS | 3.567 / 4.445 / 0.002 / 0.639 cm | 4.020 ms | zero contingency, infeasible, or failed ticks |
| official-G1 moving liftoff | PASS | 0.748 / 1.561 / 0.000 / 0.173 cm | 3.844 ms | zero contingency, release, infeasible, or failed ticks |

Both regressions keep DCM disabled and reproduce the r32 non-timing metrics.

## Rejected variants

| Variant | Evidence | Decision |
|---|---|---|
| rooted CoM jet interpreted through DCM | no capture shift before synthetic liftoff; ZMP immediately clips to the lone support and the model falls away | a capture reference must anticipate contact changes |
| direct 40-tick support-centroid DCM | excellent short prefix, then diverges after the first single-support interval | future averaging is not a DCM dynamics plan |
| direct 80-tick support-centroid DCM | 400-tick root/swing RMS 28.793/45.728 cm and 63.76° rotation | spreading an arbitrary average does not make it dynamically consistent |
| DCM at Invariant priority | falls before the full horizon | balance is not allowed to override rigid contact and attitude invariants |
| CoM/DCM weight 1.0 | improves short DCM residual but sacrifices swing and later diverges | do not hide hierarchy conflict in a same-level weight |
| feedback gain 1.75 or 7.0 s⁻¹ | trades swing against attitude and worsens the 300-tick combined behavior | gain is not the remaining root cause |
| shorten precontact from 200 to 80 ticks | 94 failed ticks, 170 height-floor ticks, and 173.06° rotation | the earlier bounded landing path is stabilizing despite competition |
| cadence 0.5× / 0.75× | falls below the measured support plane before 600 ticks | terminal-horizon and capture planning must be evaluated over complete steps, not rescued by cadence alone |

## Next experiment

The next controller slice should couple capture planning and swing reachability
instead of optimizing them independently:

1. generate the DCM/ZMP reference over complete contact phases with an explicit
   terminal capture state beyond the scored horizon;
2. expose measured DCM viability margin to the support polygon and use it to
   delay liftoff when the next single-support state is not capturable;
3. retime the swing/precontact trajectory from the same phase duration and
   measured reachability envelope;
4. keep r31 measured touchdown admission and outgoing-support retention
   unchanged;
5. require the existing toe-step/liftoff regressions, full transfer gates,
   hard-row `1e-8` contract, and 5 ms p99 deadline.

## Retained artifacts

- authoritative full r33 trace:
  `benchmarks/results/g1-transfer-r33-dcm-backward-final`
- unchanged toe-step regression:
  `benchmarks/results/g1-synthetic-step-r33-dcm-regression`
- unchanged moving-liftoff regression:
  `benchmarks/results/floating-g1-liftoff-r33-dcm-regression`
- standalone kernel prefix:
  `benchmarks/results/g1-transfer-r33-dcm-prefix`
- direct-preview and priority/gain controls are retained under
  `benchmarks/results/g1-transfer-r33-*`.
