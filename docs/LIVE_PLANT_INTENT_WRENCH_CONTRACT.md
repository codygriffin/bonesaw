# Live plant intent, feedback, and wrench contract

The browser exposes two different causes of motion and keeps their evidence
separate:

| input | owner | meaning | physical effect |
| --- | --- | --- | --- |
| green target / `TARGET` while dragging | Rust `/ws` | draft desired pose or end-effector intent | state-local preview only; no MuJoCo write |
| torso release / `plant_target_commit` | Rust `/plant-ws` → Python MuJoCo → Rust WBC | bounded world-frame root target | measured-state quintic root trajectory, then endpoint hold; never a qpos/qvel write |
| orange Ctrl-drag / `PUSH` | Python MuJoCo through Rust `/plant-ws` | external world-frame wrench | applied to the selected MuJoCo body and point for its expiring lease |

The live plant loop is a measured-feedback loop:

```text
MuJoCo qpos/qvel/root pose
        ↓
Rust WBC observation (50 Hz)
        ↓
torque command
        ↓
MuJoCo (5 × 4 ms physics steps)
        ↓
measured state and simulator witnesses
```

The WBC never integrates its own plant proxy in this mode. Each solve starts
from the latest MuJoCo root pose/twist and joint position/velocity. A committed
torso target is converted to a bounded C2 root trajectory in the worker and
presented as a low-priority root/joint task on that measured solve. The target
does not write MuJoCo state, does not replace contact or balance authority, and
is held at its endpoint until another target or reset. A wrench remains a
separate disturbance to balance against, not an intent target. The streamed
record labels target phase (`executing`, `holding`, or `rejected`), target
progress/error, measured state, wrench provenance, and WBC admission
independently.

Contact authority follows the same measured-state boundary. The worker derives
the two wheel subtrees once, refreshes MuJoCo collision data after each 4 ms
integration step, and writes a fixed five-row caller-owned 250 Hz contact
window. The 50 Hz WBC consumes only the final completed row from the preceding
window (the startup observation is frame zero); frame indices, per-substep
loss/gain edges, and the terminal physical mask are streamed so this one-tick
causal latency is testable. A transient loss inside a window is diagnostic for
that tick; the next boundary sees it only if the terminal mask is still lost.
The raw mask is passed to the persistent Rust adapter with an explicit
fresh-observation flag; Rust owns timestamp/provenance checks, three-sample
positive debounce, and immediate hard-row removal on contact loss. The stream
exposes raw, debounced, diagnostic hard, and executable hard masks as
`wbc_observed_contact_active`, `wbc_debounced_contact_active`,
`wbc_hard_contact_active`, and `wbc_hard_contact_executable`, together with
observation status/provenance/flags, support counts, and raw/admitted solver
status. A non-admitted result (including `MaxIterations`), reset, or paused
heartbeat cannot advertise retained hard rows as executable authority and
cannot turn the authored nominal two-wheel stance into support authority. This
is measured contact admission, not a claim that the WBC can execute a floating
walking transfer.

The evaluation-only R301 profile adds a separate measured-flight contingency
layer. `wbc_support_contingency_requested`, `..._admitted`, and
`..._selected` are distinct from primary WBC status; `..._mode`, support mask,
candidate residual, author/query timing, candidate/incremental power, and
forecast-guard state are diagnostics rather than hidden authority. On the
frozen R300 lateral-wrench trace this profile removes the two primary
`MaxIterations` ticks and delays the fall boundary by 520 ms, but it still
falls and is not enabled by the public worker.

R302 tightens the consequence contract without adding live authority. “No
automatic reset yet” is insufficient: recovery requires a ten-tick dwell with
both measured wheels, root height at least 0.48 m, tilt at most 0.20 rad, and no
pending reset. A low-body/no-wheel interval is counted separately. The best
candidate in the retained 160-profile calibration delays the terminal boundary
but fails both physical predicates, so it remains evaluation-only.

Simulation lifecycle commands are explicit and fail-safe:

- `plant_pause` releases any active wrench lease and freezes MuJoCo time while
  keeping a heartbeat/state stream alive. It does not run a WBC solve.
- `plant_resume` restarts the measured-state WBC/physics loop from the frozen
  MuJoCo state. It does not resurrect a previous wrench.
- `plant_reset` rebuilds the worker at the authored balanced standing state,
  increments `reset_epoch`, clears the wrench, and preserves the paused state
  when reset was requested while paused.

The target commit is deliberately bounded in the first live prototype: the
server accepts only `torso` or `base`, a 1000–5000 ms duration (default 3000 ms),
and finite world coordinates; the worker clamps root motion to a small squat
envelope. Invalid
commands are rejected without releasing a prior held target. This is the
separate admitted target/tracking/resource contract that keeps preview intent,
measured plant execution, and PUSH evidence distinct.

The orange wireframe, dashed measured rig, CoM, contact points, ground plane,
constraint rows, actuator/generalized forces, and simulator energies are
visual witnesses of this ownership boundary—not a blended estimate.
