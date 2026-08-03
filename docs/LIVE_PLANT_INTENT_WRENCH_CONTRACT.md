# Live plant intent, feedback, and wrench contract

The browser exposes two different causes of motion and keeps their evidence
separate:

| input | owner | meaning | physical effect |
| --- | --- | --- | --- |
| green target / `TARGET` | Rust `/ws` | desired pose or end-effector intent | state-local WBC query or guided preview; it is not a MuJoCo command |
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
from the latest MuJoCo root pose/twist and joint position/velocity. A wrench is
therefore a disturbance to balance against, not an intent target and not a
reason to copy the controller's predicted state into the plant. The streamed
record labels the measured state, the accepted wrench provenance, and the WBC
admission result independently.

Contact authority follows the same measured-state boundary. On every 50 Hz
observation the worker derives the two wheel subtrees once, scans MuJoCo's
world-ground contacts, and writes the raw mask into caller-owned scratch
storage. That mask is passed to the persistent Rust adapter with an explicit
fresh-observation flag; Rust owns timestamp/provenance checks, three-sample
positive debounce, and immediate hard-row removal on contact loss. The stream
exposes all three witnesses as `wbc_observed_contact_active`,
`wbc_debounced_contact_active`, and `wbc_hard_contact_active`, together with
observation status/provenance/flags and support counts. A reset or paused
heartbeat reports no fresh observation and cannot turn the authored nominal
two-wheel stance into support authority. This is measured contact admission,
not a claim that the WBC can execute a floating walking transfer.

Simulation lifecycle commands are explicit and fail-safe:

- `plant_pause` releases any active wrench lease and freezes MuJoCo time while
  keeping a heartbeat/state stream alive. It does not run a WBC solve.
- `plant_resume` restarts the measured-state WBC/physics loop from the frozen
  MuJoCo state. It does not resurrect a previous wrench.
- `plant_reset` rebuilds the worker at the authored balanced standing state,
  increments `reset_epoch`, clears the wrench, and preserves the paused state
  when reset was requested while paused.

When an intent-to-plant path is added, it must be a third typed command and
must enter the same authority stack as any other desired acceleration. It must
not be encoded as a wrench, and it must never overwrite measured MuJoCo state.
The current editor intentionally keeps intent preview and physical plant
execution distinct until that command has a separately admitted target,
tracking, and resource contract.

The orange wireframe, dashed measured rig, CoM, contact points, ground plane,
constraint rows, actuator/generalized forces, and simulator energies are
visual witnesses of this ownership boundary—not a blended estimate.
