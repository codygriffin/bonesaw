# Live MuJoCo ground/state audit · R234

> **PASS** for ground-plane provenance, measured collision-state transport, base-target clearance, and live plant transport. Browser frame-time/visual inspection remains **NOT RUN** because this CLI session has no attached in-app browser.

## What changed

- The viewport ground is derived from MuJoCo's streamed plane point and normal rather than an implicit browser-only `z=0` constant. It is labelled `MUJOCO GROUND` in the 3D view.
- TARGET mode now overlays the measured MuJoCo collision shapes as an orange wireframe, in addition to the dashed measured skeleton. Penetrating measured collision faces turn red.
- The measured MuJoCo center of mass and its projection onto the simulator plane render independently from the green WBC preview CoM.
- Plant state now streams the solver forward/inverse residual pair, scalar constraint force/position/velocity rows, generalized actuator/passive/bias forces, actual actuator force, exact MuJoCo CoM, constraint-row count, and summed ground-normal force.

## End-to-end evidence

The existing one-metre-down base-target regression still clamps the authored preview to **0.250000 mm collision clearance** and **0.162025 mm exact visual-mesh clearance**. An ordinary plant step reports the explicit plane at `[0, 0, 0]` with normal `[0, 0, 1]`, four ground contacts, 52.420 N summed normal load, 0.00657 mm maximum soft-contact penetration, 12 scalar constraint rows, and zero MuJoCo warnings.

The live `/plant-ws` regression also passes 250 Hz physics, 50 Hz WBC/stream, five 4 ms substeps, bounded external-load admission/expiry, fall reporting, next-step reset, recovery, fresh worker reconnect, and the expanded finite state vectors. It ran against the existing single server on port 8777; no second server or tunnel was started.

## Boundary

Green base motion is still an authored, state-local WBC query and is not silently presented as plant motion. Orange collision geometry, contacts, CoM, constraint state, and ground load are MuJoCo measurements. This checkpoint improves state ownership and observability; it does not claim that the green target is physically realizable or execute it through MuJoCo.
