# Live ground and MuJoCo-state audit · R223

> **PASS** for target-ground registration and simulator-state transport. Browser frame-time remains **NOT RUN** because this CLI session has no attached in-app browser.

## What was wrong

The green TARGET pose and the physical MuJoCo plant were different state streams, but the viewport showed only one at a time. A green base intent is a guided, state-local WBC preview; it is not a realized plant command. The z=0 reference was also grid-only, and the upstream tire STL extended 0.114 mm below the collision tangent. Those three facts made a preview look like a simulated body passing through an ambiguous floor.

## Changed contract

- MuJoCo now connects continuously while the page is open. TARGET keeps the green authored pose and overlays the measured plant as a dashed orange rig; PUSH makes the measured plant primary.
- The physical plane is an explicit filled z=0 surface with a grid. Contacts and normals render in both modes.
- Green state is labelled `WBC TARGET PREVIEW · NOT PLANT`; the dashed state is labelled `MUJOCO MEASURED`.
- The preview reserves 0.25 mm collision-ground clearance. The browser computes exact transformed visual-vertex clearance and turns any face below -1 mm red.
- The plant stream now includes actuator effort, generalized acceleration, generalized constraint force, kinetic/potential energy, warning count, root pose/twist, joint speed, contacts, penetration, solver work, and the existing 250/50 Hz timing contract.

## End-to-end results

| Gate | Local 8777 | Public Cloudflare |
|---|---:|---:|
| reachable base endpoint residual | 0.000 mm | 0.000 mm |
| reachable collision clearance | 0.250000 mm | 0.250000 mm |
| reachable exact STL clearance | 0.135636 mm | 0.135636 mm |
| one-metre-down clamped collision clearance | 0.250000 mm | 0.250000 mm |
| one-metre-down exact STL clearance | 0.162025 mm | 0.162025 mm |
| local/public command acknowledgement | 22.33 ms | 44.70 ms |
| local/public controller query | 38.69 µs | 20.62 µs |

The local plant gateway separately passed 250 Hz physics, 50 Hz WBC/stream, five 4 ms substeps, finite 12-value acceleration/constraint vectors, six actuator efforts, explicit z=0 ground, finite energy, zero MuJoCo warnings, live contacts, bounded wrench expiry, fall report, next-step simulator reset, recovery, and fresh-worker reconnect.

## Scope

The exact ground test transforms all 41 authored visuals, including all 25 STL instances, from the same streamed frame record rendered by the page. The base-target arm uses no physics or policy. The plant arm is a separate MuJoCo consequence/telemetry test. A red WBC preview remains visible by design and does not claim realization. Green base intent is still not executed by the plant; this revision makes that separation visible rather than silently merging the states.
