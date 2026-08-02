# Bonesaw dynamic swept-collision admission · r101

## Outcome

**PASS.** R101 applies the compiled conservative self-collision model to the actual commanded joint polynomial on a deterministic 1 ms grid, including both endpoints. Primary and braking contingency are swept independently; their clearance evidence and typed flags remain separate from solver, joint-limit, effort, and plant-response layers.

This is command-geometry validation, not physics. The observation supplies the root pose, while the joint command segment is swept without integrating root acceleration or inferring contact response.

## Retained cases

Selection codes are `0=Primary`, `1=Contingency`, `2=Rejected`.

| case | clearance m | selection | flags | primary / contingency valid | primary min m | contingency min m | first pair / time ns | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|---|---|---|
| safe_enabled | 0.02 | 0 | 0x000 | True / True | 0.19999999999999996 | 0.19999999999999996 | -1 / -1 | 13.560 / 23.571 / 35.147 | 0 / 0 |
| swept_primary_collision | 0.02 | 1 | 0x080 | False / True | 0.01053187952928658 | 0.03499999999999992 | 0 / 16000000 | 15.930 / 24.757 / 26.880 | 0 / 0 |
| same_motion_collision_disabled | DISABLED | 0 | 0x000 | True / True | inf | inf | -1 / -1 | 9.939 / 15.136 / 15.820 | 0 / 0 |

The collision fixture has two 0.2 m spheres separated by one prismatic coordinate. In the fault case the primary command crosses the 0.02 m clearance at pair `0` and time `16000000 ns`; flag `0x80` activates, primary is withheld, and a zero-effort braking contingency remains clear. The same motion with collision policy explicitly disabled selects primary and reports no fabricated distance evidence (`Infinity`). Unknown or disabled geometry therefore does not silently masquerade as a measured clear path.

All three cases replay semantic bytes exactly across 100 resets, satisfy hard dynamics residuals below `1e-7`, remain inside 20 ms, and allocate zero bytes inside the Rust transaction.

## Boundary and remaining work

The current CPU sweep uses deterministic dense sampling over conservative spheres. It is not continuous collision detection and cannot prove safety between 1 ms samples without a distance-rate bound. Meshes are skipped by the compiled high-rate proxy, world SDF/obstacles are not yet ingested, and the floating root command is not synthesized. These remain explicit gaps rather than free-space assumptions.
