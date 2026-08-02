# Pinocchio differential oracle

Bonesaw's independent rigid-body reference is Pinocchio 4.0.0. The oracle
loads the same pinned Upkie URDF in separate Rust and Python processes. Joint
and frame values are joined by canonical names rather than assumed vector
ordering.

## Reproduce

```bash
./scripts/run-pinocchio-oracle.sh \
  --model models/upkie/upkie.urdf \
  --samples 50
```

The runner creates or reuses an isolated environment at
`/tmp/bonesaw-pinocchio`, installs the pinned `pin==4.0.0` wheel when needed,
builds `bonesaw-oracle-fixture`, and runs the strict comparison. Override the
environment location with `BONESAW_ORACLE_VENV`.

## Current Upkie result

Captured 2026-07-31 over 50 deterministic states and all 41 body frames:

| Product | Maximum absolute error | Acceptance limit |
|---|---:|---:|
| Frame translation | `2.220e-16 m` | `1e-10 m` |
| Rotation-matrix element | `7.772e-16` | `1e-10` |
| Frame Jacobian element | `8.976e-16` | `1e-9` |
| Center of mass | `1.110e-16 m` | `1e-10 m` |
| Mass-matrix element | `9.714e-17` | `1e-9` |
| Generalized gravity | `1.110e-15` | `1e-9` |
| Inverse-dynamics torque | `1.110e-15` | `1e-8` |
| Centroidal-map element | `1.665e-16` | `1e-9` |
| Centroidal momentum | `5.551e-17` | `1e-9` |
| Floating mass-matrix element | `8.882e-16` | `1e-9` |
| Floating bias force | `2.132e-14` | `1e-8` |
| Floating inverse dynamics | `2.132e-14` | `1e-8` |
| Floating centroidal-map element | `8.882e-16` | `1e-9` |

The toy humanoid also passes over 20 states. Its largest measured error is
`1.066e-14` in generalized gravity.

The comparison explicitly uses Pinocchio's `LOCAL_WORLD_ALIGNED` frame
Jacobian and changes row order from Pinocchio `[linear; angular]` to Bonesaw
`[angular; linear]`. This prevents an accidental convention mismatch from
appearing as a dynamics failure.

The floating-root fixture also records a nonzero root twist. Bonesaw's root
tangent acceleration is `[angular; classical linear acceleration of the root
origin]` in world coordinates. Pinocchio's free-flyer input is spatial
acceleration, so the oracle applies the explicit `-ω×v` linear conversion at
the identity comparison pose. Generalized root forces and centroidal products
are compared in Bonesaw's `[moment; force]` order.
