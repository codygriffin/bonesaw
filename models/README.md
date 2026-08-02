# Robot models

`toy_humanoid.urdf` is a compact Bonesaw-owned model used by unit tests,
benchmarks, and the browser demonstration. It intentionally uses primitive
collision shapes so geometry remains cheap and inspectable.

The primary external reference is
[Upkie](https://github.com/tasts-robots/upkie_description), an open-source
wheeled biped with inertias, collisions, joint limits, wheel joints, materials,
and visual meshes. Its description is Apache-2.0 licensed. Fetch the pinned
URDF, license, and ten referenced STL assets with:

```bash
./scripts/fetch-upkie-reference.sh
```

The fetch is pinned to the revision recorded in `models/upkie/REVISION` so
updates to a hardware model never silently change a compiled program.
`models/upkie/MESHES.sha256` verifies the complete fetched set.
