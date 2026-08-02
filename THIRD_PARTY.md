# Third-party model references

## Upkie robot description

- Project: Upkie wheeled biped robot description
- Upstream: https://github.com/tasts-robots/upkie_description
- License: Apache License 2.0
- Pinned revision: `94735fbe6137276a41de0ff4cc04d2e533fa9e33`
- Copyright notices are retained in the fetched upstream URDF and license.
- Bonesaw vendors the pinned URDF, license, and ten referenced STL assets under
  `models/upkie/`. `models/upkie/MESHES.sha256` covers all of those bytes, and
  `scripts/fetch-upkie-reference.sh` reproduces them from the recorded upstream
  revision.

## Pinocchio differential oracle

- Project: Pinocchio rigid-body algorithms
- Upstream: https://github.com/stack-of-tasks/pinocchio
- License: BSD-2-Clause
- Oracle version: `pin==4.0.0`
- The optional runner installs Pinocchio into an isolated temporary Python
  environment. It is not linked into, vendored by, or required to run
  Bonesaw.

## PlaCo reference WBC

- Project: PlaCo whole-body inverse kinematics and dynamics
- Upstream: https://github.com/Rhoban/placo
- License: MIT
- Comparison version: `placo==0.9.23`
- The optional Python evaluation runner installs PlaCo into an isolated
  temporary environment. It is an independent performance and behavior
  reference, including the standalone r41 walking-pattern generator oracle,
  not a Bonesaw runtime dependency. The WPG path runs without PlaCo WalkTasks,
  IK, WBC, integration, or simulation.
- The r42 Bonesaw LIPM result is an Apache-2.0 in-tree implementation and uses
  PlaCo only as an independent comparison oracle; it does not call or link
  PlaCo.

## CMU walking motion reference

- Dataset: CMU Graphics Lab Motion Capture Database
- Source: http://mocap.cs.cmu.edu
- Pinned sample: subject 37, trial 1, “slow walk,” 120 Hz
- License statement: the database FAQ permits the motion data to be copied,
  modified, or redistributed without permission; the data itself may not be
  resold directly.
- The raw ASF/AMC files are fetched into the ignored benchmark cache rather
  than vendored. URLs, checksums, unit conversion, and the requested
  acknowledgement are recorded in
  `benchmarks/references/cmu-37-walk.json`.
