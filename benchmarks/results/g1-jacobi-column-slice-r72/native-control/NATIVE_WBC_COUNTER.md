# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 22.022 million instructions, 5.787 million cycles, and 1.387 ms task-clock per additional tick. Native measured latency is 1428.639 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 22.022 | 5.793 | 1.395 | 3.801 | 1388.834 / 1439.009 / 2870.874 | 0 / 0 |
| 2 | 22.022 | 5.787 | 1.387 | 3.805 | 1385.387 / 1428.639 / 3031.087 | 0 / 0 |
| 3 | 22.022 | 5.781 | 1.386 | 3.810 | 1385.077 / 1426.786 / 1746.310 | 0 / 0 |
| 4 | 22.022 | 5.790 | 1.389 | 3.803 | 1385.318 / 1472.392 / 2795.592 | 0 / 0 |
| 5 | 22.022 | 5.787 | 1.386 | 3.805 | 1386.379 / 1420.975 / 1821.261 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /tmp/bonesaw-eval-r72-control | 93eb53f3bfa03861e3dba7c50e78fdb967b3bdffe7553e8f245575fc125849b4 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-1-perf.csv | 5c14da663c84124cf2f0e90ca68a7749f8a0d139dd3926c918f32f5a505b628b |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-1.json | 1d0fef3f5c245e32e224f819034b2c4186c3181cdea1262abd68a5c1a27b0f50 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-1-perf.csv | 2b5d8aea456f42a1bcff04715dfb8b39e7a7c9d80f0220b6eeea56d61874ce72 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-1.json | cca3acdc1c939bee462d8ab579e786ff2ae662308ad5212e49b087cb4306e3f0 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-2-perf.csv | 1171d82c53185602a3d4044b1ee44b6d4e1d81a9dc4d5b7eca87c91f291ea63d |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-2.json | 134222990b74beb6f6b0810b3ae72f1132e12e85a3545142fd333923940d1a67 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-2-perf.csv | e7d5fa9fc4242a540a9207a5e3b33a8e80432d475647a02a6efdd4d9a5aaa7b2 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-2.json | 318fd055f36b7f352fe0443feea3d41f5853e866c0c403aed26072f68573edb2 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-3-perf.csv | a8f66474f1d598c267fa80bbace243f95aef89a5c884f16547552ab27491d0d6 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-3.json | 0ee600e20ae6081c08b524ae765e27e3af8a9144eb39eaa017497421f0e1b867 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-3-perf.csv | da3c0198c18018906358ecc4711a925003a19ebac1fa276290d63d992bc56f2d |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-3.json | dcc2091eca1e346652aa0b2e43334b3ef9a2d2c375643029c1ffa803b9b158dd |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-4-perf.csv | 538b17a3bde50589d675c6179d67041a7102c54c317c1ce46b25ef4dfdd2edad |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-4.json | 84adf1a626ef1b2f04a82854eb109fc2c2cd7074302c987c52cc8872bd01cb8e |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-4-perf.csv | e8d71a593dcf0c297af5b9569d70440f7624149ea6e5c9a535887a2ce51acc9d |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-4.json | 6d00b7b7a6caee61ad1b5e818ca0387af6df8d890d63832d27c6605808a98cf1 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-5-perf.csv | 0e39575a1870dec9c56ed4f669d458084bc493169dd3528192a3041ed44b33ca |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/short-5.json | 4c0a5236d64a6a2a399d8833534d932fc005cfe5db0422968354baa71fc7d959 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-5-perf.csv | 8e9128f01f5bd75f59d0437f7924f4e388977d051574ebe180661378441b34ff |
| benchmarks/results/g1-jacobi-column-slice-r72/native-control/long-5.json | af3e6fb6f8ff05978e06120ae81657daf492e00e3d699a442e0bc74158c49e2b |
