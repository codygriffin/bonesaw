# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 22.083 million instructions, 5.845 million cycles, and 1.436 ms task-clock per additional tick. Native measured latency is 1755.498 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 22.083 | 7.755 | 1.897 | 2.848 | 2063.650 / 2288.685 / 2362.124 | 0 / 0 |
| 2 | 22.083 | 5.848 | 1.419 | 3.776 | 1399.005 / 1755.498 / 3075.362 | 0 / 0 |
| 3 | 22.083 | 5.843 | 1.420 | 3.779 | 1401.539 / 1715.843 / 2247.658 | 0 / 0 |
| 4 | 22.083 | 5.838 | 1.436 | 3.783 | 1421.878 / 1739.819 / 2236.457 | 0 / 0 |
| 5 | 22.083 | 5.845 | 1.442 | 3.778 | 1423.580 / 1757.191 / 1987.055 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/r60-experiment/release/bonesaw-eval | f3d8719fc8126f0dcf936a3c07b00e75388df594e6a8ed3785a58c1520057a0d |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-1-perf.csv | c40a2e95879248df954ffec529c476316da7eb5a193783dd0e785c4ec6edace9 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-1.json | a896c238fb54ea9e8ecca17d76d763dcb37e1ea98a6076af25b34ce777e434ca |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-1-perf.csv | eef9be73ea8f77b7cead2b5fa7a2280674529bf162c53b0a8e7d50e9ebd7442d |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-1.json | 38347c597e9f31f6e240d1443b702bc40613d999762e29d5c3179514212c92f9 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-2-perf.csv | eba5d8ab15dff729f107cf7da33145c04ee4d34c8a2c17978e964fe01c03e405 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-2.json | 0a6c53246a86787d3e34214c63c1025152fa1e8c335b167e6221ea0f7c439f28 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-2-perf.csv | b605595d2b5e6af150b6c337124afc30889210c90627506a7fcbc85f14b4c36e |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-2.json | b5f1d9ba4e5a70de2981eb1b4933453e28db8709e3d057c1c8ccdb9e609c1c2b |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-3-perf.csv | 26b1ffb62e92d376160dc9cd03ee74bf78a926402f0414674770649ba0e1ff43 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-3.json | 15e38d80f4085281efeae6236b157dbbcbf285130f38501e9f3c9b0b5334e73c |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-3-perf.csv | 482f771f4755524ff7b812a28128e0b81b55dc45fb9bfbb392d8484f073e33c0 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-3.json | 3b4d8f5d4cfc3dfc388bcd4c3a2992741d03cff52814e8fe9ad1ef5163ef42bb |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-4-perf.csv | 2d94122a60af90a451f6342dbbbae16fdeb97407e9a95d45ae65a18944b23faa |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-4.json | f13048e2a2d04b88ae31e9411003edfb91a0d576d5da7892377188f896467618 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-4-perf.csv | 28e016461ee3338fa589ce330e64d419de6296c18dede109e34a9a0286027f93 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-4.json | 29f840b9f4a84c6aef0e3022625fd88e8e4f56f508b99e7350976516f6c11e30 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-5-perf.csv | b65f4255fc493664460524b7b806b47a8142b6fc7a43ac62e176b18e671e6098 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/short-5.json | 1401268a21c7c13916dbdbf0095bdbe631f59f2117d6a30863ed44b0c44744ee |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-5-perf.csv | 92d608609202360f82eb48efa7e8534a7aecc02541cce31f18e1ae32910857b4 |
| benchmarks/results/g1-jacobi-discarded-r60/experiment/long-5.json | 52d0d08773ad64f70441ef3baeacad7cbd1ec8d07c3974ea802fb2db0e4004b7 |
