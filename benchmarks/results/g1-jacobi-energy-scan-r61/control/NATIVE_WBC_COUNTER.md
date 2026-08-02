# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 21.991 million instructions, 5.855 million cycles, and 1.422 ms task-clock per additional tick. Native measured latency is 1539.089 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 21.991 | 5.968 | 1.448 | 3.685 | 1414.292 / 1944.955 / 2124.414 | 0 / 0 |
| 2 | 21.991 | 5.846 | 1.416 | 3.762 | 1408.772 / 1498.652 / 2114.015 | 0 / 0 |
| 3 | 21.991 | 5.855 | 1.422 | 3.756 | 1412.309 / 1606.946 / 1979.129 | 0 / 0 |
| 4 | 21.991 | 5.856 | 1.422 | 3.756 | 1413.100 / 1539.089 / 2105.269 | 0 / 0 |
| 5 | 21.991 | 5.852 | 1.424 | 3.758 | 1417.799 / 1495.486 / 1943.883 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/release/bonesaw-eval | 4ecb568ee558cb80f130a90cbbe8a1e4ee45918ec624909bd9ba071627a65ca3 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-1-perf.csv | 07c7d0ca848adc035509a0e3cfa7308eec641eeb0d758625fea24f9247926837 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-1.json | c864ae64221445530d437de72a185cb91d84233b7d8444748fc28268a4d43136 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-1-perf.csv | e3b53d985ad7d61fa5511319b9c8fa8f337a01b4b369145b9deb3748da6c80a7 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-1.json | 5fa2ba095b0cb6ed17eded20dc4efc065bddaf78911896915e07cae921362881 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-2-perf.csv | 44cd5104c1e6068c0415b8287e8c34f9dcc23b6f6f872d23f9f966c4e566c050 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-2.json | 3f57e60c1297d50832b816f21a9a4df78e1e5ec189bf7c43651a6bd840cb9bda |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-2-perf.csv | c4d97dc80d935e47af34a2ce151955befb120c0118d7a6001938d30b6f57e11f |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-2.json | 62a4e43978ee119782a847b47f8f32091dd366dd7bfc457a975f10cd286385c5 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-3-perf.csv | fa53dc7b0a6f7b2dc375ae67b35c8042c60fff78ad10f353dc4f15603cef6e8d |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-3.json | e56c8ad15c43b7713adc8811d7f7b97f97d6a7617a936ff2b8334701b272444b |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-3-perf.csv | a2b9cf797d99b1a5b0e4037607d20b9703daa89bbf831dd3730db83ec0c5d70f |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-3.json | 6162f916fd48d8e15c74e6bc5e8e16b24c0e973a8ce4bc2ece4fe577290e1a9f |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-4-perf.csv | d21690eff378856f0a3763162ce560927ede50979a8a8b4f3f525cad7bfb8c4e |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-4.json | d6f46b734adc2895318b040dcd83538877b0ce6a42180874ddafafd2ec5b6030 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-4-perf.csv | f97d8ae351f34783da7ca111401ba969e55f64d1a03f436c3d99f6cd6faf6c21 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-4.json | cba95b546ffa83c9b68b9d1a693cf6c7b801cbbbdbb30960999a1ba043653d1c |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-5-perf.csv | e2ccb936b6a9ce508ac2f4f5807c08151dffff2dc2157b87ec7d3328f9040acd |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/short-5.json | 6af4a9e683ee70757a0b5aba8d1d487fd2f5ca384f0ac5bc9106fdf7ef8daf53 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-5-perf.csv | bdf5b9204ebca749edfff1e466278f2366b059e399022af0751328b169f548e8 |
| benchmarks/results/g1-jacobi-energy-scan-r61/control/long-5.json | ac746aac3602dcb07a3920c1b882b9664a9a8549a5fd48f48191e6a25bd01f1e |
