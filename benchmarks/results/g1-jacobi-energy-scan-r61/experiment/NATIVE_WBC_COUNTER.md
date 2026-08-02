# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 22.050 million instructions, 5.861 million cycles, and 1.419 ms task-clock per additional tick. Native measured latency is 1724.429 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 22.050 | 5.831 | 1.414 | 3.781 | 1405.977 / 1481.299 / 2073.439 | 0 / 0 |
| 2 | 22.050 | 5.847 | 1.419 | 3.771 | 1407.199 / 1669.304 / 1901.783 | 0 / 0 |
| 3 | 22.050 | 5.917 | 1.434 | 3.727 | 1405.957 / 1889.691 / 1984.300 | 0 / 0 |
| 4 | 22.050 | 5.861 | 1.415 | 3.762 | 1401.999 / 1724.429 / 1930.788 | 0 / 0 |
| 5 | 22.050 | 5.975 | 1.460 | 3.691 | 1419.022 / 1895.151 / 5935.155 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/r61-experiment/release/bonesaw-eval | bfd2227453226cb8e09fd8ebd09926db13c46aa1d6802489c7cf9e38417895db |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-1-perf.csv | 457d7ece2b1b8e34be726f2c956c83d0deb530ae2ef1227877af7b5d7da287b0 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-1.json | 2233b937d87696468e0b510a2e1ac0377700a5ce93bd4d73a26850bced53807d |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-1-perf.csv | 11e868675e3a55471584756678e3abd64604614bfe38f011585dfbae0f2293cc |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-1.json | 514883408f45b12500f7c13a8ce29cdb96390d97a0e5ef7f6ac3ee96c8cd2f25 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-2-perf.csv | e9e09ba31cb6b513f8090879d177e7d052af8b113fcd745b1cca58b7d7a46691 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-2.json | aeb456ac574ff7e7d4c4465320e925b139797e4567b4aed097091a6ef894be95 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-2-perf.csv | bf374977cb3f13360df6d990903e3be639d378cc1cd55b444715840d2b3ec14e |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-2.json | da264bbf07aa2a875ede421a8d840da902249b7c59e4ba8db27aa8cce57c7db4 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-3-perf.csv | 03c2c685a6cca6c2e83c41261cdab99070d0b03fac6bdc9a22f0ecf2fce53bab |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-3.json | 633b349fb77f5b325c46d65b309959a4f97fb22e3d621de5894dabcfe406ef3c |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-3-perf.csv | b6ac3481018ac11fb40d745247da69d69e168852433e136ef53d95d927d34a83 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-3.json | ca48cde17c8d62c352f3a8bf434ba1f87821c97547120fa850876a6f081a7290 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-4-perf.csv | 36d6732f6f04196b73e217f8986db1ec1a650032a5917b63d69b4c68a3984cd2 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-4.json | 57e12aa92be8f7a03703b281d7fb4aa8470a48186bd30257c4d26333fbde2418 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-4-perf.csv | f17504c560ad09ea76a7ad0681a0bb42d88fcd7670f99b9dbba41eb9e70d1fda |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-4.json | 7a06dcd5369e2da1d046f15845e044f59ea8e9b80aca3ea87eee1a7c002dba8b |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-5-perf.csv | fe7e04dfda819aecdac286a6c9083fc217f4809c74626edb8d8a6e4af93e68a7 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/short-5.json | ca88551915eaa41f62e5ccd27a4dee0ce49e0b3d588a58ef77c0dd68d93f22e2 |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-5-perf.csv | 7340e81fab098a92ca161cd13d34c7e3c41efcfc85a33b80b0477b8a6192d8cf |
| benchmarks/results/g1-jacobi-energy-scan-r61/experiment/long-5.json | fc310355537de3bd26b8eb99aa4afa2a5413aba51524fabd1bb41f2e49e216a9 |
