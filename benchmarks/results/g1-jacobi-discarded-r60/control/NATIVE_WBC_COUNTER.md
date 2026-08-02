# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 21.991 million instructions, 5.886 million cycles, and 1.434 ms task-clock per additional tick. Native measured latency is 1758.324 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 21.991 | 6.221 | 1.510 | 3.535 | 1409.885 / 2169.029 / 2391.049 | 0 / 0 |
| 2 | 21.991 | 5.902 | 1.431 | 3.726 | 1408.061 / 1726.263 / 2073.259 | 0 / 0 |
| 3 | 21.991 | 5.868 | 1.426 | 3.747 | 1410.215 / 1739.157 / 1870.936 | 0 / 0 |
| 4 | 21.991 | 5.885 | 1.434 | 3.737 | 1416.467 / 1758.324 / 2063.149 | 0 / 0 |
| 5 | 21.991 | 5.886 | 1.435 | 3.736 | 1414.273 / 1778.812 / 2070.944 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/release/bonesaw-eval | 5384d7af7b2a71adafa62cc6dd1dc3d5e9e0cb8a2b3f634b4e7c5423649d1156 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-1-perf.csv | 6f79b263a9c03a2edaabf082f47f6809865c667a3fff02b2311782af8ca427d5 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-1.json | c5b2d9b83fbfcb25dafc9ab39840d1aaa51b439768720866818f3225d6115dcf |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-1-perf.csv | 3c4a155ed1f0aa7eca158380f80343693415bb62c9ad7053d9fc9c244f0451f7 |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-1.json | a3dfc3c80ddec8ea9bd3072f11aa19064df50064ac01dd545f431cf86acf9322 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-2-perf.csv | df5138c82a5b0c872d6d929a473d66519136a7ceaac9162505aaede4b24faa74 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-2.json | 70649095b166244a2e8f0fddaace8b8f297703a1306152792baeee733bf47945 |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-2-perf.csv | 85c5a0ce97007b4fafeeb4e39355b2bd10196072ab25ab4fb46622deead7d771 |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-2.json | 70f8e1100736e48f236135727e329ba35c844ed97c9bf931ed3de8f1ba1726df |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-3-perf.csv | 74497f3c1eca798760ff771e50e6a9038d0a8496a361b69ca317695d42407159 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-3.json | d20be3501b9970755f19d8a0b232eb9f948585dbb164e95c63a13ff0d903335f |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-3-perf.csv | d5757a779bb006bdaf5deb8c2892a9c4a028dabb857ba5f36f809f1fef33011e |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-3.json | d2a9410139afb5441ae155075cd9760c4a85456cdb7e8663093458f8d8c83785 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-4-perf.csv | 029891e06c387b68bb0cf4c5b145112d84bc5733e704a926d7ad696002e1a629 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-4.json | 800696925f0e4887a31bb72c6dd53ef03da2610804fefe87faad61f9e967c1aa |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-4-perf.csv | 24399882834ee918c89e19ecdb0680645844a8df87faad0c046e654c40ef26c1 |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-4.json | f3d41ff6e763cd28bc54332c38bc5bfd68f5f0851ff8e892a7e0d7b94c34ba1c |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-5-perf.csv | 2991a202af5472eede0e3b65be30f84b84b6bf2ad83ce7a4613229891420fbe3 |
| benchmarks/results/g1-jacobi-discarded-r60/control/short-5.json | 85c5c8335d0af6f2f7e445248c71024b1e28c51d0332811b250f0316d5a6f23b |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-5-perf.csv | f82cc2a3812882ece98c8783a0002e37acd88ca9d53d7478134dbef5c4eebd31 |
| benchmarks/results/g1-jacobi-discarded-r60/control/long-5.json | 04db63b2c8daaab3cbdbbc8a9324c5230fee040c28b27f4aeb048c883c870d9e |
