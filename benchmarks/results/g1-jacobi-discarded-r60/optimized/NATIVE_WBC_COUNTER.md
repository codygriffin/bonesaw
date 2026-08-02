# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 22.083 million instructions, 5.829 million cycles, and 1.440 ms task-clock per additional tick. Native measured latency is 1716.203 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 22.083 | 5.839 | 1.423 | 3.782 | 1405.706 / 1726.593 / 2008.245 | 0 / 0 |
| 2 | 22.083 | 5.829 | 1.441 | 3.789 | 1427.258 / 1715.552 / 1823.507 | 0 / 0 |
| 3 | 22.083 | 5.828 | 1.440 | 3.789 | 1423.249 / 1716.203 / 2071.885 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/release/bonesaw-eval | 8fc3e29923e52fc6c7b30417bafa9ab594844d81ca8e7fb33078b4aae44802a6 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-1-perf.csv | fa901f34c6e204ec7c488dc890d3c6ae41f1b11966049e2dc5d193beace31ee9 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-1.json | 4e8d47113d9de512369b36709d3cc09f09511a0f8a8d9bb221927cb3697d6515 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-1-perf.csv | 30b704741c194999d7a8bea64c4a3eba9d56418bec037587fd18aa1a6f83c2b4 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-1.json | 6784abaef8021e6ca433e173cfcc42f9d0e0b58d00db805758c1532733f28358 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-2-perf.csv | f387dfba717de087215f508e87b4e0e46ee5b898b06ab32faf13a9a579c2b477 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-2.json | 00f44faa50db03c6f7383e0d384205f4796074897908477afda302eaf74551b6 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-2-perf.csv | 6d4509f6e4e94b0bb2ff2bb00eeed424c8d7d4d60c899eb7095b71e924e1ccda |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-2.json | 2d98421b735efaf867451030ea0bdc9ae056072adc4ca6efa0b479c44eaeef28 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-3-perf.csv | 8a898cbc58b1182e5d1b8884d824ecd86c9427a02ee22227aa5dd0347a072edf |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/short-3.json | 0dfd33531d9c8a98ae40c9c45758315ae33bc6319156430add5366edaa4104f9 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-3-perf.csv | 857b25cb862e090991827fcb32d7ffad393d194a6186d3c889add4587b728f00 |
| benchmarks/results/g1-jacobi-discarded-r60/optimized/long-3.json | eeb9ba67d09b8a2dbda4061d1d9671a51f129c7b5eb2d3585ff8c70bdaf8a7c5 |
