# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 15.317 million instructions, 5.380 million cycles, and 1.286 ms task-clock per additional tick. Native measured latency is 1331.105 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 15.317 | 5.379 | 1.286 | 2.848 | 1283.114 / 1331.185 / 1512.988 | 0 / 0 |
| 2 | 15.317 | 5.388 | 1.287 | 2.843 | 1282.303 / 1329.241 / 1563.163 | 0 / 0 |
| 3 | 15.317 | 5.382 | 1.285 | 2.846 | 1281.542 / 1331.105 / 1604.060 | 0 / 0 |
| 4 | 15.317 | 5.380 | 1.285 | 2.847 | 1281.440 / 1337.717 / 1615.892 | 0 / 0 |
| 5 | 15.317 | 5.378 | 1.287 | 2.848 | 1281.420 / 1330.534 / 2643.583 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /tmp/bonesaw-eval-r74-candidate | 4ac4cf86389314a92806b97ffba2e7c8a53d8cb95b5f4decb3d10f8b672f9a96 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-1-perf.csv | aefa73cb24faf39d0341b33561bad63e3234fe514103c4ad12370728088c84aa |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-1.json | 468ad12b1fe6377172e7d498a23014f48e8cb2df0041d71de852eb8cd8497019 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-1-perf.csv | b19f06fd5060187ef831b2848a3b8c32f15a730d0560fc4fce69cb1a5843c71b |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-1.json | b69ec10bc23a7ca3ddce6aa59228919b286e013c1a740961f8ebf63371f3ffcc |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-2-perf.csv | 678cc61bf6e231b77daaff599971751908619fa66ad234648a791a7536a78f73 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-2.json | 601a7bfddea521510c07deef3e152eb49e08dfb7b16812c19d86ce74334367ac |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-2-perf.csv | f4d85688ef30f973c31429efac6fb7ef38c66e18a89df1bf8a207c119696946c |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-2.json | d47a03a85b3c43cfdca04be67d35a798549fdd264950b2608205b049716a48f1 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-3-perf.csv | d970e5301850eab2e256bd0b3fe20cbb0677d4861b5e624f10140c1f58b680bc |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-3.json | 28ef0d82ff94aa37ff6196363360808a8ab5d21e60c0ec24f5c9dc3dc5e816c1 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-3-perf.csv | ec7dcbb84c102f33469b1b2427a601b4a8417690b8c2d6890b621ea9d9719792 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-3.json | f80d806a98a7484ef18e693814525f45630561294a7613e898d34386573fb31f |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-4-perf.csv | 9e14007205cec269aaae0001c0dede35a623e0f66bcfe6a725978f9d7e149626 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-4.json | e152fc6b5fd7fda40bfeb02e39070d472aff31e9866c90b35207144718cd2a41 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-4-perf.csv | 0d2d13db31b9df5f1991fe8c4aaf3d3f2230e60fe469e295b1e9dbba2ac55bbf |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-4.json | 75cd6c8a39af3f61c285e727e5bd0b5f0327fcbc8e03eaffae7c84c5d41f58b4 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-5-perf.csv | a2421cc3a5678cc342a53745528c4ddf9c2eb9b293ccc13ed53c7271a16c5b74 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/short-5.json | d396d113d2e5d429c9f115aaedeba9eae4f7ce14a848eb8cd4a1b00219aadf0b |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-5-perf.csv | a721c08e55674392b8c918d211f8ae10ce87b1cee8f3cae774207c7976171dc5 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-candidate/long-5.json | e0c38f0820597ed385929113518dd406864c15e407dd7a373b3e531ab0ab7b2a |
