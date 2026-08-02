# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 15.807 million instructions, 5.499 million cycles, and 1.316 ms task-clock per additional tick. Native measured latency is 1360.581 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 15.807 | 5.620 | 1.352 | 2.812 | 1320.384 / 1667.400 / 1915.969 | 0 / 0 |
| 2 | 15.807 | 5.499 | 1.313 | 2.875 | 1310.185 / 1356.733 / 1434.720 | 0 / 0 |
| 3 | 15.807 | 5.502 | 1.313 | 2.873 | 1311.418 / 1355.742 / 1501.636 | 0 / 0 |
| 4 | 15.807 | 5.498 | 1.316 | 2.875 | 1310.776 / 1360.581 / 2591.225 | 0 / 0 |
| 5 | 15.807 | 5.495 | 1.317 | 2.876 | 1314.994 / 1360.690 / 1528.537 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /tmp/bonesaw-eval-r74-control | e69a8686ec6c275a353f30630cfa6131486e7af65816a6b16fa08e05efb8347d |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-1-perf.csv | faef2c66dc91629b237fcfbe26287c1a836458a1afb225710b06ed97d86bb3e6 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-1.json | 874f3271789e87019d0c7a0dc9137927b2e703fe8be2d47103dfe7e7d65a18ce |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-1-perf.csv | 00cdc27e975abe40f69f448318ada137ad9714c53131e2e574efe1d138464653 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-1.json | 2ea4c3936a27c8350791b323880a9e88b58c8a9d61db28d42b9c6143a2a91f5a |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-2-perf.csv | c03e57e2c26b81e92de72581387275952e2606f192516fc1c8599ffb65b2a167 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-2.json | 504970a4c2d359881494b13e78211cf4601a4d3288e7ddfdebe2a4b369820283 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-2-perf.csv | 27f75b7f1e472aa6535e9f034b7691743b7d1366056999d981dea2966e6d0ace |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-2.json | 864416a0e6c3760bdbcb11fa5523af257a79cc22ee2d4fe014710403f67e9d63 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-3-perf.csv | ee88aa30a0d5c1d99ac8a1d3bce9a895761d0a16bb3461ecaeda9ebeafa1a3cd |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-3.json | 1b34100db17ae48e33376edc11e55c6a87c19fe62b02bd1f34f58bd3050e3791 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-3-perf.csv | 282476ebba05c884f1cfc4e323235959a529131102718ce24787d6c25c3801b6 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-3.json | 4743556919b55c35f1ed220755b26c2f64dc1058ab1f02d8e043abc901326ffc |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-4-perf.csv | df8e5fd001bf55cb96fd2bbd94ee44f6c32dfd37c927cb55af9661a48b6052bf |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-4.json | cfb36213dd3515a04c5c2749a6b1e32f1a342b9efffbbf0e55a234698a124420 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-4-perf.csv | 4ed4d1de33bb1a4405c15fe4246c3b11c7a5397ea0af675d76c96be410813fb5 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-4.json | 79cc3776137b0debd9d12aa31738cb4ae79952834783f493c21ad46c79c2eabe |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-5-perf.csv | ada330eff9ca93d707827929081bcca26b19b4a7649dd80a046c1d70c8c0f873 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/short-5.json | 85f34188564ebfb392b0a004ffea8b3b04998104b46c15849e6321b7db5190e4 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-5-perf.csv | b67ddec446ffd82ec9d4a998b2d586664ad0d703c8f6bcc20488696967ca30d6 |
| benchmarks/results/g1-dense-multiply-row-slice-r74/native-control/long-5.json | 98b77953d8c0b8c932b3a17c52e97fe18972cfc8e71cba3f5e3e9c34223ac200 |
