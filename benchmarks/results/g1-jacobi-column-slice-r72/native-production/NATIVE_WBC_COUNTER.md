# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 15.807 million instructions, 5.487 million cycles, and 1.313 ms task-clock per additional tick. Native measured latency is 1348.418 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 15.807 | 5.487 | 1.313 | 2.881 | 1311.559 / 1342.717 / 2044.783 | 0 / 0 |
| 2 | 15.807 | 5.482 | 1.310 | 2.884 | 1308.712 / 1344.911 / 1598.751 | 0 / 0 |
| 3 | 15.807 | 5.488 | 1.316 | 2.880 | 1310.767 / 1357.915 / 2529.018 | 0 / 0 |
| 4 | 15.807 | 5.488 | 1.312 | 2.880 | 1310.987 / 1348.418 / 2365.159 | 0 / 0 |
| 5 | 15.807 | 5.487 | 1.320 | 2.881 | 1310.276 / 1487.902 / 2685.464 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /tmp/bonesaw-eval-r72-production | e2fac411cb0c12ac2f01d2fd36580c64a41b1d5a8a1da4e3a61a7697307489d5 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-1-perf.csv | 5cea1c3cb1863daa70f60ccc8e1410f49385320bb6b81dbe46b68438deb76b4f |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-1.json | ff0c416f3481a34994b38fe822dccbc8ab08820fa84b9440864c67a8e8c3ab54 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-1-perf.csv | 1516dc2a7a708a88ad3a818193255d8dc77907ee5052a4f556dd1d0a32ce572c |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-1.json | 5955928f0c95d0ea0ea949379744da4dff5ee39b003834e89015c2b279998c67 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-2-perf.csv | a54a8e5a75d41954c12ec0154b92bc3560f35c98f51e615f60f110fc3ab01005 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-2.json | b9dc5f424df78ceea939c5d4229cfe83ceef9af4cf11f51226c60bfc83ef0137 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-2-perf.csv | a0e4b6ca4db4dc44812c00078de1a82a74942a24c9a9dafcb471b903faebf266 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-2.json | bbf0caa5b14c3ddc4ac9825331c709ecbbf66e4e5a55a91a2835b9908369111a |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-3-perf.csv | 16e16b47427b441a71ce68e41f3aed5c617f27e9be469158d7ca49ed5a0a12a4 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-3.json | 5da4dacc68125f9a6b731b95d1dea543f373b7560a610113c9f68f9b541665f0 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-3-perf.csv | 87e9660f4c9a54e2d90094887c6eeca3da5dc90409376694d6e589ee47b7d6f7 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-3.json | ff0cbb85b9eb62e5a1514b5004fe43d49a1a1185815b7503fd8ac6f0999ae218 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-4-perf.csv | abc6363b9e3ccd7f387eb461dbff04a8fcc13f85e384d531e229f02600a6cc67 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-4.json | fde1c0b01f8541ac20169d0aec1912e5c72ffbe55852212bfe814fca4610042b |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-4-perf.csv | 6dba08dd20a95f96aa70304649b194306b1f3e4d10b325328494a84f23effd15 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-4.json | fc14d3cff49fc5c63989062b016b20cd30dc2b1c48d4ac71272723b456ebf17d |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-5-perf.csv | 5b75a99cd40811a5fa5cdbfe37e18b1f53c90984649dfc090cad11eb7dab388a |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/short-5.json | 64beab3fcb0a3f1e970666d82942bd94c162529d70473588d72331facaa5c76e |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-5-perf.csv | 55213978f972f23e8159446302d2a2d5a63b598d79f8640a6e50728facf88176 |
| benchmarks/results/g1-jacobi-column-slice-r72/native-production/long-5.json | 94f91825a33e0b357330fef91641c19e19b4f2590b27d98350bf85b1aae3cc8d |
