# Bonesaw native WBC counter sentinel · r59

## Result

Five native release trials on the official 23-DOF G1 model each execute 2,000 fixed-shape floating WBC solves. After subtracting an adjacent one-tick process, the marginal median is 21.991 million instructions, 6.099 million cycles, and 1.505 ms task-clock per additional tick. Native measured latency is 2107.002 µs p99 median across trials. Every long trial has the same non-timing semantic report, zero infeasible ticks, bitwise final-repeat agreement, and zero solve-loop allocations.

> This is a native marginal upper bound, not direct single-solve hardware sampling. It includes the per-loop timer and semantic aggregation around each solve, while subtracting model load, process startup, and serialization using an adjacent one-tick process.

## Repeated native trials

| trial | instructions / tick M | cycles / tick M | task-clock / tick ms | IPC | p50 / p99 / max µs | alloc calls / bytes |
|---|---|---|---|---|---|---|
| 1 | 21.991 | 8.980 | 2.187 | 2.449 | 2178.467 / 2357.405 / 2400.467 | 0 / 0 |
| 2 | 21.991 | 6.853 | 1.661 | 3.209 | 1414.203 / 2318.893 / 2371.513 | 0 / 0 |
| 3 | 21.991 | 5.884 | 1.439 | 3.737 | 1421.647 / 1742.322 / 1940.407 | 0 / 0 |
| 4 | 21.991 | 6.099 | 1.505 | 3.606 | 1435.423 / 2107.002 / 2324.874 | 0 / 0 |
| 5 | 21.991 | 5.890 | 1.451 | 3.734 | 1428.239 / 1718.889 / 2941.900 | 0 / 0 |

## Scope and decision

- Model fingerprint: `9e1ba655204c01f52fa2493582affc237b2343da6fb9743d0fa4723d69660a03`; 23 actuated coordinates, 29 generalized accelerations, two locked ankle contacts, and 58 decision variables.
- The sentinel keeps one redundant Cartesian Preference task active, varies a deterministic nominal tangential force, and retains full floating dynamics/contact/torque feasibility.
- It does not reproduce r54 finite support patches or the complete Invariant/Viability/Preference/Style task stack; r54 remains the behavioral admission authority.
- Hardware-counter optimization may use this sentinel only if the semantic report, allocation gate, residuals, status, and separate r54 byte-exact trace remain unchanged.
- The next optimization target is the one-sided Jacobi/pseudoinverse dense kernel, selected using r54 work correlation and this native counter baseline—not a clock-triggered early exit.

## Artifact integrity

| artifact | SHA-256 |
|---|---|
| /home/codygriffin/Documents/bonepilot/target/release/bonesaw-eval | 46b19a91efb9db3ec70aa12b3b943e1811200a46bb4e9e729e6f3ff83e4084a8 |
| /home/codygriffin/Documents/bonepilot/benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf | 9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43 |
| benchmarks/results/g1-native-wbc-counter-r59/short-1-perf.csv | 88fa83c45c32f350591c0c5b84a0a7123fab71fef8299a9f6ec7ceeedfb63354 |
| benchmarks/results/g1-native-wbc-counter-r59/short-1.json | 38e33226c7b1c99ebf9a3459322ec3b6e5676fd66dedbf4a3e908b78c6c893c6 |
| benchmarks/results/g1-native-wbc-counter-r59/long-1-perf.csv | 09512b2885aa57fba74bf35085be89b3e88a66e42eeb29d941a81d9b012f3087 |
| benchmarks/results/g1-native-wbc-counter-r59/long-1.json | 88fb836ccb866a91bf020252b7810df9634c3711064c52856f949b5d952f33dd |
| benchmarks/results/g1-native-wbc-counter-r59/short-2-perf.csv | f23503de8cdf7819aec846f5c573fea02e5feb06436afb81d7c78fa5d5d3675c |
| benchmarks/results/g1-native-wbc-counter-r59/short-2.json | 6aa7c6da78d6eb263b74d8fd7fcd8b29863194631f632b11d1af63c3e861e791 |
| benchmarks/results/g1-native-wbc-counter-r59/long-2-perf.csv | db1b7b5965a7253051caa5390de0fcd86614812ffeb2e3c41d849297d7fd0822 |
| benchmarks/results/g1-native-wbc-counter-r59/long-2.json | 15af7aa3199199010aa208d7da72496463c67a9a7598bd517c7722529db7edd9 |
| benchmarks/results/g1-native-wbc-counter-r59/short-3-perf.csv | db465c92cc6bbf6327a6e21660eedb9c95c23366ab5f83a77aa76fd0d03530aa |
| benchmarks/results/g1-native-wbc-counter-r59/short-3.json | d184a29d00573c019115263fdcb5db3bd60791c6cf63d764f73ed30a863480bc |
| benchmarks/results/g1-native-wbc-counter-r59/long-3-perf.csv | 7ab36af9339f2395f610e8627904136224c7130d879bed51102621783f784eb5 |
| benchmarks/results/g1-native-wbc-counter-r59/long-3.json | ea4c0f8d0cb6df45f31654222833218c674fc26194dbb9a961c8746c2aab495e |
| benchmarks/results/g1-native-wbc-counter-r59/short-4-perf.csv | 9365d9b8218a6049aa478120f59fd946f117ae026e5f28fb98307b6a50732825 |
| benchmarks/results/g1-native-wbc-counter-r59/short-4.json | a09f59a606fbeb83e2dd3182583e7c8f58d42898d27e94b9ec1c288268df1fb4 |
| benchmarks/results/g1-native-wbc-counter-r59/long-4-perf.csv | 30896e953eba966a5811b0ed339bd2350897d76d0bf2b522b8826de3f255ada3 |
| benchmarks/results/g1-native-wbc-counter-r59/long-4.json | c0706f773843a2d0045a27b35e515f463d5ec62dd1fcce649276f87f618a3db1 |
| benchmarks/results/g1-native-wbc-counter-r59/short-5-perf.csv | 1b6689113ebc0507e64305c8f8e8d9884daa10c2ad28a507f6123b0d20b3f81f |
| benchmarks/results/g1-native-wbc-counter-r59/short-5.json | 434e62f2db96cd04335af1ae859440bfebd14a2d5d31b8c1c50c64d8371989b9 |
| benchmarks/results/g1-native-wbc-counter-r59/long-5-perf.csv | de7cea5fbe5a6125b9d70947001dd0e3163761ab31107ece1ed5f9fd51dda7fd |
| benchmarks/results/g1-native-wbc-counter-r59/long-5.json | 649b924f241bc28615d35f00e5983ef6757bd0da8fdc01c091e69a58228739f5 |
