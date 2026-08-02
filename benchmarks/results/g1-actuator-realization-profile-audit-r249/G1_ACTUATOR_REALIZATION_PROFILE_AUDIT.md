# Bonesaw actuator realization profile audit · r249

> Rust realization mechanism **PASS** · plant **NOT RUN** · authority **NOT ADMITTED**.

R249 replays 96 selected R248 efforts through Rust's first-order bandwidth and slew boundary at the 20 ms / 50 Hz WBC cadence. Every row resets the complete realized-effort state before evaluation. No policy or MuJoCo step runs; the failed R248 plant labels remain immutable.

| profile | response norm · p50 / p99 | max |error| Nm | slew-limited coordinates | p99 µs | decision |
|---|---|---|---|---|---|
| ideal_no_bandwidth_no_slew | 1.000 / 1.000 | 0.000 | 0 | 2.20 | PASS |
| bandwidth_50hz_no_slew | 0.998 / 0.998 | 0.041 | 0 | 0.42 | PASS |
| bandwidth_25hz_slew_1000_nm_s | 0.957 / 0.957 | 1.814 | 3 | 3.48 | PASS |
| bandwidth_12hz_slew_500_nm_s | 0.779 / 0.779 | 11.814 | 60 | 2.26 | PASS |

The selected construction profile for the next fresh plant matrix is **bandwidth_25hz_slew_1000_nm_s**. This is a declared sensitivity profile, not G1 actuator calibration and not authority; the next gate must apply it without retuning on new plant laws and offsets.
