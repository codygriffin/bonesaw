# Joint motion headroom witness · R288

> Mechanism **PASS** · allocation contract **PASS** · authority **NOT ADMITTED**.

R288 adds a default-off support-transfer witness in Rust. It combines one controller reaction interval, ideal stopping distance at the declared acceleration envelope, and authored joint velocity utilization. The result is evidence for the existing support-tube scale; it never clamps a command or turns a soft request into authority.

## Frozen CPU audit

| case | position margin rad | stopping margin rad | velocity fraction | conservative fraction | ns/call | alloc bytes | repeatability |
|---|---|---|---|---|---|---|---|
| centered | 1.260 | 1.260 | 1.000 | 0.500 | 77.2 | 0 | exact |
| moving | 0.560 | 0.440 | 0.861 | 0.175 | 87.9 | 0 | exact |
| near_limit | 0.060 | -0.540 | 0.583 | -0.214 | 87.5 | 0 | exact |

The Upkie model is evaluated with 20,000 warmed calls per case, joint-acceleration authority 200.0 rad/s², and 0.020 s reaction time. Centered → moving → near-limit headroom is strictly ordered; the near-limit case remains negative instead of being silently saturated. Across 5 CPU-4 process repeats, median cost is 77.3 ns/call and p99 is 87.8 ns/call.

The measured loop reports zero allocations (True), zero allocated bytes (True), and zero deallocations (True); repeated outputs are bitwise exact. Python performs only orchestration/reporting, with no policy or physics steps.

## Integration boundary

The feature remains opt-in through `support_trajectory_tube_motion_headroom`. Existing dormant and R280/R287 profiles do not execute it and therefore retain their established semantics. When enabled with a positive headroom floor, the support-transfer envelope uses the minimum of the legacy position margin and this stopping/velocity witness. A negative margin is evidence of an unrecoverable observation, not permission to invent authority.

This closes the missing typed joint-headroom input to the support-feasible tube. It does not claim that the G1 walking trace now meets its release, tracking, timing, contact, thermal, or plant gates; those remain separate qualification work.
