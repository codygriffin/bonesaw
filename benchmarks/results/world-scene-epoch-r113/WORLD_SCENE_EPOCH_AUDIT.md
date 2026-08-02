# Bonesaw versioned world-scene snapshot authority · r113

## Outcome

**PASS.** One immutable SDF snapshot now carries an independent scene epoch, source timestamp, and closed validity interval in smooth `control_world`. The command transaction checks the expected epoch, source causality, current validity, maximum age, and full 20 ms horizon coverage before either Primary or brake can gain authority. There is no policy, physics engine, state integration, map-frame jump, or plant rollout.

| case | typed validity | selection | scene/unknown flags |
|---|---|---|---|
| valid | valid | Primary | False / False / False |
| epoch_mismatch | epoch_mismatch | Rejected | True / True / True |
| source_from_future | source_from_future | Rejected | True / True / True |
| not_yet_valid | not_yet_valid | Rejected | True / True / True |
| expired_at_tick | expired_at_tick | Rejected | True / True / True |
| horizon_expired | horizon_expired | Rejected | True / True / True |
| too_old | too_old | Rejected | True / True / True |

The valid snapshot selects Primary. Epoch mismatch, future source time, not-yet-valid data, expiry at the tick, expiry inside the command horizon, and excessive age all reject both plans with a dedicated scene-invalid bit plus independent primary/brake world-unknown bits. A malformed validity interval is rejected at construction.

## Timing and allocation

The complete valid WBC + actuator/root prediction + robust two-plan world transaction ran 2000 times at `14.808/20.529/24.787` µs p50/p99/max. Semantic replay was exact: **True**. Maximum timed allocation calls/bytes were `0/0`.

## Authority boundary

The scene epoch is not the MotionProgram epoch and is not `map` or `odom`. Global localization corrections remain external-frame observations; they cannot jump the smooth WBC root. R113 versions the immutable field consumed by local and segment queries and proves freshness fail-closed. It does not yet hot-swap field storage inside one controller call, interpolate scene epochs, or sweep moving-obstacle primitives. Those require an explicit fixed-capacity scene-view input and velocity-aware obstacle evidence.
