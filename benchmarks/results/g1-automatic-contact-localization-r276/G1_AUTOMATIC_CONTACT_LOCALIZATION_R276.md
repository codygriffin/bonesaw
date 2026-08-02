# G1 automatic contact localization · R276

**Mechanism passed / walking profile rejected.** R276 spends at most one normal-only solve per represented contact target, in stable target order, only after the ordinary full-lock solve is unfinished and only when at least two targets are represented.

| profile | probe ticks | probes | admitted target | fallback | release | root RMS m | foot RMS m | p99 ms |
|---|---|---|---|---|---|---|---|---|
| control | [] | 0 | [] | 875 | 1108 | 15.514 | 15.467 | 4.800 |
| automatic | [875] | 2 | [1] | 875 | 886 | 16.782 | 16.376 | 5.216 |

The default-off replay matches every one of the 81 shared R275 non-timing arrays and emits no probe. At tick 875 the enabled path rejects target 0/left, admits target 1/right on its second bounded probe, emits typed status 13, and keeps the left foot locked. Single-target failures are deliberately not probed.

The contact choice is correct but insufficient: left-only support reaches normal fallback at tick 885 and releases at 886, earlier than the control's tick 1108. Root/foot RMS regress to 16.782/16.376 m and p99 is 5.216 ms. The next slice must shape the pre-liftoff coupled root/CoM/support state; more failure-time contact selection is not the limiting behavior. No authority is admitted.
