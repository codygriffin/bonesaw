# Bonesaw inexact-hold forecast-improvement gate A/B · r189

> Mechanism **PASS** · same-corpus consequence thresholds **NONE** · policy **REJECTED** · synchronous profile **REJECTED**.

## Threshold result

- Stored first-run work: **127,865 control ticks** across **154 profiles**, each with an exact replay; selector queries: **1,792**.
- Q15 selection counts across gated selector arms: **{'0': 1445, '8192': 12, '16384': 18, '24576': 2, '32768': 315}**.

| minimum improvement | 5 ms new green falls | 5 ms earlier | 10 ms new green falls | 10 ms earlier | both pass |
|---|---|---|---|---|---|
| 0.000 | 1 | 3 | 0 | 3 | NO |
| 0.001 | 0 | 1 | 0 | 3 | NO |
| 0.005 | 0 | 2 | 0 | 2 | NO |
| 0.025 | 0 | 2 | 0 | 2 | NO |
| 0.100 | 0 | 2 | 0 | 2 | NO |
| 0.250 | 0 | 2 | 0 | 2 | NO |
| 1.000 | 0 | 3 | 0 | 2 | NO |
| WITHHELD ENDPOINT | 0 | 3 | 0 | 2 | NO |

- Worst selector call: **8.847 µs**; Rust allocation and Python GC: **zero**.
- Timing: **192** 5 ms overruns, **6.134 ms** worst loop, and **5.699 ms** worst controller call.
- An independent full-process repetition produced the same plant outcomes with **210** overruns, a **9.448 µs** selector maximum, a **7.039 ms** loop maximum, and a **6.645 ms** controller maximum. Both timing realizations reject the 5 ms profile.
- The 1e9 endpoint selects zero authority on every query and is execution-exact to the explicit hold-0 control.
- Threshold points were derived from and evaluated on this same corpus; they are a calibration ablation, never independent promotion evidence.

## Contract

- Rust first chooses the exact lower-authority score argmin, then spends it only when predicted improvement over zero authority meets the configured finite nonnegative margin.
- The gate consumes no burst duration, future sample, physics rollout, policy state, hidden time, or heap allocation. The independent one-tick hold budget still withholds later missing samples even if the selector proposes nonzero authority.
- Rejection installs zero retained authority; it does not fall through to a cached lease, contact-force witness, or renewed primary health.
