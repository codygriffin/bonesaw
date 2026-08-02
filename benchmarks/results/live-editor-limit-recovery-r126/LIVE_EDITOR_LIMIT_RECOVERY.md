# Live editor joint-limit recovery · r126

> **Admission: PASS.** 100 soft joint-drag hold/retreat/idempotent-release cycles completed without a reset, nonfinite state, hard-bound crossing, or latched drag.

- Limited joints checked: left_hip, left_knee, right_hip, right_knee
- Lower/upper coverage threshold: `≤ 0.030 rad`
- `left_hip`: lower 13× / 0.0132179 rad; upper 13× / 0.00314295 rad; max recovery 1 frame(s)
- `left_knee`: lower 13× / 0.00111503 rad; upper 13× / 0.00117204 rad; max recovery 1 frame(s)
- `right_hip`: lower 12× / 0.0206695 rad; upper 12× / 0.0203601 rad; max recovery 1 frame(s)
- `right_knee`: lower 12× / 0.0174992 rad; upper 12× / 0.00769263 rad; max recovery 1 frame(s)
- Handle coverage: `{'left_ankle_mj5208_rotor': 26, 'left_knee_qdd100_rotor': 26, 'right_ankle_mj5208_rotor': 24, 'right_knee_qdd100_rotor': 24}`
- Minimum streamed joint headroom: `0.00111503 rad`
- Maximum intent residual: `1.10405`
- Maximum represented joint speed: `19.4623 rad/s`
- Retreat recovery frames p50/p95/p99/max: `{'p50': 1.0, 'p95': 1.0, 'p99': 1.0, 'maximum': 1}`
- Idempotent releases: `100`
- Correlated drag/settle/release acknowledgements: `751`

Each row retains before/boundary/recovery/settled/released transition witnesses: solve status, active constraint rows, position and stopping margins, residuals, commanded velocity, backtracking scale/work, joint q/v, recovery latency, and release tick. The server streams joint q/v directly, so bounds are asserted on represented state rather than inferred from rendered pixels.
