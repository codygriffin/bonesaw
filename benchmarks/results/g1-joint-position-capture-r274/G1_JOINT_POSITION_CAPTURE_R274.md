# G1 joint-position stopping-headroom capture · R274

**Mechanism passed / walking profile rejected.** R274 adds a default-off soft Viability request when directional stopping distance plus a bounded reaction guard exceeds the remaining authored position headroom. Existing hard joint intervals remain the authority boundary.

## Result

| profile | weight | brake | reaction s | first active | active ticks | knee limit | fallback | release | root RMS m | p99 ms |
|---|---|---|---|---|---|---|---|---|---|---|
| r273 baseline | 0.00 | 50 | 0.02 | 2317 | 0 | 874 | 875 | 1108 | 15.514 | 4.714 |
| r274 dormant | 0.00 | 50 | 0.02 | 2317 | 0 | 874 | 875 | 1108 | 15.514 | 4.719 |
| r274 w025 b25 r02 | 0.25 | 25 | 0.02 | 859 | 17 | 875 | 876 | 955 | 18.094 | 5.075 |
| r274 w025 b50 r02 | 0.25 | 50 | 0.02 | 863 | 13 | 875 | 888 | 889 | 18.989 | 5.187 |
| r274 w025 b50 r04 | 0.25 | 50 | 0.04 | 859 | 17 | 875 | 888 | 889 | 18.560 | 5.124 |
| r274 w05 b50 r02 | 0.50 | 50 | 0.02 | 863 | 13 | 875 | 876 | 1020 | 18.748 | 5.399 |
| r274 w1 b50 r02 | 1.00 | 50 | 0.02 | 863 | 13 | 875 | 876 | 1012 | 14.886 | 4.955 |

The disabled replay matches every shared R273 non-timing array exactly and emits zero capture-active coordinates. Every enabled trace is also exact through its first measured activation. The mechanism is therefore dormant and causally timed rather than a hidden retiming of the reference.

No retained profile prevents the right-knee limit: enabled rows reach it at ticks 875–875 versus baseline 874. The best release is tick 1020, still before baseline 1108. Root RMS remains 14.886–18.989 m. No profile or authority is admitted.

## Execution contract

- Rust computes one scalar stopping-headroom request per selected joint without allocation; zero weight skips the path exactly.
- The request shares the existing fixed Viability joint-task slot with velocity protection. Same-direction requests retain the stronger acceleration; the hard velocity-braking interval is unchanged.
- Capture active-coordinate count is a dedicated raw trace, separate from velocity-envelope activity, solver status, position-limit contact, tracking, and timing.
- The replay consumes the immutable reference and initial-state witness with zero policy and zero physics steps.
