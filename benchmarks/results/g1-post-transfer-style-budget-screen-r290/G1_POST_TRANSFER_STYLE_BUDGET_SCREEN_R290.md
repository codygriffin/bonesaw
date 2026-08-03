# G1 causal post-transfer Style budget profile screen · R290

> Mechanism PASS · active finite profiles REJECTED · authority NOT ADMITTED.

R290 adds a default-off continuous degradation boundary: a support-tube intent clip arms the already-typed Style projected-solve ceiling only after the clipping tick completes. Nominal Preference/Style work remains unbounded before that causal event, and every trace/reset restores the nominal configuration. Rust owns the hot-boundary update; Python owns immutable profile orchestration and reporting.

## Profile screen

| profile | first exhaustion tick | exhausted ticks | release ticks | root RMS m | root Δ | p99 range µs | changed common arrays | decision |
|---|---|---|---|---|---|---|---|---|
| Style-1 | 820 | 3 | 875, 1134, 1734, 2295 | 18.783 | +41.35% | 4852.1–5185.5 | 61 | REJECT |
| Style-2 | 1152 | 1 | 1210, 1719, 2293 | 14.623 | +10.05% | 5220.2–5450.9 | 61 | REJECT |
| Style-3 | None | 0 | 1234, 1694, 2296 | 13.288 | +0.00% | 5387.9–5387.9 | 0 | INERT |
| Style-4 | None | 0 | 1234, 1694, 2296 | 13.288 | +0.00% | 5256.3–5256.3 | 0 | INERT |
| Style-6 | None | 0 | 1234, 1694, 2296 | 13.288 | +0.00% | 5218.1–5218.1 | 0 | INERT |
| Style-8 | None | 0 | 1234, 1694, 2296 | 13.288 | +0.00% | 5343.2–5343.2 | 0 | INERT |

The first support-tube intent clip is tick 819. Style-1 first exhausts at tick 820—never on the clipping tick—and repeats one non-timing digest across all five runs. Four p99 values fall below 5 ms, but one reaches 5185.5 µs (mean 4944.9 µs), so the repeatability gate fails. It also moves the first release 1234→875 and worsens root RMS +41.35%.

Style-2 first exhausts at tick 1152 and is likewise bitwise repeatable. It still misses 5 ms (5220.2–5450.9 µs), moves first release 1234→1210, and worsens root RMS +10.05%. Style-3/4/6/8 never exhaust and preserve every common non-timing array, so they provide no work reduction.

## Boundary and decision

The mechanism is retained because it supplies the requested event-driven, reset-safe degradation surface without capping higher authority or changing dormant behavior. No measured finite profile is promoted: Style-1 misses the repeatable timing gate and destroys transfer, Style-2 changes behavior while remaining above 5 ms, and higher ceilings are inert. There are zero typed primal-infeasible/failed ticks in the active profiles, but hard feasibility alone is not tracking or command authority.

The corpus executes no policy and no physics. Contact, actuator, thermal, plant, authenticated transport, browser frame-time, and hardware realization remain separate open gates.
