# Live plant fault-recovery supplement · r131

**Result: PASS at the declared simulator boundary.** The deployed Rust gateway
and Python/MuJoCo worker were exercised once through localhost and once through
the public Cloudflare path after the five-session 4 N admission audit.

| path | push ack | dead-man expiry | peak displacement | peak tilt | fall reset | settled station error | reconnect |
|---|---:|---:|---:|---:|---:|---:|---|
| localhost | 19.19 ms | 160.62 ms | 0.3306 m | 75.46° | 1 | −0.0243 mm | fresh tick 1 in 173.0 ms |
| Cloudflare | 32.91 ms | 170.69 ms | 0.2768 m | 78.45° | 1 | −0.0243 mm | fresh tick 1 in 371.1 ms |

The sequence deliberately sends an invalid `8.01 N` command first, proves the
stream remains alive, lets a valid `2 N` command expire without a release,
then refreshes a sustained `5 N` body wrench. The model crosses the declared
fall threshold. That state is streamed with `fallen=true`; the worker resets
on the following stream step, increments its fall-reset counter, and returns
to the recovery envelope without disconnecting the WebSocket. An explicit
reset is then correlated, the session is closed, and a new connection starts a
fresh isolated worker.

This is a simulator fault-containment gate, not a hardware recovery claim.
Automatic reconstruction is intentionally visible in `plant_state`; it never
silently rewrites a guided TARGET pose. Browser visual/touch/frame-time QA,
lateral disturbances, arbitrary application-point moments, slopes, friction,
delay/noise, thermal limits, and authenticated hardware leases remain open.

Raw results: [local.json](local.json) and [public.json](public.json).
