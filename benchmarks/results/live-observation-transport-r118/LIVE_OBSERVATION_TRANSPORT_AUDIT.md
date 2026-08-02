# Bonesaw live observation transport authority · r118

**PASS.** The real WebSocket/Rust WBC boundary exercised exact, local cubic
interpolation, bounded constant-velocity prediction, and a deliberately stale
producer without a policy or physics rollout. Every admitted reconstructed
state remained hard-eligible. Stale evidence was withheld before WBC at query
`745000000` ns with newest sample `735000000` ns,
then exact delivery recovered at tick `38` without restarting the session.

| transport | frames | stream provenance | WBC + stream E/I/P/H | solve p50 / p99 µs | admission p50 / p99 µs |
|---|---:|---|---|---:|---:|
| exact | 12 | `{"exact": 12}` | `{"exact": 60, "held": 0, "interpolated": 0, "predicted": 0}` | 1663.759 / 2273.830 | 1224.435 / 2190.974 |
| interpolated | 12 | `{"interpolated": 12}` | `{"exact": 0, "held": 0, "interpolated": 60, "predicted": 0}` | 2031.483 / 2984.401 | 1427.327 / 1624.155 |
| predicted | 12 | `{"predicted": 12}` | `{"exact": 25, "held": 0, "interpolated": 0, "predicted": 35}` | 2086.743 / 3069.441 | 1421.226 / 1541.127 |

## Typed stale witness

```json
{
  "newest_sample_time_ns": 735000000,
  "query_time_ns": 745000000,
  "reason": "robot observation reconstruction withheld WBC input: robot observation prediction is disabled or exceeds its horizon",
  "tick": 37,
  "transport_mode": "stale",
  "type": "observation_withheld"
}
```

## Interpretation

- `exact` is the zero-lookback 5 ms producer path.
- `interpolated` deliberately queries 2.5 ms behind control time with two
  bracketing samples; downstream observation admission sees the same 2.5 ms age.
- `predicted` uses a 10 ms producer offset by 5 ms, making the 20 ms streamed
  boundary a genuine 5 ms prediction inside the declared horizon.
- `stale` pauses the producer. The controller clock continues advancing and
  emits a structured `observation_withheld` event when extrapolation exceeds
  5 ms; no held state reaches hard rows.

This measures state reconstruction and command authority, not plant tracking,
closed-loop stability, probabilistic uncertainty, or a physics simulator.
