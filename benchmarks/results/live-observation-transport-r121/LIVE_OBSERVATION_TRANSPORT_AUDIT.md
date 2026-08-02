# Bonesaw live observation transaction authority · r121

**PASS.** The real WebSocket/Rust WBC boundary exercised exact, local cubic
interpolation, bounded constant-velocity prediction, and a deliberately stale
producer without a policy or physics rollout. Every admitted reconstructed
state remained hard-eligible. Stale evidence was withheld before WBC at query
`780000000` ns with newest sample `770000000` ns,
then exact delivery recovered at tick `41` without restarting the session.

| transport | frames | command provenance | four command queries E/I/P/H | solve p50 / p99 µs | admission p50 / p99 µs |
|---|---:|---|---|---:|---:|
| exact | 12 | `{"exact": 12}` | `{"exact": 48, "held": 0, "interpolated": 0, "predicted": 0}` | 2157.315 / 3113.542 | 1897.185 / 2240.593 |
| interpolated | 12 | `{"interpolated": 12}` | `{"exact": 0, "held": 0, "interpolated": 48, "predicted": 0}` | 2546.441 / 3201.389 | 1810.887 / 2065.292 |
| predicted | 12 | `{"predicted": 12}` | `{"exact": 24, "held": 0, "interpolated": 0, "predicted": 24}` | 2622.811 / 4122.448 | 1866.302 / 2176.723 |

## Typed stale witness

```json
{
  "newest_sample_time_ns": 770000000,
  "query_time_ns": 780000000,
  "reason": "robot observation reconstruction withheld WBC input: robot observation prediction is disabled or exceeds its horizon",
  "tick": 39,
  "transport_mode": "stale",
  "type": "observation_withheld"
}
```

## Interpretation

- `exact` is the zero-lookback 5 ms producer path.
- `interpolated` deliberately queries 2.5 ms behind control time with two
  bracketing samples; downstream observation admission sees the same 2.5 ms age.
- `predicted` uses a 10 ms producer. The fourth 5 ms command query is a genuine
  5 ms prediction inside the declared horizon, and the streamed reconstruction,
  WBC, command admission, and selection retain that same query timestamp.
- `stale` pauses the producer. The controller clock continues advancing and
  emits a structured `observation_withheld` event when extrapolation exceeds
  5 ms; no held state reaches hard rows.

This measures state reconstruction and command authority, not plant tracking,
closed-loop stability, probabilistic uncertainty, or a physics simulator.
