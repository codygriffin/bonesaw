# G1 native-reference integrated bridge · R268

**Mechanism PASS / controller profile REJECTED.** This evaluation uses no policy and no physics simulator. The Rust controller integrates its own state; only the immutable Rust-LIPM reference and the first state of the offline morphology witness initialize it. The optional posture case consumes only the witness q/v/q̈ jet, never oracle WBC force, status, or solved acceleration outputs.

## Result

| profile | first contingency tick | prefix root RMS cm | prefix max attitude deg | p99 ms | >20 ms |
|---|---|---|---|---|---|
| initialization only | 501 | 7.890 | 7.743 | 4.884 | 2 |
| oracle task stack | 188 | 2.879 | 1.269 | 5.028 | 2 |
| morphology posture jet | 439 | 4.849 | 7.424 | 4.836 | 3 |

Initialization-only extends the retained native-reference clean prefix from **265** to **501 ticks** and reaches the authored liftoff at tick 300 with 0.003 mm root error. It still releases support before the first touchdown edge at tick 529, so no profile is promoted.

Every explicit release/fallback tick clears rejected contact residuals before integration: the regenerated traces report zero dynamics and contact residual witnesses on all status-5 ticks. This is diagnostic hygiene, not a claim that the free-body state is physically supported.

The exact R54 oracle task stack is not a closed-loop policy: it fails earlier. Directly replaying the policy-free morphology q/v/q̈ witness as a Preference posture jet also fails earlier. Both are retained as causal negative controls, not averaged into a score.

## Contract

- Reference SHA-256: `429baba94dae0c3b4e490dc7f0bc107416d3c0559fbfbad4df6e009fac3e48c6`.
- Witness SHA-256: `f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295`.
- Initial morphology error: 4.042 mm foot / 18.446 mm CoM; both remain inside the retained 10/30 mm certificate.
- Authored root position, velocity, and acceleration jets now cross the standalone boundary independently; CoM derivatives are no longer substituted for pelvis derivatives.
- The optional morphology posture jet is sampled and time-warped in allocation-free Rust under the same reference cursor. It remains opt-in because this profile is red.
- Defaults and execution authority are unchanged. The next controller slice must preserve root attitude/support through the final 90 ticks of the first swing without importing oracle WBC outputs or weakening touchdown/contact constraints.
