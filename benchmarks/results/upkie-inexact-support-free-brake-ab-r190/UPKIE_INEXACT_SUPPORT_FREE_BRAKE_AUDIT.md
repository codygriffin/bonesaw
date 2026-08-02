# Bonesaw support-free observation-loss brake A/B · r190

> Mechanism **PASS** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Plant consequence

- Stored first-run work: **60,987 control ticks** across **77 profiles**, each with an exact replay; fresh brake selections: **247**.

| case | dropout | exact | support-free brake | fall Δ s |
|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | FALL 2.605s | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | FALL 5.645s | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 2.920s | +0.450 |
| left_1n | drop10 | FALL 2.470s | FALL 3.355s | +0.885 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.220s | -0.300 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.215s | -0.305 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 2.695s | -1.935 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 4.620s | -0.010 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.600s | -0.010 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.685s | +0.075 |

## Runtime

- Rust author / independent WBC maximum: **4.048 / 178.638 µs**.
- Full loop: **132** 5 ms overruns; loop/controller maxima **6.605 / 6.080 ms**.
- Two additional full-process repetitions produced the same plant outcomes: across all three runs, author/WBC maxima span **3.486–9.778 / 154.372–178.638 µs**, loop misses span **113–132**, and controller maxima span **5.779–6.261 ms**. The two retained loop maxima span **6.605–6.618 ms**; all realizations reject timing.
- Rust allocation and Python GC inside the measured authority path: **zero**.

## Contract

- Missing contact evidence authors a current-state flight-mode request: ballistic gravity, attitude damping, and joint damping, with no support centroid or fictitious contact impulse.
- A separate zero-contact floating WBC must admit the command on every missing tick before generic Rust authority emits typed selection 5.
- The command carries no contact-force witness and never refreshes cached Primary command age, health, or contact evidence.
- The matched 5/10 ms first-loss pair is bit-exact through the first brake command; no burst-duration oracle exists.
