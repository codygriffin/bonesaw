# Bonesaw G1 model-coupled positive-reference audit · r230

> Mechanism **PASS** · fresh-holdout profile **FROZEN** · authority **NOT ADMITTED** · new MuJoCo/policy/controller/selector/plant steps **0 / 0 / 0 / 0 / 0**.

## Contract

- Rust owns the floating state, five authored 1 ms event steps, sphere support geometry, rigid point velocity, convective/free acceleration, floating inverse dynamics, full Delassus refresh, projected contact solve, and state output. Collision membership is sampled once at the beginning of each state step; inner projection sweeps never become hidden collision-detector calls.
- R228 is a spent construction corpus. Completed impulses score the selected result only. Sweep selection uses only prediction change, a 2% fraction of the useful-width gates, measured deadline, bitwise repeat, and zero timed Rust allocation.
- One compliant update per 1 ms state tick follows the authored integrator clock. Subdividing a physics tick would change event semantics and is not an eligible convergence knob.
- A complete retained rerun reproduced all 106 non-timing NPZ arrays exactly. Timing is retained as measured evidence and excluded from semantic equality.
- A retained independent rerun reproduces all 106 non-timing arrays and normalized non-timing metrics exactly. Timing arrays and rendered timing cells remain measured evidence and are excluded from semantic equality.

## Causal sweep selection

| sweeps | double-sweep width ω/v/joint | largest gate fraction | p99 ms | decision |
|---|---|---|---|---|
| 1 | 0.1306 / 0.0252 / 14.9907 | 149.907% | 0.549 | — |
| 2 | 0.1413 / 0.0280 / 10.3458 | 103.458% | 0.556 | — |
| 4 | 0.2118 / 0.0313 / 3.3521 | 33.521% | 0.835 | — |
| 8 | 0.1910 / 0.0268 / 1.1354 | 11.354% | 0.864 | — |
| 16 | 0.0838 / 0.0113 / 0.4740 | 4.740% | 0.579 | — |
| 32 | 0.0101 / 0.0017 / 0.1619 | 1.619% | 0.899 | SELECT |
| 64 | — | — | 0.927 | — |

## Selected spent-label score

| law | exact active sets | old frozen coverage | fitted width ω/v/joint | p99 ms |
|---|---|---|---|---|
| soft_pyramidal_euler | 47/48 | 100.000% | 0.038 / 0.008 / 7.271 | 0.618 |
| stiff_elliptic_implicitfast | 48/48 | 97.917% | 0.168 / 0.023 / 9.893 | 0.899 |

## Decision

The independently selected state-refresh construction fits inside every useful-width gate and is frozen for a genuinely fresh law/state holdout. This is not command authority.
