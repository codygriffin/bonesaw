# Bonesaw support-hypothesis terminal envelope A/B · r193

> Mechanism **FAIL** · plant consequence **REJECTED** · synchronous profile **REJECTED**.

## Explicit uncertainty model

- Missing support evidence no longer collapses the plant to one fictitious contact mode. Rust fixed-effort dynamics realizes withhold, retained effort, and support-free effort under none/left/right/double support; Rust then takes a componentwise harm envelope and applies the unchanged conservative typed chooser.
- Every action × support query is state-local and bounded. MuJoCo remains an offline consequence oracle only. Unavailable alternatives fail closed, ballistic evidence remains common, and no envelope component is hidden inside one scalar score.
- Stored first-run work: **40,982 control ticks**; independently re-audited **266** online envelope selections.

## Plant consequence

| case | dropout | exact | r191 single | r193 envelope | r193 fall Δ s |
|---|---|---|---|---|---|
| nominal | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| nominal | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| forward_4n_reference | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop5 | RECOVERED | RECOVERED | RECOVERED | — |
| backward_4n | drop10 | RECOVERED | RECOVERED | RECOVERED | — |
| left_1n | drop5 | FALL 2.470s | FALL 1.790s | FALL 1.790s | -0.680 |
| left_1n | drop10 | FALL 2.470s | FALL 2.735s | FALL 2.735s | +0.265 |
| right_1n_mirror | drop5 | FALL 2.520s | FALL 2.170s | FALL 2.170s | -0.350 |
| right_1n_mirror | drop10 | FALL 2.520s | FALL 2.365s | FALL 2.365s | -0.155 |
| handle_forward_4n | drop5 | FALL 4.630s | FALL 5.970s | FALL 3.385s | -1.245 |
| handle_forward_4n | drop10 | FALL 4.630s | FALL 4.230s | FALL 5.805s | +1.175 |
| forward_4n_friction_0p03 | drop5 | FALL 1.610s | FALL 1.635s | FALL 1.700s | +0.090 |
| forward_4n_friction_0p03 | drop10 | FALL 1.610s | FALL 1.595s | FALL 1.595s | -0.015 |

## Runtime

- Twelve fixed-effort hypotheses / Rust aggregation maxima: **1591.422 / 0.922 µs**.
- Full loop: **62** 5 ms overruns; loop/controller maxima **6.475 / 5.940 ms**.
- Fixed-effort, scoring, aggregation, selection, and controller hot paths report zero Rust allocation; Python GC remains zero.
