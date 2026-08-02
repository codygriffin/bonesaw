# Bonesaw surface-material fresh holdout · r246

> Runtime mechanism **PASS** · strict holdout **PASS** · authority **NOT ADMITTED**.

Two new pyramidal laws and disjoint offsets 230,000/240,000 were generated after R245 froze per-integrator work. Gap geometry is centre-minus-radius; friction motion includes ω×r at the instantaneous surface point; aref uses the current collision-boundary gap.

| law | integrator / cone ABI | sweeps | coverage | exact active | pred-only / missed | width · ang / lin / joint | p99 ms | decision |
|---|---|---|---|---|---|---|---|---|
| medium_pyramidal_implicitfast_surface_r246 | 0 / 2 | 64 | 100.000% | 48/48 | 0 / 0 | 0.059 / 0.007 / 4.949 | 0.871 | PROMOTE |
| rigid_pyramidal_rk4_surface_r246 | 4 / 2 | 32 | 100.000% | 48/48 | 0 / 0 | 0.065 / 0.010 / 2.239 | 3.399 | PROMOTE |

Prediction received causal arrays only. Strict coverage, five-millisecond p99, bitwise repeat, and zero Rust allocation are conjunctive. Plant selection and authority remain disabled.
