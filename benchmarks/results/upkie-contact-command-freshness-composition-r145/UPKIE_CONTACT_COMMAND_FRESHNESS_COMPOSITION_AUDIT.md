# Bonesaw contact-grace + freshness composition sweep · r145

> Deployment **REJECTED**. Best nonzero row: 2 ticks (10 ms).

## Outcome

This gate keeps r143's exact-evidence contact-command lease separate from r137's already admitted continuous solver-freshness fade. A retained command executes first; after its bounded contact grace expires, r137 begins its five-to-twelve-tick fade. Reduced-support WBC output remains diagnostic-only. The zero-tick row must be execution-bit-exact with r141 withholding, while every nonzero row must preserve all r137 green outcomes and never move an existing fall earlier.

| case | r137 live | r141 withheld | grace 0 | grace 2 | grace 4 | grace 8 | grace 16 |
|---|---|---|---|---|---|---|---|
| nominal | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_2n | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_4n_reference | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_6n_overload | FALL 4.820s | FALL 1.365s | FALL 1.365s | FALL 1.360s | FALL 1.355s | FALL 1.350s | FALL 1.350s |
| backward_2n | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| backward_4n | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| left_1n | FALL 2.470s | FALL 1.810s | FALL 1.810s | FALL 1.800s | FALL 1.780s | FALL 1.770s | FALL 1.760s |
| left_2n | FALL 2.350s | FALL 1.415s | FALL 1.415s | FALL 1.410s | FALL 1.400s | FALL 1.395s | FALL 1.380s |
| left_4n | FALL 2.610s | FALL 1.685s | FALL 1.685s | FALL 1.645s | FALL 1.630s | FALL 1.605s | FALL 1.590s |
| right_2n | FALL 2.015s | FALL 1.670s | FALL 1.670s | FALL 1.665s | FALL 1.665s | FALL 1.635s | FALL 1.595s |
| right_4n | FALL 2.415s | FALL 1.395s | FALL 1.395s | FALL 1.410s | FALL 1.410s | FALL 1.365s | FALL 1.335s |
| diagonal_4n | FALL 2.125s | FALL 1.330s | FALL 1.330s | FALL 1.330s | FALL 1.330s | FALL 1.330s | FALL 1.330s |
| up_4n | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| down_4n | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| handle_forward_4n | FALL 4.630s | FALL 1.385s | FALL 1.385s | FALL 1.380s | FALL 1.380s | FALL 1.375s | FALL 1.375s |
| short_8n_50ms | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| long_2n_200ms | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_2n_three_pulses | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_4n_friction_0p1 | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED | RECOVERED |
| forward_4n_friction_0p03 | FALL 1.610s | FALL 0.315s | FALL 0.315s | FALL 0.315s | FALL 0.320s | FALL 0.325s | FALL 0.315s |

## Aggregate gates

| grace | green failures | new falls | earlier falls | min Δ s | sum Δ s | leased ticks | gate |
|---|---|---|---|---|---|---|---|
| 0 (0 ms) | 0 | 0 | 9 | -3.455 | -12.675 | 0 | FAIL |
| 2 (10 ms) | 0 | 0 | 9 | -3.460 | -12.730 | 68 | FAIL |
| 4 (20 ms) | 0 | 0 | 9 | -3.465 | -12.775 | 85 | FAIL |
| 8 (40 ms) | 0 | 0 | 9 | -3.470 | -12.895 | 124 | FAIL |
| 16 (80 ms) | 0 | 0 | 9 | -3.470 | -13.015 | 192 | FAIL |

## Boundary

- Exact absence never re-adds a hard row, and reduced-support diagnostics never execute.
- Contact grace and solver freshness are independent typed authorities; this experiment composes them sequentially instead of treating either as recovery.
- A negative result closes timeout tuning. The next candidate must change the solved physical action before sustained support loss.
