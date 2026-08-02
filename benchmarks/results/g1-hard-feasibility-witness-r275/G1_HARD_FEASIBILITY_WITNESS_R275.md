# G1 hard-feasibility witness · R275

**Diagnostic mechanism passed; recovery profile remains closed.** R275 separates anonymous coordinate-bound residue from named hard-row residue before contingency retries overwrite the first solve. The 8-sweep control remains byte-identical to R274 on all 74 shared non-timing arrays.

## Result

| profile | first hard fail | first named row | bound coord | fallback | release | root RMS m | foot RMS m | p99 ms |
|---|---|---|---|---|---|---|---|---|
| control cap8 | 875 | dynamics[3] | None | 875 | 1108 | 15.514 | 15.467 | 4.734 |
| control cap16 | 875 | dynamics[5] | None | 875 | 1108 | 15.514 | 15.467 | 4.908 |
| control cap64 | 875 | dynamics[5] | None | 875 | 1108 | 15.514 | 15.467 | 5.001 |
| hard capture w025 | 869 | dynamics[3] | 15 | 869 | 1006 | 17.141 | 16.778 | 4.840 |
| local left | 875 | dynamics[3] | None | None | 927 | 16.980 | 16.569 | 5.427 |
| local right | 875 | dynamics[3] | None | 875 | 885 | 16.423 | 16.005 | 6.591 |
| accel 1000 | 875 | dynamics[3] | None | None | 875 | 16.681 | 16.283 | 6.382 |
| torque 20000 | 875 | dynamics[3] | None | 875 | 1108 | 15.514 | 15.467 | 4.924 |
| normal 30x | 875 | dynamics[3] | None | 875 | 1108 | 15.514 | 15.467 | 4.963 |
| friction 2 | 887 | dynamics[4] | None | 887 | 1076 | 11.365 | 11.309 | 6.147 |
| friction 3 | 911 | dynamics[5] | None | None | 911 | 18.770 | 18.486 | 6.979 |
| friction 5 | 978 | dynamics[3] | 8 | 978 | 1039 | 13.532 | 13.309 | 6.953 |
| friction 10 | 950 | dynamics[5] | None | 950 | 1408 | 9.829 | 9.732 | 7.298 |
| one point | 176 | dynamics[3] | None | 176 | 337 | 46.537 | 46.126 | 4.622 |

At tick 875 the retained control does not finish the hard problem. Its terminal bounded witness has zero coordinate-bound violation and a 13.781 violation of `dynamics[3]`, the floating-base world-X dynamics equality. Raising the projection cap to 16 or 64 changes the terminal witness but not the state, status, support phase, or failure tick. Raising torque to 20,000 or aggregate normal capacity to 30× is fully trace-exact, so neither resource is the active cause in this state stream.

The per-foot ablation is sharper. Making only the right-foot rows normal-only yields an executable contingency at tick 875 while the left foot stays locked. Trying the left foot instead still fails and requires a typed left-support release. This localizes the first coupled conflict to right-foot tangential locking against floating-base dynamics; it does not yet authorize an automatic selector.

A hard stopping-envelope variant activates the right-knee generalized acceleration bound (solver coordinate 15 = root 6 + joint 9) and fails six ticks earlier at 869 while root RMS worsens. Higher friction delays the first event to ticks 887–978, but every sweep still releases/fails later, changes the physical assumption, and misses the 5 ms profile. One-point contact and wider acceleration bounds also regress. No recovery profile or authority is admitted.

## Execution contract

- Rust scans the terminal fixed solver workspace without allocation and records deterministic bound coordinate/side plus named stable row/side.
- Python captures the first solve before normal-only, localized-handoff, or release retries overwrite it.
- The witness is diagnostic only: it cannot change contact mode, solve admission, integration, plant commands, or authority.
- All replays consume the immutable R53 reference and R54 initial state with zero policy steps and zero physics steps.
