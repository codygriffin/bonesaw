# Bonesaw floating target-reacquisition localization · r267

> Per-target mechanism **PASS** · controller profile **REJECTED** · default **UNCHANGED** · authority **NOT ADMITTED**.

R267 is artifact-only. It separates three questions after R263: whether a different target can cross its own schedule edge after another target is suppressed, whether a larger Dykstra budget retains locked contact, and whether the existing equality-first accelerator is a promotable real-time controller profile.

The per-target edge is exercised: failed target 1 remains suppressed while target 0 enters Precontact at tick 428 after its genuine swing interval. It never locks and is released at tick 460. By then the support-free fallback has advanced for 116 ticks and the root ends at z = -5.277 m; a late schedule edge cannot undo a physical fall.

| Dykstra cap | status@300 | first contingency | p99 ms | max ms | >20 ms |
|---|---|---|---|---|---|
| 128 | 4 | 300 | 4.191 | 5.429 | 0 |
| 256 | 4 | 300 | 4.126 | 5.859 | 0 |
| 512 | 4 | 300 | 4.637 | 9.890 | 0 |
| 768 | 1 | 313 | 11.467 | 22.934 | 1 |
| 1024 | 1 | 313 | 11.360 | 30.055 | 1 |

The full-contact convergence knee is 768 sweeps: cap 512 still degrades at tick 300, while 768 solves it but creates a 22.9 ms deadline miss. Raising the cap is therefore rejected.

| active-set iterations | first contingency | p99 ms | max ms | >5 ms | >20 ms |
|---|---|---|---|---|---|
| 4 | 302 | 5.396 | 6.501 | 5 | 0 |
| 8 | 302 | 5.516 | 11.487 | 5 | 0 |
| 16 | 330 | 10.736 | 19.787 | 25 | 0 |
| 64 | 330 | 10.650 | 55.337 | 25 | 1 |

Equality-first repair with 16 iterations retains full contact through tick 329, but the native 600-tick profile has 270 contingency ticks, 23.264 ms p99, 33.291 ms maximum, and 51 twenty-millisecond misses. R165 already rejected this seed rule after 11 plant fall boundaries moved earlier. R267 therefore keeps it diagnostic and default-off.

The remaining functional problem precedes recovery: the authored single-support motion becomes badly untrackable before tick 300. The next controller slice should improve causal support/CoM/reference compatibility while preserving the R262 bounded fail-closed timing contract; neither a late reacquisition edge nor a semantic feasibility shortcut is sufficient.
