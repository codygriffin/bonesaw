# G1 support-policy sweep

Predeclared `600`-tick strict floating-WBC comparison. Every case uses the same official Unitree G1 model and CMU-derived target trace; only the named support policy changes.

A case is green only if the unchanged floating-walk acceptance gate passes. Avoiding infeasibility while remaining in normal-only touchdown or drifting away from the reference is not counted as success.

| case | gate | nominal ticks | touchdown max | normal | release | infeasible | root RMS cm | stance RMS cm | swing RMS cm | rotation deg | centroidal residual RMS/max N·m | p50 ms | p99 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline | FAIL | 330 | 3 | 27 | 111 | 131 | 156.891 | 106.353 | 143.447 | 144.209 | 0.000/0.000 | 11.019 | 225.371 |
| centroidal-intent-0p1 | FAIL | 348 | 16 | 53 | 91 | 92 | 171.664 | 90.505 | 173.748 | 179.434 | 48.209/233.680 | 2.501 | 224.594 |
| centroidal-intent-1 | FAIL | 350 | 3 | 33 | 51 | 166 | 117.682 | 71.208 | 126.229 | 179.685 | 46.211/106.754 | 2.506 | 227.745 |
| support-preview-0p1 | FAIL | 452 | 24 | 99 | 49 | 0 | 48.630 | 28.913 | 68.286 | 179.230 | 0.000/0.000 | 2.669 | 163.292 |
| support-preview-0p1-centroidal-1 | FAIL | 532 | 104 | 51 | 17 | 0 | 45.616 | 46.114 | 70.212 | 17.719 | 32.173/105.394 | 11.289 | 136.103 |

## Interpretation

The sweep separates three claims: exact external-wrench task support, a longer nominal window, and a completed walking transfer. The first can pass unit and oracle checks while the latter two remain red. The canonical policy is not changed unless a case passes the whole gate.

Each case directory retains its full Markdown report, metrics JSON, and compressed per-tick NPZ trace.
