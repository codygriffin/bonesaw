# R305 Rust contact-reacquisition observer

R305 closes the evidence seam identified by the R302 recovery eval without
claiming recovery. The allocation-free Rust observer is downstream of the
existing contact observation filter and upstream of command authority. It
accepts an explicit target support mask, raw/stable/hard contact masks, and
measured normal loads. It reports a witness only after three consecutive
monotonic exact samples in which every target contact carries at least 0.5 N,
at least 5% of the total load, and the total load is at least 1 N.

Startup support is a baseline, not reacquisition. A valid target loss arms the
observer. A geometric touch with zero load, inexact evidence, NaNs, and
reordered ticks remain non-qualified. A rejected sample never publishes a
sticky previous `qualified=true` bit. The observer does not write a torque,
change the worker's 250/50 Hz defaults, or bypass ordinary WBC admission.

The synthetic contract eval is reproducible with:

```bash
PYTHONPATH=python/evals \
  /tmp/bonesaw-mujoco/bin/python \
  python/evals/contact_reacquisition_observer_r305.py \
  --model models/upkie/upkie.urdf
```

Observed status sequence: `Baseline → Armed → Candidate → Candidate →
Qualified`; a subsequent inexact sample is `RejectedEvidence` with
`qualified=false`. This is a bounded admission witness, not a recovery claim.
