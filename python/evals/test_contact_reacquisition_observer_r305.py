from __future__ import annotations

import pathlib
import sys
import unittest

import numpy as np


EVALS = pathlib.Path(__file__).resolve().parent
ROOT = EVALS.parents[1]
if str(EVALS) not in sys.path:
    sys.path.insert(0, str(EVALS))

try:
    import bonesaw  # noqa: F401

    HAS_RUNTIME = True
except ImportError:
    HAS_RUNTIME = False

from contact_reacquisition_observer_r305 import run_sequence  # noqa: E402


@unittest.skipUnless(HAS_RUNTIME, "the Rust extension is not installed")
class ContactReacquisitionObserverR305Tests(unittest.TestCase):
    def test_baseline_loss_loaded_return_and_rejected_evidence(self) -> None:
        result = run_sequence(ROOT / "models/upkie/upkie.urdf")
        self.assertEqual(result["baseline_status"], 0)
        self.assertEqual(result["armed_status"], 1)
        self.assertEqual(result["qualified_status"], 3)
        self.assertTrue(result["qualified"])
        self.assertEqual(result["rejected_status"], 4)
        self.assertFalse(result["rejected_qualified"])
        self.assertFalse(result["actuator_authority_emitted"])
        self.assertFalse(result["public_worker_defaults_changed"])

    def test_nan_load_is_fail_closed(self) -> None:
        import bonesaw

        session = bonesaw.UpkieBalanceSession(str(ROOT / "models/upkie/upkie.urdf"))
        session.configure_contact_reacquisition(2, 1.0, 0.5, 0.05)
        mask = np.asarray([1, 1], np.uint8)
        diagnostics = np.empty(13, np.float64)
        session.step_contact_reacquisition_from_loads(
            1,
            True,
            mask,
            mask,
            mask,
            mask,
            np.asarray([float("nan"), 20.0], np.float64),
            diagnostics,
        )
        # Status 4 is RejectedEvidence; no NaN can become a qualification.
        self.assertEqual(int(diagnostics[0]), 4)
        self.assertEqual(int(diagnostics[6]), 0)


if __name__ == "__main__":
    unittest.main()
