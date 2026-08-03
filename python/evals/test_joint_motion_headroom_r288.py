import unittest

from joint_motion_headroom_r288 import qualify


def audit() -> dict:
    cases = [
        {
            "name": "centered",
            "position_margin_rad": 1.26,
            "stopping_margin_rad": 1.26,
            "velocity_fraction": 1.0,
            "fraction_of_range": 0.5,
            "nanoseconds_per_call": 150.0,
            "measured_allocation_calls": 0,
            "measured_allocated_bytes": 0,
            "measured_deallocation_calls": 0,
            "repeated_result_bitwise_equal": True,
        },
        {
            "name": "moving",
            "position_margin_rad": 0.56,
            "stopping_margin_rad": 0.44,
            "velocity_fraction": 0.86,
            "fraction_of_range": 0.17,
            "nanoseconds_per_call": 150.0,
            "measured_allocation_calls": 0,
            "measured_allocated_bytes": 0,
            "measured_deallocation_calls": 0,
            "repeated_result_bitwise_equal": True,
        },
        {
            "name": "near_limit",
            "position_margin_rad": 0.06,
            "stopping_margin_rad": -0.54,
            "velocity_fraction": 0.58,
            "fraction_of_range": -0.21,
            "nanoseconds_per_call": 150.0,
            "measured_allocation_calls": 0,
            "measured_allocated_bytes": 0,
            "measured_deallocation_calls": 0,
            "repeated_result_bitwise_equal": True,
        },
    ]
    return {
        "schema": "bonesaw.joint-motion-headroom-r288.v1",
        "model": "upkie",
        "repetitions_per_case": 20000,
        "maximum_acceleration_mps2": 200.0,
        "reaction_time_seconds": 0.02,
        "cases": cases,
    }


class JointMotionHeadroomR288Tests(unittest.TestCase):
    def test_qualification_accepts_conservative_ordering(self):
        result = qualify([audit()] * 5, 4)
        self.assertTrue(result["allocation_contract"]["all_cases_zero_allocations"])
        self.assertTrue(result["behavior_contract"]["default_off"])

    def test_qualification_rejects_allocation(self):
        broken = audit()
        broken["cases"][1]["measured_allocated_bytes"] = 8
        with self.assertRaises(ValueError):
            qualify([broken] * 5, 4)


if __name__ == "__main__":
    unittest.main()
