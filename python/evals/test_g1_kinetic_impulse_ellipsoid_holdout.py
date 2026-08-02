from __future__ import annotations

import unittest

from g1_kinetic_impulse_ellipsoid_holdout import (
    CONTROL_DT,
    GRAVITY,
    KINETIC_RESERVE_FRACTION,
    analytic_twice_energy,
)


class G1KineticImpulseEllipsoidHoldoutTests(unittest.TestCase):
    def test_analytic_energy_has_declared_translation_speed_scale(self) -> None:
        mass = 37.0
        energy2 = analytic_twice_energy(mass)
        speed = (energy2 / mass) ** 0.5
        self.assertAlmostEqual(
            speed, KINETIC_RESERVE_FRACTION * GRAVITY * CONTROL_DT
        )

    def test_analytic_energy_rejects_bad_mass(self) -> None:
        with self.assertRaises(ValueError):
            analytic_twice_energy(0.0)


if __name__ == "__main__":
    unittest.main()
