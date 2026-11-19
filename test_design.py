"""Independent physical and data-conversion checks for the selected design."""
import unittest
from dataclasses import replace

import numpy as np

from ae352 import equations_of_motion
from design import (components, mass_properties, selected_battery, selected_params,
                    propeller_rows, propeller_fit, design_report, MOTOR_ROTOR_INERTIA)


class DesignTests(unittest.TestCase):
    def test_parallel_axis_assembly_is_translation_invariant(self):
        parts = components()
        shift = np.array([2., -3., 4.])
        translated = [replace(p, position_m=tuple(np.asarray(p.position_m)+shift)) for p in parts]
        mass, cg, inertia = mass_properties(parts)
        shifted_mass, shifted_cg, shifted_inertia = mass_properties(translated)
        self.assertAlmostEqual(mass, shifted_mass)
        np.testing.assert_allclose(shifted_cg, cg+shift, atol=1e-12)
        np.testing.assert_allclose(shifted_inertia, inertia, atol=1e-12)
        self.assertTrue(np.all(np.linalg.eigvalsh(inertia) > 0))

    def test_battery_mass_counted_once_before_allowance(self):
        parts = components()
        packs = [p for p in parts if 'battery' in p.name]
        self.assertEqual(len(packs), 1)
        self.assertAlmostEqual(packs[0].mass_kg, selected_battery().mass_kg)
        self.assertAlmostEqual(selected_params().mass, sum(p.mass_kg for p in parts)*1.1)

    def test_manufacturer_units_and_static_operating_point(self):
        self.assertAlmostEqual(MOTOR_ROTOR_INERTIA, 4.1e-6)
        rows = propeller_rows()
        row = rows[rows[:, 0] == 6000][0]
        # Independently transcribed SI columns from APC's 6000 RPM static row.
        self.assertAlmostEqual(row[2], 3.859)
        self.assertAlmostEqual(row[4], 34.224)
        self.assertAlmostEqual(row[3]*row[1], 34.224)
        kt, kq, fit_rows = propeller_fit()
        self.assertLess(np.max(abs(kt*fit_rows[:, 1]**2/fit_rows[:, 2]-1)), .02)
        self.assertLess(np.max(abs(kq*fit_rows[:, 1]**3/fit_rows[:, 4]-1)), .10)

    def test_selected_hover_is_equilibrium(self):
        p = selected_params()
        state = np.zeros(16)
        state[2] = 1
        state[12:] = np.sqrt(p.mass*p.gravity/(4*p.k_thrust))
        torque = p.k_drag*state[12:]**2
        d = equations_of_motion(0, state, lambda t, s: torque, p)
        np.testing.assert_allclose(d, 0, atol=1e-10)

    def test_component_compatibility_and_physical_margins(self):
        report = design_report()
        self.assertTrue(report['all_design_checks_pass'], report['checks'])
        self.assertGreater(report['thrust_weight_ratio_at_reserve'], 2.)
        self.assertGreater(report['hover_voltage_at_reserve_soc_V'], 18.)


if __name__ == '__main__':
    unittest.main()
