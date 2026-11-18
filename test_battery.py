"""Battery conservation, loading limits, and depleted-mission regressions."""
from dataclasses import replace
import unittest
import numpy as np
from battery import Battery, BatteryParams
from ae352 import DroneParams
from simulation import run_mission, summarize


class BatteryTests(unittest.TestCase):
    def test_pack_ratings_and_mass(self):
        p = BatteryParams()
        self.assertAlmostEqual(p.mass_kg, .48)
        self.assertAlmostEqual(p.nominal_energy_Wh, 92.88)
        self.assertAlmostEqual(Battery(p).open_circuit_voltage(), 25.2)

    def test_power_balance_and_charge_conservation(self):
        b = Battery()
        torque = np.full(4, .04)
        actual, log = b.supply(torque, np.full(4, 400.))
        np.testing.assert_allclose(actual, torque)
        self.assertAlmostEqual(log['power_W'], 64/(.85*.95)+9)
        self.assertAlmostEqual(log['voltage_V']*log['current_A'], log['power_W'])
        self.assertAlmostEqual(b.open_circuit_voltage()-log['voltage_V'], log['current_A']*b.params.resistance_ohm)
        b.advance(log, 10)
        self.assertAlmostEqual(b.energy_Wh, log['power_W']*10/3600)
        self.assertAlmostEqual(b.soc, 1-log['current_A']*10/(3600*4.3*.8))

    def test_current_limit_reduces_actual_torque(self):
        b = Battery(replace(BatteryParams(), cell_max_current_A=1))
        actual, log = b.supply(np.full(4, .12), np.full(4, 600.))
        self.assertTrue(log['power_limited'])
        self.assertLessEqual(log['current_A'], 1+1e-10)
        self.assertTrue(np.all(actual < .12))
        self.assertAlmostEqual(log['power_W'], log['shaft_power_W']/(.85*.95)+9)

    def test_resistanceless_pack_and_zero_speed_losses(self):
        b = Battery(replace(BatteryParams(), cell_resistance_ohm=0))
        _, log = b.supply(np.full(4, .12), np.zeros(4))
        self.assertEqual(log['power_W'], 9)
        self.assertAlmostEqual(log['current_A'], 9/25.2)

    def test_undervoltage_supply_disarms(self):
        b = Battery(replace(BatteryParams(), cell_resistance_ohm=100))
        torque, log = b.supply(np.full(4, .04), np.full(4, 400.))
        self.assertTrue(log['cutoff'])
        np.testing.assert_array_equal(torque, 0)

    def test_last_step_cannot_overdraw_charge(self):
        b = Battery(replace(BatteryParams(), cell_capacity_Ah=.0001))
        _, log = b.supply(np.full(4, .04), np.full(4, 400.), dt=.01)
        b.advance(log, .01)
        self.assertLessEqual(b.charge_Ah, b.params.capacity_Ah*b.params.discharge_efficiency+1e-12)

    def test_depleted_battery_disarms_and_fails_every_mission(self):
        for name in ['hover', 'circle', 'maneuver']:
            records, mission, touchdown = run_mission(name, initial_soc=0)
            summary = summarize(records, mission, touchdown, DroneParams())
            self.assertFalse(summary['passed'])
            self.assertTrue(summary['battery']['cutoff'])
            self.assertFalse(summary['checks']['ER3_no_battery_cutoff'])
            np.testing.assert_array_equal(records[0][4], 0)

    def test_tiny_capacity_runs_out_before_hover_completes(self):
        records, mission, touchdown = run_mission('hover', battery_params=replace(BatteryParams(), cell_capacity_Ah=.00001))
        summary = summarize(records, mission, touchdown, DroneParams())
        self.assertFalse(summary['passed'])
        self.assertLess(summary['metrics']['duration_s'], 120)

    def test_invalid_parameters_rejected(self):
        for kwargs in [{'motor_efficiency': 0}, {'parallel': 0}, {'reserve_soc': 1}, {'cell_resistance_ohm': -1}]:
            with self.assertRaises(ValueError): BatteryParams(**kwargs)
        with self.assertRaises(ValueError): Battery(soc=1.1)


if __name__ == '__main__':
    unittest.main()
