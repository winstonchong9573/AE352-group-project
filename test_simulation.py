"""Mechanical regression checks: python -m unittest -v."""
import unittest
import numpy as np
from ae352 import DroneParams, equations_of_motion, rotation_body_to_world
from simulation import Controller, hold, rk4


class DynamicsTests(unittest.TestCase):
    def setUp(self):
        self.params = DroneParams()
        self.state = np.zeros(16)
        self.state[2] = 1
        self.speed = np.sqrt(self.params.mass*self.params.gravity/(4*self.params.k_thrust))
        self.torque = self.params.k_drag*self.speed**2

    def derivative(self, torque):
        return equations_of_motion(0, self.state, lambda t, s: torque, self.params)

    def test_free_fall(self):
        d = self.derivative(np.zeros(4))
        np.testing.assert_allclose(d[3:6], [0, 0, -self.params.gravity])

    def test_hover_equilibrium(self):
        self.state[12:16] = self.speed
        np.testing.assert_allclose(self.derivative(np.full(4, self.torque)), 0, atol=1e-10)

    def test_rotation_preserves_lengths(self):
        R = rotation_body_to_world(.4, -.3, 1.2)
        np.testing.assert_allclose(R.T@R, np.eye(3), atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(R), 1.)

    def test_roll_pitch_and_yaw_signs(self):
        self.state[13] = self.speed  # left rotor
        self.assertGreater(self.derivative(np.zeros(4))[9], 0)
        self.state[13] = 0
        self.state[12] = self.speed  # front rotor
        self.assertLess(self.derivative(np.zeros(4))[10], 0)
        self.assertLess(self.derivative(np.array([.01, 0, 0, 0]))[11], 0)

    def test_collective_torque_spools_up(self):
        self.state[12:16] = self.speed
        d = self.derivative(np.full(4, self.torque*1.2))
        self.assertTrue(np.all(d[12:] > 0))
        next_state = rk4(self.state, np.full(4, self.torque*1.2), .01, self.params)
        self.assertGreater(next_state[5], 0)

    def test_controller_recovers_perturbation(self):
        self.state[2] = .8
        self.state[6:8] = [.05, -.04]
        self.state[12:16] = self.speed
        controller = Controller(self.params)
        for _ in range(1000):
            torque, _ = controller.command(self.state, hold([0, 0, 1]))
            self.state = rk4(self.state, torque, .01, self.params)
        self.assertLess(np.linalg.norm(self.state[:3]-[0, 0, 1]), .01)
        self.assertLess(np.linalg.norm(self.state[6:9]), .01)

    def test_invalid_inputs_rejected(self):
        for torque in [np.zeros(3), np.array([-1, 0, 0, 0]), np.full(4, np.nan)]:
            with self.assertRaises(ValueError): self.derivative(torque)

    def test_ground_support_and_liftoff(self):
        self.state[2] = 0
        supported = rk4(self.state, np.zeros(4), .01, self.params)
        self.assertEqual(supported[2], 0)
        self.assertEqual(supported[5], 0)
        self.state[12:16] = self.speed*1.1
        airborne = rk4(self.state, np.full(4, self.torque*1.21), .01, self.params)
        self.assertGreater(airborne[2], 0)


if __name__ == '__main__':
    unittest.main()
