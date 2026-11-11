#main
import numpy as np
"""AE 352 quadcopter: 6-DOF EOM with four motor-torque inputs.

Frames: inertial z up; body x forward, y left, z up.
Rotor order: front (+x), left (+y), rear (-x), right (-y).
Positive rotor spin directions: [+1, -1, +1, -1].
No battery model, aerodynamic translational drag, or rotor gyroscopic coupling.
"""

from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class DroneParams:
    mass: float = 1.2             # kg
    gravity: float = 9.81         # m/s^2
    arm: float = 0.22             # m
    Ixx: float = 0.018            # kg m^2
    Iyy: float = 0.018            # kg m^2
    Izz: float = 0.032            # kg m^2
    rotor_inertia: float = 1.5e-5 # kg m^2
    k_thrust: float = 1.8e-5      # N / (rad/s)^2
    k_drag: float = 2.5e-7        # N m / (rad/s)^2


def rotation_body_to_world(roll, pitch, yaw):
    """ZYX Euler convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cphi, sphi = np.cos(roll), np.sin(roll)
    cth, sth = np.cos(pitch), np.sin(pitch)
    cps, sps = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cps * cth, cps * sth * sphi - sps * cphi, cps * sth * cphi + sps * sphi],
        [sps * cth, sps * sth * sphi + cps * cphi, sps * sth * cphi - cps * sphi],
        [-sth,      cth * sphi,                       cth * cphi],
    ])


def equations_of_motion(t, state, motor_torques, params):
    """Return derivatives of 16-state drone model.

    motor_torques(t, state) returns four nonnegative torque magnitudes (N m).
    Rotor spin direction signs are applied separately.
    """
    pos = state[0:3]
    vel = state[3:6]
    roll, pitch, yaw = state[6:9]
    body_rates = state[9:12]  # p, q, r (rad/s)
    omega = state[12:16]      # rotor speeds (rad/s), all nonnegative

    tau = np.asarray(motor_torques(t, state), dtype=float)
    if tau.shape != (4,) or np.any(tau < 0):
        raise ValueError('Motor inputs must be four nonnegative torque magnitudes')

    # Rotor spin directions: front, left, rear, right.
    spin = np.array([1.0, -1.0, 1.0, -1.0])

    # Rotor speed dynamics and generated thrust.
    rotor_accel = (tau - params.k_drag * omega**2) / params.rotor_inertia
    thrust = params.k_thrust * omega**2

    # Translation in inertial frame.
    R = rotation_body_to_world(roll, pitch, yaw)
    force_body = np.array([0.0, 0.0, np.sum(thrust)])
    acceleration = (R @ force_body) / params.mass - np.array([0.0, 0.0, params.gravity])

    # Roll and pitch from lever arms; yaw from motor reaction torque.
    moment_body = np.array([
        params.arm * (thrust[1] - thrust[3]),
        params.arm * (thrust[2] - thrust[0]),
        -np.dot(spin, tau),
    ])
    I = np.diag([params.Ixx, params.Iyy, params.Izz])
    rate_accel = np.linalg.solve(I, moment_body - np.cross(body_rates, I @ body_rates))

    # Euler-angle rates from body angular velocity (ZYX convention).
    p, q, r = body_rates
    if abs(np.cos(pitch)) < 1e-6:
        raise ValueError('Euler angle singularity near pitch = +/- 90 degrees')
    euler_rates = np.array([
        p + np.sin(roll) * np.tan(pitch) * q + np.cos(roll) * np.tan(pitch) * r,
        np.cos(roll) * q - np.sin(roll) * r,
        np.sin(roll) / np.cos(pitch) * q + np.cos(roll) / np.cos(pitch) * r,
    ])

    return np.concatenate([vel, acceleration, euler_rates, rate_accel, rotor_accel])


def main():
    params = DroneParams()
    hover_omega = np.sqrt(params.mass * params.gravity / (4.0 * params.k_thrust))
    hover_tau = params.k_drag * hover_omega**2
    print(f'Hover rotor speed: {hover_omega:.2f} rad/s ({hover_omega * 60 / (2 * np.pi):.0f} rpm)')
    print(f'Hover motor torque per rotor: {hover_tau:.4f} N m')

    # Start level, stationary, at z = 1 m with all four rotors at hover speed.
    initial = np.zeros(16)
    initial[2] = 1.0
    initial[12:16] = hover_omega

    def motor_torques(t, state):
        # Constant input; replace with a controller for circle/waypoint flight.
        return np.full(4, hover_tau)

    times = np.linspace(0, 120, 1201)
    sol = solve_ivp(
        lambda t, s: equations_of_motion(t, s, motor_torques, params),
        (0, 120), initial, t_eval=times, rtol=1e-9, atol=1e-11,
    )
    if not sol.success:
        raise RuntimeError(sol.message)

    print(f'Altitude after 120 s: {sol.y[2, -1]:.6f} m')
    print(f'Max altitude deviation: {np.max(np.abs(sol.y[2] - 1.0)):.3e} m')

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i, label in enumerate(('x', 'y', 'z')):
        axes[0].plot(sol.t, sol.y[i], label=label)
    axes[0].set_ylabel('Position (m)')
    axes[0].legend()
    axes[0].grid(True)
    for i, label in zip(range(6, 9), ('roll', 'pitch', 'yaw')):
        axes[1].plot(sol.t, np.rad2deg(sol.y[i]), label=label)
    axes[1].set_ylabel('Angle (deg)')
    axes[1].set_xlabel('Time (s)')
    axes[1].legend()
    axes[1].grid(True)