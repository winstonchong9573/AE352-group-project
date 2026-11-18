"""Torque-controlled missions. SI units"""
import argparse
import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from ae352 import DroneParams, equations_of_motion, rotation_body_to_world
from battery import Battery, BatteryParams

SPIN = np.array([1., -1., 1., -1.])
STATE_NAMES = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'roll', 'pitch', 'yaw',
               'p', 'q', 'r', 'omega_front', 'omega_left', 'omega_rear', 'omega_right']


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Reference:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    yaw: float = 0.


def hold(position, yaw=0.):
    return Reference(np.array(position, dtype=float), np.zeros(3), np.zeros(3), yaw)


def smooth_move(start, end, elapsed, duration, yaw=0.):
    """Quintic path with zero endpoint velocity and acceleration."""
    u = np.clip(elapsed / duration, 0., 1.)
    displacement = np.asarray(end) - np.asarray(start)
    h = 10*u**3 - 15*u**4 + 6*u**5
    hd = (30*u**2 - 60*u**3 + 30*u**4) / duration
    hdd = (60*u - 180*u**2 + 120*u**3) / duration**2
    return Reference(np.asarray(start) + h*displacement, hd*displacement,
                     hdd*displacement, yaw)


class Controller:
    def __init__(self, params):
        self.params = params
        p = params
        c = p.k_drag / p.k_thrust
        # [total thrust, roll moment, pitch moment, steady yaw moment] = B @ thrusts
        self.mixer = np.array([[1, 1, 1, 1], [0, p.arm, 0, -p.arm],
                               [-p.arm, 0, p.arm, 0], [-c, c, -c, c]])
        self.inverse_mixer = np.linalg.inv(self.mixer)

    def command(self, state, ref):
        p = self.params
        accel = ref.acceleration + 4*(ref.position-state[:3]) + 4*(ref.velocity-state[3:6])
        accel[:2] = np.clip(accel[:2], -3., 3.)
        accel[2] = np.clip(accel[2], -3., 4.)
        force = p.mass * (accel + np.array([0., 0., p.gravity]))
        # Exact thrust direction for desired yaw, rather than a small-angle conversion.
        cy, sy = np.cos(ref.yaw), np.sin(ref.yaw)
        forward = cy*force[0] + sy*force[1]
        left = -sy*force[0] + cy*force[1]
        pitch = np.arctan2(forward, force[2])
        roll = np.arctan2(-left, np.hypot(forward, force[2]))
        desired = np.array([roll, pitch, ref.yaw])
        desired[:2] = np.clip(desired[:2], -np.deg2rad(25), np.deg2rad(25))
        error = wrap(desired-state[6:9])
        angular_accel = np.array([36., 36., 16.])*error - np.array([12., 12., 8.])*state[9:12]
        inertia = np.array([p.Ixx, p.Iyy, p.Izz])
        moment = inertia*angular_accel + np.cross(state[9:12], inertia*state[9:12])
        R = rotation_body_to_world(*state[6:9])
        total = max(0., force @ R[:, 2])
        thrusts = self.inverse_mixer @ np.r_[total, moment]
        target_omega = np.sqrt(np.clip(thrusts, 0., p.k_thrust*p.max_rotor_speed**2)/p.k_thrust)
        torque = p.k_drag*state[12:16]**2 + p.rotor_inertia*25*(target_omega-state[12:16])
        return np.clip(torque, 0., p.max_motor_torque), desired


class Mission:
    """Full mission phases advance after measured settling, with explicit timeouts."""
    def __init__(self, name):
        self.name = name
        self.phase = 'takeoff' if name == 'maneuver' else name
        self.start = 0.
        self.settled_since = None
        self.events = []
        self.done = False

    def reference(self, t):
        e = t-self.start
        if self.name == 'hover':
            return hold([0, 0, 1])
        if self.name == 'circle':
            a = .25*t
            return Reference(np.array([2*np.cos(a), 2*np.sin(a), 1.]),
                             np.array([-.5*np.sin(a), .5*np.cos(a), 0.]),
                             np.array([-.125*np.cos(a), -.125*np.sin(a), 0.]))
        if self.phase == 'takeoff':
            return smooth_move([0, 0, 0], [0, 0, 1], e, 4.)
        if self.phase == 'leg1':
            return smooth_move([0, 0, 1], [5, 0, 1], e, 5.)
        if self.phase == 'yaw':
            angle = smooth_move([0], [np.pi/2], e, 4.).position[0]
            return hold([5, 0, 1], angle)
        if self.phase == 'leg2':
            return smooth_move([5, 0, 1], [5, 5, 1], e, 5., np.pi/2)
        # Slow descent; continue reference below ground so the controller does not
        # asymptotically hover above it. Ground contact ends the mission.
        return Reference(np.array([5., 5., 1.-.008*e]), np.array([0., 0., -.008]),
                         np.zeros(3), np.pi/2)

    def update(self, t, state):
        if self.name != 'maneuver' or self.phase == 'land':
            return
        elapsed = t-self.start
        duration = {'takeoff': 4., 'leg1': 5., 'yaw': 4., 'leg2': 5.}[self.phase]
        target = self.reference(self.start+duration)
        settled = (elapsed >= duration and np.linalg.norm(state[:3]-target.position) < .05
                   and np.linalg.norm(state[3:6]) < .05
                   and abs(wrap(state[8]-target.yaw)) < np.deg2rad(2)
                   and np.linalg.norm(state[9:12]) < .05)
        if not settled:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = t
        if self.settled_since is not None and t-self.settled_since >= .5:
            self.events.append({'phase': self.phase, 'start_s': self.start, 'end_s': t,
                                'end_position_m': state[:3].tolist(), 'end_yaw_rad': float(state[8]),
                                'end_speed_m_s': float(np.linalg.norm(state[3:6]))})
            self.phase = {'takeoff': 'leg1', 'leg1': 'yaw', 'yaw': 'leg2', 'leg2': 'land'}[self.phase]
            self.start = t
            self.settled_since = None
        elif elapsed > duration+20:
            raise RuntimeError(f'{self.phase} failed to settle within timeout')


def rk4(state, torque, dt, params):
    def f(s):
        derivative = equations_of_motion(0., s, lambda t, x: torque, params)
        # Flat ground supports the drone until upward force produces liftoff.
        if s[2] <= 0 and s[5] <= 0 and derivative[5] < 0:
            derivative[2] = derivative[5] = 0.
        return derivative
    k1 = f(state)
    k2 = f(state+dt*k1/2)
    k3 = f(state+dt*k2/2)
    k4 = f(state+dt*k3)
    return state + dt*(k1+2*k2+2*k3+k4)/6


def run_mission(name, params=None, dt=.01, battery_params=None, initial_soc=1.):
    if name not in {'hover', 'circle', 'maneuver'}:
        raise ValueError('Unknown mission')
    if not 0 < dt <= .01:
        raise ValueError('Use a positive step no larger than 0.01 s')
    params = params or DroneParams()
    controller, mission = Controller(params), Mission(name)
    battery = Battery(battery_params, initial_soc)
    if battery.params.mass_kg >= params.mass:
        raise ValueError('Total drone mass must include battery and positive airframe mass')
    mission.battery = battery
    mission.battery_records = []
    mission.initial_soc = initial_soc
    state = np.zeros(16)
    if name != 'maneuver':
        state[2] = 1.
        state[12:16] = np.sqrt(params.mass*params.gravity/(4*params.k_thrust))
    if name == 'circle':
        state[0], state[4] = 2., .5
    duration = {'hover': 120., 'circle': 70., 'maneuver': 190.}[name]
    records = []
    touchdown = None
    for step in range(round(duration/dt)+1):
        t = step*dt
        mission.update(t, state)
        ref = mission.reference(t)
        torque, angles = controller.command(state, ref)
        torque, power = battery.supply(torque, state[12:16], dt)
        mission.battery_records.append({'time_s': t, **power})
        records.append((t, state.copy(), ref.position.copy(), ref.velocity.copy(),
                        torque.copy(), angles.copy(), mission.phase))
        if mission.done or battery.cutoff or step == round(duration/dt):
            break
        next_state = rk4(state, torque, dt, params)
        battery.advance(power, dt)
        if not np.all(np.isfinite(next_state)):
            raise RuntimeError(f'Nonfinite state at {t}')
        if next_state[2] < 0:
            if state[2] > 0 and mission.phase == 'land':
                fraction = state[2]/(state[2]-next_state[2])
                impact = state+fraction*(next_state-state)
                touchdown = {'time_s': t+fraction*dt, 'vertical_speed_m_s': float(abs(impact[5])),
                             'position_m': impact[:3].tolist()}
                mission.done = True
            next_state[2] = 0.
            next_state[5] = max(0., next_state[5])
        next_state[12:16] = np.clip(next_state[12:16], 0, params.max_rotor_speed)
        state = next_state
    return records, mission, touchdown


def summarize(records, mission, touchdown, params):
    t = np.array([r[0] for r in records])
    s = np.array([r[1] for r in records])
    torque = np.array([r[4] for r in records])
    checks = {}
    metrics = {'duration_s': float(t[-1]), 'max_torque_Nm': float(torque.max()),
               'max_rotor_speed_rad_s': float(s[:, 12:16].max())}
    if mission.name == 'hover':
        metrics['max_position_error_m'] = float(np.max(np.linalg.norm(s[:, :3]-[0, 0, 1], axis=1)))
        checks = {'120_seconds': bool(t[-1] >= 120-1e-8),
                  'position_within_5cm': metrics['max_position_error_m'] <= .05}
    elif mission.name == 'circle':
        mask = t >= 10
        metrics.update(max_radius_error_m=float(np.max(abs(np.linalg.norm(s[mask, :2], axis=1)-2))) if mask.any() else None,
                       max_altitude_error_m=float(np.max(abs(s[mask, 2]-1))) if mask.any() else None,
                       max_speed_error_m_s=float(np.max(abs(np.linalg.norm(s[mask, 3:6], axis=1)-.5))) if mask.any() else None,
                       evaluated_duration_s=max(0., float(t[-1]-10)))
        checks = {'60_seconds': metrics['evaluated_duration_s'] >= 60-1e-8,
                  'radius_within_10cm': mask.any() and metrics['max_radius_error_m'] <= .1,
                  'altitude_within_5cm': mask.any() and metrics['max_altitude_error_m'] <= .05,
                  'speed_within_005m_s': mask.any() and metrics['max_speed_error_m_s'] <= .05}
    else:
        checks['phases_completed'] = len(mission.events) == 4 and touchdown is not None
        for phase, start, end in [('leg1', [0, 0, 1], [5, 0, 1]), ('leg2', [5, 0, 1], [5, 5, 1])]:
            checks[phase] = False
            mask = np.array([r[6] == phase for r in records])
            if mask.sum() > 1:
                # Five-second commanded translation window, excluding the explicitly
                # reported settling interval. Also report average including settling.
                times = t[mask]; points = s[mask]
                moving = times <= times[0]+5+1e-8
                distance = float(np.linalg.norm(points[moving][-1, :2]-points[0, :2]))
                metrics[phase] = {'distance_in_5s_m': distance, 'average_speed_in_5s_m_s': distance/5,
                                  'average_speed_including_settle_m_s': float(np.linalg.norm(points[-1, :2]-points[0, :2])/(times[-1]-times[0])),
                                  'endpoint_error_m': float(np.linalg.norm(points[-1, :3]-end)),
                                  'stop_speed_m_s': float(np.linalg.norm(points[-1, 3:6])),
                                  'max_altitude_error_m': float(np.max(abs(points[:, 2]-1)))}
                checks[phase] = (abs(distance-5) <= .25 and metrics[phase]['endpoint_error_m'] <= .05
                                 and metrics[phase]['stop_speed_m_s'] <= .05
                                 and metrics[phase]['max_altitude_error_m'] <= .05)
        yaw_event = next((e for e in mission.events if e['phase'] == 'yaw'), None)
        checks['left_yaw_90deg'] = yaw_event is not None and abs(wrap(yaw_event['end_yaw_rad']-np.pi/2)) <= np.deg2rad(2)
        checks['touchdown_at_most_001m_s'] = touchdown is not None and touchdown['vertical_speed_m_s'] <= .01
        metrics['touchdown'] = touchdown
    checks['torque_limit'] = bool(torque.max() <= params.max_motor_torque+1e-10)
    checks['rotor_limit'] = bool(s[:, 12:16].max() <= params.max_rotor_speed+1e-8)
    checks['ground_not_penetrated'] = bool(s[:, 2].min() >= 0)
    logs = mission.battery_records
    bp = mission.battery.params
    power_metrics = {'initial_soc': mission.initial_soc, 'final_soc': mission.battery.soc,
                     'energy_used_Wh': mission.battery.energy_Wh,
                     'charge_used_Ah': mission.battery.charge_Ah,
                     'nominal_pack_energy_Wh': bp.nominal_energy_Wh,
                     'minimum_voltage_V': min(r['voltage_V'] for r in logs),
                     'peak_current_A': max(r['current_A'] for r in logs),
                     'peak_power_W': max(r['power_W'] for r in logs),
                     'battery_mass_kg': bp.mass_kg, 'airframe_and_propulsion_mass_kg': params.mass-bp.mass_kg,
                     'cutoff': mission.battery.cutoff,
                     'power_limited_samples': sum(r['power_limited'] for r in logs)}
    checks['ER3_no_battery_cutoff'] = not mission.battery.cutoff
    checks['battery_reserve_at_least_20percent'] = mission.battery.soc >= bp.reserve_soc
    checks['battery_voltage_limit'] = power_metrics['minimum_voltage_V'] >= bp.series*bp.cell_cutoff_V-1e-9
    checks['battery_current_limit'] = power_metrics['peak_current_A'] <= bp.parallel*bp.cell_max_current_A+1e-9
    checks = {key: bool(value) for key, value in checks.items()}
    return {'mission': mission.name, 'passed': bool(all(checks.values())), 'checks': checks,
            'metrics': metrics, 'battery': power_metrics, 'battery_parameters': asdict(bp),
            'phase_events': mission.events, 'parameters': asdict(params),
            'limitations': ['ER3 passes only under the documented battery and propulsion assumptions; no thermal model.',
                            'Motor/rotor, airframe parameters and actuator limits remain provisional; ER4 is not fully verified.',
                            'OCV curve, DC resistance and motor/ESC losses are assumed, not measured.',
                            'Tracking tolerances are project choices, not instructor-specified.']}


def save_results(records, summary, folder, battery_records=None):
    folder.mkdir(parents=True, exist_ok=True)
    with (folder/'trajectory.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['time_s']+STATE_NAMES+['ref_x', 'ref_y', 'ref_z', 'ref_vx', 'ref_vy', 'ref_vz']+
                        ['torque_front', 'torque_left', 'torque_rear', 'torque_right']+
                        ['cmd_roll', 'cmd_pitch', 'cmd_yaw', 'phase'])
        for t, s, pos, vel, torque, angles, phase in records:
            writer.writerow([t, *s, *pos, *vel, *torque, *angles, phase])
    (folder/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
    if battery_records:
        with (folder/'battery.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(battery_records[0]))
            writer.writeheader(); writer.writerows(battery_records)
        bt = np.array([r['time_s'] for r in battery_records])
        fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        for ax, key, label in zip(axes.flat, ['soc', 'voltage_V', 'current_A', 'energy_used_Wh'],
                                 ['State of charge (%)', 'Terminal voltage (V)', 'Battery current (A)', 'Energy used (Wh)']):
            values = np.array([r[key] for r in battery_records])
            ax.plot(bt, values*100 if key == 'soc' else values)
            ax.set_ylabel(label); ax.set_xlabel('Time (s)'); ax.grid(alpha=.25)
        axes[0, 0].axhline(100*summary['battery_parameters']['reserve_soc'], color='r', ls='--', label='Reserve')
        axes[0, 0].legend()
        fig.suptitle(f"{summary['mission'].title()} - battery and energy")
        fig.tight_layout(); fig.savefig(folder/'battery.png', dpi=160); plt.close(fig)
    t = np.array([r[0] for r in records]); s = np.array([r[1] for r in records])
    ref = np.array([r[2] for r in records]); angles = np.array([r[5] for r in records])
    torques = np.array([r[4] for r in records])
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)
    for i, label in enumerate(['x', 'y', 'z']):
        axes[i, 0].plot(t, s[:, i], label='Actual')
        axes[i, 0].plot(t, ref[:, i], '--', label='Command')
        axes[i, 0].set_ylabel(f'{label} (m)')
        axes[i, 1].plot(t, np.rad2deg(s[:, 6+i]), label='Actual')
        axes[i, 1].plot(t, np.rad2deg(angles[:, i]), '--', label='Command')
        axes[i, 1].set_ylabel(['Roll (deg)', 'Pitch (deg)', 'Yaw (deg)'][i])
    for ax in axes.flat:
        ax.grid(alpha=.25); ax.legend()
    for ax in axes[-1]: ax.set_xlabel('Time (s)')
    fig.suptitle(f"{summary['mission'].title()} - position and attitude")
    fig.tight_layout(); fig.savefig(folder/'states.png', dpi=160); plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    for i, label in enumerate(['Front', 'Left', 'Rear', 'Right']):
        axes[0].plot(t, s[:, 12+i], label=label)
        axes[1].plot(t, torques[:, i], label=label)
    axes[0].set_ylabel('Rotor speed (rad/s)'); axes[1].set_ylabel('Motor torque (N m)')
    axes[1].set_xlabel('Time (s)')
    for ax in axes: ax.grid(alpha=.25); ax.legend(ncol=4)
    fig.tight_layout(); fig.savefig(folder/'motors.png', dpi=160); plt.close(fig)
    fig = plt.figure(figsize=(8, 6)); ax = fig.add_subplot(projection='3d')
    ax.plot(*s[:, :3].T, label='Actual'); ax.plot(*ref.T, '--', label='Command')
    ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)', title=summary['mission'].title())
    # Show physical height rather than magnifying micrometer-scale hover errors.
    ax.set_zlim(0, max(1.2, float(s[:, 2].max())+.1))
    ax.set_box_aspect((max(float(np.ptp(s[:, 0])), 1),
                       max(float(np.ptp(s[:, 1])), 1), 1.2))
    ax.legend(); fig.tight_layout(); fig.savefig(folder/'path.png', dpi=160); plt.close(fig)


def cli():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', choices=['all', 'hover', 'circle', 'maneuver'], default='all')
    parser.add_argument('--output', type=Path, default=Path('results'))
    parser.add_argument('--dt', type=float, default=.01)
    parser.add_argument('--initial-soc', type=float, default=1.)
    parser.add_argument('--fresh-battery-each-mission', action='store_true',
                        help='Default all-mission run carries SOC across missions; this resets it instead')
    args = parser.parse_args()
    if not 0 < args.dt <= .01:
        parser.error('--dt must be positive and at most 0.01 seconds')
    params = DroneParams()
    summaries = []
    soc = args.initial_soc
    if not np.isfinite(soc) or not 0 <= soc <= 1:
        parser.error('--initial-soc must be between 0 and 1')
    for name in (['hover', 'circle', 'maneuver'] if args.mission == 'all' else [args.mission]):
        records, mission, touchdown = run_mission(name, params, args.dt, initial_soc=soc)
        summary = summarize(records, mission, touchdown, params)
        save_results(records, summary, args.output/name, mission.battery_records)
        if not args.fresh_battery_each_mission:
            soc = mission.battery.soc
        summaries.append(summary)
        print(f"{name}: {'PASS' if summary['passed'] else 'FAIL'}", flush=True)
        print(json.dumps(summary['metrics'], indent=2), flush=True)
        print(json.dumps(summary['battery'], indent=2), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    if not all(s['passed'] for s in summaries):
        raise SystemExit(1)


if __name__ == '__main__':
    cli()
