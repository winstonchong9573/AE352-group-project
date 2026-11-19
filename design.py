"""Reproducible AE352 D440 preliminary hardware design. See DRONE_DESIGN.md.

Manufacturer ratings, derived geometry, and engineering allowances are kept
separate. Importing this module does not change the original simulation defaults.
Run `python design.py` to regenerate the design artifacts.
"""
import csv
import hashlib
import json
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from ae352 import DroneParams
from battery import Battery, BatteryParams

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'design_data' / 'PER3_9x45MR.dat'
ARM = .220
PROP_DIAMETER = .2286
PROP_MASS = .39 * .028349523125  # APC: ounces to kg
MOTOR_KT = .0108                # KDE published N m/A
MOTOR_KE = 60 / (2*np.pi*885)   # SI conversion from 885 rpm/V
MOTOR_RESISTANCE = .127         # published phase-to-phase; equivalent-model approximation
MOTOR_IDLE_CURRENT = .5        # manufacturer test at 10 V; approximation elsewhere
MOTOR_ROTOR_INERTIA = .041e-4   # kg cm^2 -> kg m^2
SPEED_LIMIT_RPM = 8500.         # selected operational limit, below APC 11667 rpm
TORQUE_LIMIT = .15             # selected shaft torque cap, NOT a manufacturer rating
MASS_ALLOWANCE = .10

SOURCES = {
    'motor': 'https://www.kdedirect.com/products/kde2315xf-885',
    'esc': 'https://www.kdedirect.com/products/kdexf-uas20lv',
    'prop_ccw': 'https://www.apcprop.com/product/9x4-5mr/',
    'prop_cw': 'https://www.apcprop.com/product/9x4-5mrp/',
    'prop_data': 'https://www.apcprop.com/files/PER3_9x45MR.dat',
    'prop_method': 'https://www.apcprop.com/technical-information/performance-data/',
    'prop_rpm': 'https://www.apcprop.com/technical-information/rpm-limits/',
    'battery': 'https://www.gaoneng.shop/products/gaoneng-gnb-6s-22.2v-4500mah-10c-xt60-li-ion-battery-made-with-molicel-21700-p45b',
    'cell': 'https://www.molicel.com/wp-content/uploads/INR21700P45B_1.2_Product-Data-Sheet-of-INR-21700-P45B-80109.pdf',
    'tube': 'https://www.easycomposites.co.uk/20mm-woven-finish-carbon-fibre-tube',
    'plate': 'https://www.easycomposites.co.uk/high-strength-carbon-fibre-sheet',
    'fc': 'https://docs.holybro.com/autopilot/pixhawk-6c-mini/technical-specification',
    'power': 'https://holybro.com/products/micro-power-module-pm06-v2',
    'gps': 'https://holybro.com/products/m10-gps',
    'flow': 'https://docs.holybro.com/peripherals/h-flow-dronecan/overview',
    'receiver': 'https://radiomasterrc.com/products/rp1-expresslrs-2-4ghz-nano-receiver',
}


@dataclass
class Part:
    name: str
    mass_kg: float
    position_m: tuple
    own_inertia_kg_m2: tuple
    basis: str
    source: str


def box_inertia(mass, dimensions):
    x, y, z = dimensions
    return mass/12 * np.array([y*y+z*z, x*x+z*z, x*x+y*y])


def tube_inertia(mass, length, axis, ro=.010, ri=.009):
    radial = mass*(3*(ro*ro+ri*ri)+length*length)/12
    result = np.full(3, radial)
    result[axis] = mass*(ro*ro+ri*ri)/2
    return result


def components():
    """Positions use the arm-plane center as construction datum, z up.

    Central rectangular components use uniform boxes; propellers use a rotating
    uniform two-blade rod, averaged over azimuth for airframe inertia.
    """
    parts = []

    def box(name, mass, position, dimensions, basis, source=''):
        parts.append(Part(name, mass, tuple(position), tuple(box_inertia(mass, dimensions)), basis, source))

    for i, (dx, dy) in enumerate([(1, 0), (0, 1), (-1, 0), (0, -1)]):
        label = ['front', 'left', 'rear', 'right'][i]
        axis = 0 if dx else 1
        box(f'{label}: KDE2315XF-885 motor', .075, (ARM*dx, ARM*dy, .026),
            (.0283, .0283, .0285), 'published wired mass; uniform envelope inertia', SOURCES['motor'])
        prop_j = PROP_MASS*(PROP_DIAMETER/2)**2/3
        parts.append(Part(f'{label}: APC 9x4.5MR' + ('P' if i % 2 else ''), PROP_MASS,
                          (ARM*dx, ARM*dy, .046), (prop_j/2, prop_j/2, prop_j),
                          'published mass; uniform-blade approximation, azimuth averaged',
                          SOURCES['prop_cw' if i % 2 else 'prop_ccw']))
        box(f'{label}: prop adapter and retention allowance', .005, (ARM*dx, ARM*dy, .042),
            (.016, .016, .012), 'assumed additional hardware mass; verify assembled motor mass')
        box(f'{label}: KDEXF-UAS20LV ESC', .026, (.13*dx, .13*dy, -.018),
            (.032, .0225, .010) if dx else (.0225, .032, .010),
            'published wired mass and PCB planform; thickness assumed', SOURCES['esc'])
        tube_mass = .0865*.180
        parts.append(Part(f'{label}: 180 mm arm tube', tube_mass, (.13*dx, .13*dy, 0.),
                          tuple(tube_inertia(tube_mass, .180, axis)),
                          'published 86.5 g/m; 20/18 mm OD/ID; length chosen', SOURCES['tube']))
        box(f'{label}: 40 mm square motor plate', 2.80*.040*.040,
            (ARM*dx, ARM*dy, .01085), (.040, .040, .0017),
            'nominal 2 mm stock: published 2.80 kg/m2, typical 1.7 mm thickness', SOURCES['plate'])
        # Joint allowance is distributed between the root and motor-end clamps.
        for radius in [.05, .21]:
            box(f'{label}: clamp/joint at {radius:.2f} m', .005, (radius*dx, radius*dy, 0.),
                (.025, .025, .024), 'custom PA12 clamp, bolts and inserts: mass/envelope allowance')
        leg_mass = .0865*.160
        parts.append(Part(f'{label}: 160 mm landing leg', leg_mass, (.10*dx, .10*dy, -.090),
                          tuple(tube_inertia(leg_mass, .160, 2)),
                          'same tube stock; top z=-10 mm, foot z=-170 mm', SOURCES['tube']))
    for z, label in [(.01085, 'upper'), (-.01085, 'lower')]:
        box(f'{label} 120 mm square center plate', 2.80*.120*.120, (0., 0., z),
            (.120, .120, .0017), 'published areal mass; undrilled blank, conservative', SOURCES['plate'])
    box('GNB45006S10M battery', .475, (0., 0., -.0365), (.078, .064, .043),
        'published 460 +/-15 g: use upper mass; uniform-box inertia approximation', SOURCES['battery'])
    box('Pixhawk 6C Mini Model A current revision', .0424, (0., 0., .024), (.0543, .039, .0175),
        'published mass and envelope', SOURCES['fc'])
    box('Holybro PM06 V2 14S', .024, (0., 0., 0.), (.035, .035, .005),
        'published mass and envelope', SOURCES['power'])
    box('Holybro M10 GPS', .032, (0., 0., .080), (.050, .050, .0144),
        'published mass/diameter; square bounding-box inertia approximation', SOURCES['gps'])
    box('Holybro H-Flow', .0152, (0., 0., -.075), (.040, .040, .010),
        'published cased mass; assumed bounding envelope, verify mounting drawing', SOURCES['flow'])
    box('RadioMaster RP1 V2 with antenna', .0022, (0., 0., .045), (.013, .011, .003),
        'published mass incl antenna; lumped at center', SOURCES['receiver'])
    box('Harness, feet, leg joints, straps, mast and fasteners', .060, (0., 0., -.020), (.120, .120, .100),
        'engineering allowance; includes landing-leg joints; not a purchased part')
    return parts


def mass_properties(parts, allowance=MASS_ALLOWANCE):
    mass = sum(p.mass_kg for p in parts)
    cg = sum(p.mass_kg*np.asarray(p.position_m) for p in parts)/mass
    inertia = np.zeros((3, 3))
    for p in parts:
        r = np.asarray(p.position_m)-cg
        inertia += np.diag(p.own_inertia_kg_m2) + p.mass_kg*(np.dot(r, r)*np.eye(3)-np.outer(r, r))
    # Unallocated 10% mass distributed proportionally: preserves CG and scales I.
    return mass*(1+allowance), cg, inertia*(1+allowance)


def propeller_rows():
    rows, rpm = [], None
    for line in DATA.read_text().splitlines():
        if 'PROP RPM' in line:
            rpm = float(line.split('=')[1])
        try:
            values = [float(v) for v in line.split()]
        except ValueError:
            continue
        if rpm is not None and len(values) == 15 and values[0] == 0:
            # Prefer P/omega to the torque column rounded to 0.001 N m.
            omega = rpm*2*np.pi/60
            rows.append([rpm, omega, values[10], values[8]/omega, values[8]])
    if not rows:
        raise ValueError('No static rows found in APC data')
    return np.array(rows)


def propeller_fit():
    rows = propeller_rows()
    rows = rows[(rows[:, 0] >= 4000) & (rows[:, 0] <= 9000)]
    x = rows[:, 1]**2
    kt = float(x@rows[:, 2]/(x@x))
    kq = float(x@rows[:, 3]/(x@x))
    return kt, kq, rows


def selected_battery():
    return replace(BatteryParams(), packaging_mass_kg=.055, cell_max_current_A=30.,
                   cell_cutoff_V=3.0, motor_efficiency=.80, electronics_W=7., reserve_soc=.25)


def selected_params():
    mass, _, inertia = mass_properties(components())
    kt, kq, _ = propeller_fit()
    return DroneParams(mass=mass, arm=ARM, Ixx=inertia[0, 0], Iyy=inertia[1, 1], Izz=inertia[2, 2],
                       rotor_inertia=MOTOR_ROTOR_INERTIA+PROP_MASS*(PROP_DIAMETER/2)**2/3,
                       k_thrust=kt, k_drag=kq, max_motor_torque=TORQUE_LIMIT,
                       max_rotor_speed=SPEED_LIMIT_RPM*2*np.pi/60)


def design_report():
    p, bp = selected_params(), selected_battery()
    parts = components()
    mass, cg, inertia = mass_properties(parts)
    kt, kq, rows = propeller_fit()
    omega = np.sqrt(mass*p.gravity/(4*kt))
    tau = kq*omega**2
    b = Battery(bp, soc=bp.reserve_soc)
    _, hover_power = b.supply(np.full(4, tau), np.full(4, omega))
    _, power_limit = b.supply(np.full(4, 100.), np.full(4, omega))
    available_shaft = max(0., power_limit['power_W']-bp.electronics_W-4*bp.motor_idle_W_each)*bp.motor_efficiency*bp.esc_efficiency
    sustainable_speed = min(p.max_rotor_speed, np.sqrt(p.max_motor_torque/kq),
                            (available_shaft/(4*kq))**(1/3))
    motor_current_at_cap = p.max_motor_torque/MOTOR_KT+MOTOR_IDLE_CURRENT
    motor_voltage_at_cap = MOTOR_KE*p.max_rotor_speed+MOTOR_RESISTANCE*motor_current_at_cap
    tube_I = np.pi*(.020**4-.018**4)/64
    thrust_max = kt*p.max_rotor_speed**2
    cantilever_length = .170
    report = {
        'design': 'D440 / 9-inch / 6S', 'source_access_date': '2026-11-18',
        'status': 'preliminary engineering design; simulation-qualified only',
        'parameters': asdict(p), 'battery_parameters': asdict(bp),
        'nominal_mass_kg': sum(q.mass_kg for q in parts), 'mass_allowance_fraction': MASS_ALLOWANCE,
        'design_mass_kg': mass, 'cg_from_arm_plane_m': cg.tolist(), 'inertia_tensor_kg_m2': inertia.tolist(),
        'propeller_diameter_m': PROP_DIAMETER, 'propeller_pitch_m': .1143,
        'adjacent_prop_tip_clearance_m': np.sqrt(2)*ARM-PROP_DIAMETER,
        'overall_prop_envelope_m': 2*ARM+PROP_DIAMETER,
        'hover_rpm': float(omega*60/(2*np.pi)), 'hover_thrust_per_rotor_N': mass*p.gravity/4,
        'hover_torque_per_motor_Nm': float(tau), 'hover_shaft_power_W': float(4*tau*omega),
        'hover_electrical_power_W': hover_power['power_W'],
        'hover_current_at_reserve_soc_A': hover_power['current_A'],
        'hover_voltage_at_reserve_soc_V': hover_power['voltage_V'],
        'max_thrust_per_rotor_N': thrust_max,
        'thrust_weight_ratio_at_speed_cap': 4*thrust_max/(mass*p.gravity),
        'sustainable_collective_rpm_at_reserve': sustainable_speed*60/(2*np.pi),
        'thrust_weight_ratio_at_reserve': 4*kt*sustainable_speed**2/(mass*p.gravity),
        'motor_current_at_torque_cap_A_estimate': motor_current_at_cap,
        'motor_voltage_at_simultaneous_caps_V_estimate': motor_voltage_at_cap,
        'tip_mach_at_cap_assuming_343m_s': p.max_rotor_speed*PROP_DIAMETER/2/343,
        'arm_tip_deflection_at_max_thrust_mm': thrust_max*cantilever_length**3/(3*64e9*tube_I)*1000,
        'arm_bending_stress_at_max_thrust_MPa': thrust_max*cantilever_length*.010/tube_I/1e6,
        'prop_fit_max_thrust_relative_error': float(np.max(abs(kt*rows[:, 1]**2/rows[:, 2]-1))),
        'prop_fit_max_torque_relative_error': float(np.max(abs(kq*rows[:, 1]**2/rows[:, 3]-1))),
        'apc_data_sha256': hashlib.sha256(DATA.read_bytes()).hexdigest(),
        'sources': SOURCES,
    }
    report['checks'] = {
        'assignment_mass_0p1_to_10kg': .1 <= mass <= 10,
        'clearance_at_least_0p1R': report['adjacent_prop_tip_clearance_m'] >= .1*PROP_DIAMETER/2,
        'inertia_positive_definite': bool(np.all(np.linalg.eigvalsh(inertia) > 0)),
        'inertia_axes_match_model': bool(np.allclose(inertia, np.diag(np.diag(inertia)), atol=1e-10)),
        'hover_within_speed_and_torque_caps': bool(omega < p.max_rotor_speed and tau < p.max_motor_torque),
        'hover_supported_at_reserve': not hover_power['power_limited'] and not hover_power['cutoff'],
        'two_to_one_collective_at_reserve': report['thrust_weight_ratio_at_reserve'] >= 2,
        'propeller_below_manufacturer_rpm_limit': SPEED_LIMIT_RPM < 105000/9,
        'motor_6S_prop_size_rule': PROP_DIAMETER <= 9*.0254+1e-12,
        'pack_max_voltage_below_motor_and_esc_rating': bp.series*bp.cell_full_V <= 26.1,
        'motor_current_estimate_below_derated_15A': motor_current_at_cap <= 15,
        'motor_voltage_estimate_below_cutoff_voltage': motor_voltage_at_cap < bp.series*bp.cell_cutoff_V,
        'power_module_harness_current_ceiling': bp.cell_max_current_A*bp.parallel <= 30,
        'thrust_fit_within_2percent': report['prop_fit_max_thrust_relative_error'] <= .02,
        'torque_fit_within_10percent': report['prop_fit_max_torque_relative_error'] <= .10,
    }
    report['checks'] = {name: bool(value) for name, value in report['checks'].items()}
    report['all_design_checks_pass'] = all(report['checks'].values())
    return report


def write_artifacts(folder=ROOT/'design_output'):
    folder.mkdir(parents=True, exist_ok=True)
    report = design_report()
    (folder/'design_parameters.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    with (folder/'mass_budget.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['part', 'mass_g', 'x_mm', 'y_mm', 'z_mm', 'basis', 'source'])
        for part in components():
            w.writerow([part.name, part.mass_kg*1000, *[v*1000 for v in part.position_m], part.basis, part.source])
        w.writerow(['Distributed 10% design allowance', report['nominal_mass_kg']*MASS_ALLOWANCE*1000,
                    *[v*1000 for v in report['cg_from_arm_plane_m']], 'not an additional battery or purchased part', ''])
    grouped = {}
    for part in components():
        name = part.name.split(': ', 1)[-1]
        if name not in grouped:
            grouped[name] = [0, part]
        grouped[name][0] += 1
    with (folder/'bill_of_materials.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['component', 'installed_quantity', 'unit_mass_g', 'total_mass_g', 'basis_or_purchase_note', 'source'])
        for name, (quantity, part) in grouped.items():
            w.writerow([name, quantity, part.mass_kg*1000, part.mass_kg*quantity*1000, part.basis, part.source])
        w.writerow(['Carbon tube stock purchase', '2 x 1 m', '', '',
                    'Cut 4 x 180 mm arms and 4 x 160 mm legs; installed masses counted above', SOURCES['tube']])
        w.writerow(['Carbon sheet stock purchase', '1 x 250 x 230 mm nominal 2 mm sheet', '', '',
                    'Cut 2 x 120 mm squares and 4 x 40 mm squares; installed masses counted above', SOURCES['plate']])
        w.writerow(['APC LPAR06E centering rings', '4 suitable rings', '', '',
                    'Verify fit on supplied KDE M6 adapters; mass included in retention allowance', SOURCES['prop_ccw']])
    with (folder/'propeller_static_fit.csv').open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['rpm', 'omega_rad_s', 'APC_thrust_N', 'APC_torque_from_power_Nm', 'APC_shaft_power_W'])
        w.writerows(propeller_fit()[2])
    from design_figures import draw_design
    draw_design(folder, report)
    return report


if __name__ == '__main__':
    report = write_artifacts()
    print(json.dumps({k: v for k, v in report.items() if k not in ['sources', 'battery_parameters']}, indent=2))
    if not report['all_design_checks_pass']:
        raise SystemExit(1)
