"""Run the researched D440 configuration without overwriting baseline results."""
import argparse
import json
from pathlib import Path

import numpy as np

from design import (selected_params, selected_battery, write_artifacts,
                    MOTOR_KT, MOTOR_KE, MOTOR_RESISTANCE, MOTOR_IDLE_CURRENT)
from design_figures import mission_figures
from simulation import run_mission, summarize, save_results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dt', type=float, default=.01)
    parser.add_argument('--output', type=Path, default=Path('results/design'))
    parser.add_argument('--initial-soc', type=float, default=1.)
    args = parser.parse_args()
    design = write_artifacts()
    if not design['all_design_checks_pass']:
        raise RuntimeError('Physical design checks failed; inspect design_output/design_parameters.json')
    p, bp = selected_params(), selected_battery()
    soc = args.initial_soc
    summaries = []
    for name in ['hover', 'circle', 'maneuver']:
        records, mission, touchdown = run_mission(name, p, args.dt, bp, soc)
        summary = summarize(records, mission, touchdown, p)
        state = np.array([r[1] for r in records])
        torque = np.array([r[4] for r in records])
        current_est = torque/MOTOR_KT+MOTOR_IDLE_CURRENT
        voltage_est = MOTOR_KE*state[:, 12:16]+MOTOR_RESISTANCE*current_est
        pack_voltage = np.array([r['voltage_V'] for r in mission.battery_records])
        summary['design'] = design['design']
        summary['checks']['battery_reserve_at_least_25percent'] = summary['checks'].pop('battery_reserve_at_least_20percent')
        summary['hardware_estimates'] = {
            'peak_motor_current_A': float(current_est.max()),
            'peak_motor_voltage_V': float(voltage_est.max()),
            'minimum_motor_voltage_headroom_V': float(np.min(pack_voltage[:, None]-voltage_est)),
            'note': 'Algebraic motor envelope estimates; phase-resistance convention and idle-current scaling are approximations.'}
        summary['checks']['estimated_motor_current_within_15A_design_limit'] = bool(current_est.max() <= 15)
        summary['checks']['estimated_motor_voltage_feasible'] = bool(np.all(voltage_est <= pack_voltage[:, None]))
        summary['passed'] = all(summary['checks'].values())
        summary['limitations'] = [
            'Selected catalog components; custom frame joints and inertia/propeller-inertia approximations need physical validation.',
            'Propeller coefficients fit manufacturer computational static data, not a measured motor-prop combination.',
            'Battery OCV and efficiencies are approximations; no thermal, sensor-noise or complete motor-electrical model.',
            'Mission tolerance and translation averaging intervals are team choices; passing is not instructor signoff.',
            'Ground contact remains a point model, not landing-gear contact geometry.',
        ]
        folder = args.output/name
        save_results(records, summary, folder, mission.battery_records)
        mission_figures(records, mission, touchdown, folder)
        soc = mission.battery.soc
        summaries.append(summary)
        print(f"{name}: {'PASS' if summary['passed'] else 'FAIL'}; "
              f"energy={summary['battery']['energy_used_Wh']:.3f} Wh; remaining charge={100*soc:.1f}%", flush=True)
    (args.output/'summary.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    if not all(s['passed'] for s in summaries):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
