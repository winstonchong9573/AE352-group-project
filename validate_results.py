"""Compare saved 0.01/0.005 s runs and verify electrical log conservation."""
import csv
import json
from pathlib import Path
import numpy as np


def validate(root=Path('results')):
    coarse = json.loads((root/'summary.json').read_text())
    fine = json.loads((root/'fine/summary.json').read_text())
    checks = {}
    details = {}
    for a, b in zip(coarse, fine):
        name = a['mission']
        checks[f'{name}_both_pass'] = a['passed'] and b['passed']
        checks[f'{name}_energy_converged'] = abs(a['battery']['energy_used_Wh']-b['battery']['energy_used_Wh']) < .01
        checks[f'{name}_soc_converged'] = abs(a['battery']['final_soc']-b['battery']['final_soc']) < 1e-4
        for subdir in [root/name, root/'fine'/name]:
            with (subdir/'battery.csv').open() as f:
                rows = list(csv.DictReader(f))
            times = np.array([float(r['time_s']) for r in rows])
            energy = sum(float(r['power_W'])*dt/3600 for r, dt in zip(rows, np.diff(times)))
            charge = sum(float(r['current_A'])*dt/3600 for r, dt in zip(rows, np.diff(times)))
            saved = json.loads((subdir/'summary.json').read_text())
            summary = saved['battery']
            key = f'{subdir.relative_to(root)}'
            checks[f'{key}_energy_integral'] = abs(energy-summary['energy_used_Wh']) < 1e-8
            checks[f'{key}_charge_integral'] = abs(charge-summary['charge_used_Ah']) < 1e-8
            bp = saved['battery_parameters']
            soc_drop = charge/(bp['cell_capacity_Ah']*bp['parallel']*bp['discharge_efficiency'])
            checks[f'{key}_soc_balance'] = abs(summary['initial_soc']-summary['final_soc']-soc_drop) < 1e-8
        details[name] = {'energy_difference_Wh': abs(a['battery']['energy_used_Wh']-b['battery']['energy_used_Wh']),
                         'final_soc_difference': abs(a['battery']['final_soc']-b['battery']['final_soc'])}
    checks['same_pack_soc_carried'] = all(abs(coarse[i]['battery']['initial_soc']-coarse[i-1]['battery']['final_soc']) < 1e-12 for i in range(1, len(coarse)))
    report = {'passed': bool(all(checks.values())), 'checks': {k: bool(v) for k, v in checks.items()},
              'differences': details, 'total_energy_Wh': sum(s['battery']['energy_used_Wh'] for s in coarse),
              'final_soc': coarse[-1]['battery']['final_soc']}
    (root/'validation.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    validate()
