"""Lumped battery/drive model; measured cell ratings, declared loss assumptions.

Source: https://www.molicel.com/wp-content/uploads/INR21700P45B_1.2_Product-Data-Sheet-of-INR-21700-P45B-80109.pdf
No motor electrical transient, temperature, aging or regenerative charging model.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class BatteryParams:
    series: int = 6
    parallel: int = 1
    cell_capacity_Ah: float = 4.3  # datasheet minimum (typical 4.5)
    cell_nominal_V: float = 3.6
    cell_full_V: float = 4.2
    cell_cutoff_V: float = 2.5
    cell_max_current_A: float = 45.0  # datasheet rating has 80 C thermal cutoff
    cell_mass_kg: float = .070  # datasheet maximum
    packaging_mass_kg: float = .060  # assumption: wiring, enclosure, protection
    cell_resistance_ohm: float = .015  # datasheet typical DC at 50% SOC; held constant as an approximation
    discharge_efficiency: float = .8  # conservative effective-capacity factor
    motor_efficiency: float = .85  # assumption; constant over operating range
    esc_efficiency: float = .95  # assumption
    electronics_W: float = 5.0  # assumption, flight computer and electronics
    motor_idle_W_each: float = 1.0  # assumed loss, also covers zero-speed operation
    reserve_soc: float = .20

    def __post_init__(self):
        for key, value in vars(self).items():
            if not np.isfinite(value):
                raise ValueError(f'{key} must be finite')
        if any(getattr(self, k) <= 0 for k in ['series', 'parallel', 'cell_capacity_Ah',
               'cell_nominal_V', 'cell_full_V', 'cell_cutoff_V', 'cell_max_current_A', 'cell_mass_kg']):
            raise ValueError('Cell ratings and counts must be positive')
        if not isinstance(self.series, int) or not isinstance(self.parallel, int):
            raise ValueError('Cell counts must be integers')
        if not 0 < self.cell_cutoff_V < self.cell_nominal_V < self.cell_full_V:
            raise ValueError('Voltage ratings must be ordered cutoff < nominal < full')
        if any(not 0 < getattr(self, k) <= 1 for k in
               ['discharge_efficiency', 'motor_efficiency', 'esc_efficiency']):
            raise ValueError('Efficiencies must be in (0, 1]')
        if any(getattr(self, k) < 0 for k in ['packaging_mass_kg', 'cell_resistance_ohm',
                                            'electronics_W', 'motor_idle_W_each']):
            raise ValueError('Losses, mass and resistance cannot be negative')
        if not 0 <= self.reserve_soc < 1:
            raise ValueError('Reserve must be in [0, 1)')

    @property
    def mass_kg(self):
        return self.series*self.parallel*self.cell_mass_kg+self.packaging_mass_kg

    @property
    def capacity_Ah(self):
        return self.parallel*self.cell_capacity_Ah

    @property
    def resistance_ohm(self):
        return self.series/self.parallel*self.cell_resistance_ohm

    @property
    def nominal_energy_Wh(self):
        return self.series*self.cell_nominal_V*self.capacity_Ah


class Battery:
    def __init__(self, params=None, soc=1.):
        self.params = params or BatteryParams()
        if not np.isfinite(soc) or not 0 <= soc <= 1:
            raise ValueError('Initial SOC must be in [0,1]')
        self.soc = float(soc)
        self.energy_Wh = 0.
        self.charge_Ah = 0.
        self.cutoff = False

    def open_circuit_voltage(self):
        p = self.params
        # Approximate monotone OCV-SOC curve, NOT a fit to manufacturer data.
        cell_V = np.interp(self.soc, [0, .1, .2, .5, .8, 1],
                           [p.cell_cutoff_V, 3.2, 3.4, 3.6, 3.9, p.cell_full_V])
        return p.series*cell_V

    def supply(self, requested_torque, speeds, dt=None):
        """Compute a feasible constant-power load; return actual shaft torques/log.

        Current solves P=(Voc-I*R)*I on its lower-current branch. Limit current
        before the maximum-power point and before undervoltage. Scale requested
        torque when the electrical supply cannot provide its requested power.
        """
        p = self.params
        torque = np.asarray(requested_torque, dtype=float)
        speed = np.maximum(np.asarray(speeds, dtype=float), 0.)
        if torque.shape != (4,) or speed.shape != (4,) or not np.all(np.isfinite(torque)) or not np.all(np.isfinite(speed)) or np.any(torque < 0):
            raise ValueError('Four finite nonnegative torques and finite speeds required')
        voc, resistance = self.open_circuit_voltage(), p.resistance_ohm
        efficiency = p.motor_efficiency*p.esc_efficiency
        overhead = p.electronics_W+4*p.motor_idle_W_each
        shaft_request = float(torque@speed)
        requested_W = overhead+shaft_request/efficiency
        min_voltage = p.series*p.cell_cutoff_V
        current_limit = p.parallel*p.cell_max_current_A
        if dt is not None:
            if not np.isfinite(dt) or dt <= 0:
                raise ValueError('Time step must be positive')
            # Do not consume more remaining charge than exists in this step.
            current_limit = min(current_limit, self.soc*p.capacity_Ah*p.discharge_efficiency*3600/dt)
        if resistance > 0:
            current_limit = min(current_limit, voc/(2*resistance), max(0., (voc-min_voltage)/resistance))
        available_W = current_limit*(voc-current_limit*resistance)
        unavailable = self.soc <= 0 or self.cutoff or available_W < overhead or voc <= min_voltage
        if unavailable:
            self.cutoff = True
            actual = np.zeros(4)
            delivered_W = current = 0.
        else:
            delivered_W = min(requested_W, available_W)
            scale = min(1., max(0., (delivered_W-overhead)*efficiency/max(shaft_request, 1e-12))) if shaft_request > 0 else 1.
            actual = torque*scale
            # Stable form of the quadratic root, including R=0.
            current = 2*delivered_W/(voc+np.sqrt(max(0., voc*voc-4*resistance*delivered_W)))
        voltage = voc-current*resistance
        return actual, {'soc': self.soc, 'voltage_V': float(voltage), 'current_A': float(current),
                        'power_W': float(delivered_W), 'requested_power_W': requested_W,
                        'shaft_power_W': float(actual@speed), 'energy_used_Wh': self.energy_Wh,
                        'charge_used_Ah': self.charge_Ah, 'cutoff': self.cutoff,
                        'power_limited': bool(delivered_W+1e-9 < requested_W)}

    def advance(self, log, dt):
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('Time step must be positive')
        charge = log['current_A']*dt/3600
        self.charge_Ah += charge
        self.energy_Wh += log['power_W']*dt/3600
        self.soc = max(0., self.soc-charge/(self.params.capacity_Ah*self.params.discharge_efficiency))
        if self.soc <= 0:
            self.cutoff = True
