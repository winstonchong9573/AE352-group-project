# Power-supply model and parameter provenance

## Project requirement

ER3 requires the performance goals to be achieved without exhausting the power
supply. ER4 requires realistic, implementable parameters (total mass 0.1-10 kg),
preferably supported by documented parts. The project allows ideal instantaneous
motor-torque commands; a full motor electrical transient model is not mandatory.

## Documented cell ratings

Primary source: [Molicel P45B manufacturer datasheet, version 1.2](https://www.molicel.com/wp-content/uploads/INR21700P45B_1.2_Product-Data-Sheet-of-INR-21700-P45B-80109.pdf).
Checked November 18, 2026.

| Quantity | Manufacturer rating | Model value |
|---|---|---|
| Nominal cell voltage | 3.6 V | 3.6 V |
| Full-charge voltage | 4.2 V | 4.2 V |
| Discharge endpoint | 2.5 V | 2.5 V loaded cutoff |
| Capacity | 4.5 Ah typical, 4.3 Ah minimum | 4.3 Ah |
| Continuous discharge | 45 A with 80 C cutoff | 45 A current ceiling; temperature not simulated |
| Cell mass | 70 g maximum | 70 g |
| Typical DC impedance at 50% SOC | 15 milliohm | 0.015 ohm, approximated as constant |

Pack design choice: 6 series, 1 parallel (6 cells). Nominal energy is
`6 * 3.6 * 4.3 = 92.88 Wh`. Packaging adds an assumed 60 g to 420 g of cells.
This describes a feasible cell arrangement, not a verified commercial assembled
pack with specified protection, wiring, connectors and thermal management.

## Assumptions that are NOT manufacturer measurements

| Quantity | Assumed value |
|---|---|
| Resistance away from the specified 50% SOC condition | Held at 0.015 ohm; temperature/SOC dependence omitted |
| Motor efficiency | 85% |
| ESC efficiency | 95% |
| Avionics power | 5 W |
| Idle drive losses | 1 W per motor |
| Effective discharge-capacity factor | 0.8 |
| SOC reserve | 20% |
| Packaging mass | 0.060 kg |
| OCV curve | Piecewise-linear approximation below |

OCV per cell uses SOC knots `[0, .1, .2, .5, .8, 1]` and voltage knots
`[2.5, 3.2, 3.4, 3.6, 3.9, 4.2]`. It is a modeling assumption, not a fit to
the datasheet's loaded discharge curves. The lecture proposes fitting cell curves;
that would improve fidelity but is not explicitly required by the project.

## Equations and integration

1. Shaft power: `Pshaft = sum(tau_i * omega_i)`; positive motoring only.
2. Requested terminal power: `Preq = Pshaft/(eta_motor*eta_ESC) + 9 W`.
3. Pack OCV: `Voc = Ns * OCVcell(SOC)`.
4. Pack resistance: `R = Ns/Np * Rcell`.
5. Terminal voltage: `V = Voc - I*R`.
6. Current solves `P = V*I`: use the stable lower-current quadratic root
   `I = 2P/(Voc + sqrt(Voc^2 - 4RP))`.
7. Maximum allowed current is the minimum of cell-current rating, current at
   the maximum-power point, and current at loaded-voltage cutoff.
8. If demand exceeds available terminal power, reduce shaft torque proportionally
   after subtracting fixed losses. The actual limited torque drives the rotors.
9. `SOC_next = SOC - I*dt/(3600*C_Ah*eta_discharge)`.
10. `Eterminal_next = Eterminal + V*I*dt/3600`, in Wh.

The 0.8 discharge factor deliberately derates charge capacity; it is not another
motor efficiency. Do not multiply logged terminal Wh by it again. Nominal Wh is
a nameplate reference, not exact remaining energy: varying OCV and internal
resistance mean energy does not equal `SOC * nameplate_Wh` exactly.

Current and power are held over one integration/control step. State of charge
and energy are updated once per completed step. Logs describe the state at their
timestamp; the final sample is not charged an additional step. Battery cutoff
stops the run and marks failure rather than pretending a successful landing.

## Remaining handoff to design/procurement

Select and document actual motor, ESC, propeller, frame and electronics, including
their mass, torque/speed envelope, thrust/drag coefficients and efficiency map.
Replace assumed OCV/constant-resistance behavior with validated data and check temperature if using
the 45 A cell rating. Recalculate inertia with the selected battery placement.
Until then, ER3 is a conditional simulation demonstration and full ER4 and the
off-the-shelf component bonus must not be claimed.

The selected D440 design now supplies a component selection and calculated
parameters in [DRONE_DESIGN.md](DRONE_DESIGN.md). It uses a documented assembled
GNB pack and more conservative operating limits; `run_design.py` passes that
configuration explicitly. The earlier settings described in this file remain
the baseline. Catalog-supported design and simulated operation still require
the physical validation described in the design document.
