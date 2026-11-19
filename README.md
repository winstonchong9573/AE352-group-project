# AE352 quadcopter simulation

The researched **D440 component design** is in [DRONE_DESIGN.md](DRONE_DESIGN.md),
with a bill of materials, a dimensioned drawing, calculated inertia, and fitted
APC propeller data. Run `python run_design.py` for this configuration; it writes
to `results/design/`. `python ae352.py` continues to run the original parameter
baseline described below. Design evidence is summarized in
[DESIGN_RESULTS.md](DESIGN_RESULTS.md).

Mechanical six-degree-of-freedom model driven only by four motor torques.
Includes rotor inertia, position/attitude feedback, ground support, three missions,
and a finite battery with voltage sag, current limits, drive losses and cutoff.
ER3 is evaluated for the modeled conditions; all three missions share one battery
by default. Motor/rotor and airframe parameters remain provisional; the team must
replace or justify them using documented components before claiming full ER4 or
hardware feasibility. Battery mass is INCLUDED in the 1.2 kg total, not added twice.

## Run

Use Python 3.10 or newer:

```powershell
python -m pip install -r requirements.txt
python ae352.py
python ae352.py --mission circle
python ae352.py --initial-soc 0.5
python ae352.py --fresh-battery-each-mission
python -m unittest -v
python ae352.py --dt 0.005 --output results/fine
python validate_results.py
```

If Windows selects a Microsoft Store Python alias, use your installed interpreter's
full path. Runs save files without opening windows. `--output folder` changes the
destination; `--dt 0.005` allows a finer integration check. Default step: 0.01 s.

Each mission saves `trajectory.csv`, `summary.json`, `states.png`, `motors.png`,
`path.png`, `battery.csv`, and `battery.png` under `results/<mission>/`. Aggregate checks are in
`results/summary.json`. A failed requirement check returns a nonzero exit code.

## Files

- `ae352.py`: physical parameters, rotation matrix, equations of motion, CLI entry.
- `simulation.py`: controllers, allocation, mission phases, RK4, metrics and plots.
- `battery.py`: battery ratings, losses, voltage/current calculation and depletion.
- `test_battery.py`: conservation, current limit, cutoff and depleted-mission tests.
- `test_simulation.py`: independent physics and recovery checks.
- `ae352_original.py`: untouched pre-implementation copy for comparison.

## Conventions and equations

World z up; body x forward, y left, z up. Rotor indices: front, left, rear,
right. Positive spin signs: `[+1, -1, +1, -1]`. Geometry is a plus configuration.
SI units throughout; angles in CSV are radians, plot angles are degrees.
State: `[x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,omega1,omega2,omega3,omega4]`.

Each rotor follows `Jr * omega_dot = torque - k_drag * omega**2` and
`thrust = k_thrust * omega**2`. Translation is `m*v_dot = R*[0,0,sum(thrust)] -
[0,0,m*g]`. Rotation follows `I*Omega_dot = moment - cross(Omega,I*Omega)`.
Euler rates use the ZYX conversion; pitch near +/-90 degrees is rejected.
No fuselage drag or rotor gyroscopic coupling is modeled.

Yaw uses motor reaction `-sum(spin*torque)`, preserving the starting model.
The controller's allocation uses the steady-speed torque/thrust ratio, with
actual transient yaw calculated from motor torque by the plant. This is a
simplified coupled body/rotor model, not a complete rotor angular-momentum model.

Position PD plus acceleration feedforward determines thrust direction. Attitude
PD determines moments. A plus-layout mixer produces desired rotor thrusts, then
rotor speeds; speed feedback produces bounded motor torque. Clipping enforces
nonnegative thrust, 650 rad/s speed and 0.12 N m torque limits. These are assumed
limits, not electrical ratings. Commands outside feasible allocation are clipped;
exact requested moments are then not guaranteed.

## Missions and declared tolerances

Tolerances below are team choices, not instructor-specified acceptance limits.

- Hover: ideal initial equilibrium at z=1 m for 120 s.
- Circle: radius 2 m, altitude 1 m, speed 0.5 m/s. Start on the circle with
  tangent velocity; run 70 s and evaluate the final 60 s, after a 10 s controller
  transient. Radius tolerance 0.10 m, altitude 0.05 m, speed 0.05 m/s.
- Maneuver: smooth 4 s takeoff; 5 m leg along +x; settle; yaw +90 degrees;
  second 5 m leg along +y; settle; descend at 0.008 m/s. Each translation
  reference lasts 5 s (nominal average 1 m/s), with zero endpoint velocity and
  acceleration. Settling is additional time, explicitly reported separately.
  Actual five-second displacement must be within 0.25 m of 5 m, endpoint error
  below 0.05 m, altitude error below 0.05 m, and stop speed below 0.05 m/s.
  Yaw error below 2 degrees. Settle conditions must hold 0.5 s; phases time out.
  The exact 1 m/s average is a reference target, not a guaranteed exact actual
  average; both movement-window and settling-inclusive averages are reported.

Ground is a unilateral flat support at z=0, with no bounce or sliding friction.
The maneuver starts on the ground with stopped rotors. Touchdown speed is
interpolated immediately before ground projection and must be <=0.01 m/s.
The simulation ends at touchdown; projected zero velocity is not used as proof
of slow landing. Landing takes about 125 s because of the low speed limit.

## Battery, power and ER3

The chosen battery is a modeled 6S1P pack of Molicel INR-21700-P45B cells.
Manufacturer values and assumptions are separated in [BATTERY_MODEL.md](BATTERY_MODEL.md).
Nominal pack voltage is 21.6 V; minimum rated capacity is 4.3 Ah; nominal energy
is 92.88 Wh. Six cells at maximum 70 g each plus assumed 60 g packaging weigh
0.48 kg. Remaining drone mass is 0.72 kg. The total inertia is still assumed.

Requested shaft power is `sum(torque * rotor_speed)`. Motor and ESC efficiencies
are assumed 0.85 and 0.95; electronics plus idle losses add 9 W. Battery current
solves `P = (Voc - I*R)*I`, using the low-current root. SOC decreases according
to charge removed with a conservative 0.8 effective-capacity factor. Electrical
energy integrates terminal power over time; energy and charge are distinct logs.

The supply enforces cell current, loaded-voltage and maximum-power limits and
reduces the actual torques passed to the flight dynamics if demand is infeasible.
Empty batteries disarm and terminate the mission as a failure. A depleted battery
cannot pass a hover or circle merely because those cases start in flight.
The acceptance checks additionally require at least 20% SOC remaining.

For `--mission all`, SOC carries over in order: hover, circle, maneuver. Flight
states still reset between these independent demonstrations; this is not one
continuous flight. Initial airborne position, velocity and rotor kinetic energy
are prescribed initial conditions, and setup/takeoff between separate scenarios
is not charged. Use `--fresh-battery-each-mission` to evaluate separate packs.

This is a power-budget model allowed by the assignment's ideal motor-torque
control assumption. It does not implement motor back-EMF, winding resistance,
voltage-dependent maximum torque, temperature, aging or regeneration. Battery
OCV, resistance and efficiencies are assumptions; source-backed battery ratings
do not establish a source-backed propulsion design. ER3 results are conditional
on these assumptions; ER4 remains partly open. Original mechanics-only results
have been replaced by battery-aware results, including the finer-step run.
