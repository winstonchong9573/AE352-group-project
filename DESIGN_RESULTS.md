# D440 design verification

Results generated November 18, 2026 with the sourced configuration in
[DRONE_DESIGN.md](DRONE_DESIGN.md). These are simulation results with the
documented model assumptions, not flight-test measurements.

## Calculated design

| Quantity | Result |
|---|---:|
| Mass including 10% allowance | 1.51275 kg |
| Opposite motor-center spacing | 440 mm |
| Propellers | Four APC 9 x 4.5 inch, alternating rotation |
| Adjacent propeller clearance | 82.53 mm |
| Hover speed | 5869.7 RPM |
| Modeled electrical hover power | 177.37 W |
| Maximum collective thrust/weight at 25% reserve | 2.097 |
| Design checks | 15 of 15 passed |

## Missions at dt = 0.01 s

All three pass the existing mission checks plus the added motor current/voltage
envelope checks. They carry charge from one battery between runs, with flight
states reinitialized. Full machine-readable evidence is in
[results/design/summary.json](results/design/summary.json).

| Mission | Duration | Energy | Final charge | Evidence |
|---|---:|---:|---:|---|
| Hover | 120.00 s | 5.9124 Wh | 92.90% | Ideal equilibrium maintained at 1 m |
| Circle | 70.00 s, final 60 s evaluated | 3.4493 Wh | 88.67% | Maximum evaluated radius error 0.000595 m; speed error 0.000193 m/s |
| Maneuver | 151.06 s | 7.4518 Wh | 79.28% | Both legs/yaw/landing complete; touchdown speed 0.008 m/s |
| Combined | 341.06 s across separate runs | 16.8136 Wh | 79.28% | No battery cutoff or power limiting |

Peak modeled battery current is 18.856 A, below the selected 30 A harness limit.
Minimum modeled terminal voltage is 22.482 V, above the selected 18 V cutoff.
Peak estimated motor current is 14.389 A, below the chosen 15 A envelope limit.
Motor envelope estimates use simplified resistance and idle-current conventions.

Useful plots:

- [Hover states](results/design/hover/states.png)
- [Circle path](results/design/circle/path.png)
- [Circle errors/speed](results/design/circle/requirements.png)
- [Maneuver path](results/design/maneuver/path.png)
- [Maneuver phases/speed](results/design/maneuver/requirements.png)
- [Landing speed immediately before contact](results/design/maneuver/touchdown.png)
- [Battery during maneuver](results/design/maneuver/battery.png)
- [Propeller fit](design_output/propeller_fit.png)

## Interpretation of the translation requirement

The first leg covers 4.95375 m in its five-second commanded movement interval,
an actual average of 0.99075 m/s. The second covers 4.95454 m, averaging
0.99091 m/s. Both then settle near 5 m with endpoint errors about 7.7 mm.
Including settling, the averages are approximately 0.606 m/s. Therefore the
existing tolerance checks pass, but this is not proof of an exact 1 m/s average
over a complete start-to-settled-stop interval. The report must disclose the
interval and obtain agreement on its interpretation, or revise that trajectory.

## Software and numerical verification

The 22-unit-test suite passed. It covers the original physics and battery
checks plus mass accounting, parallel-axis translation invariance, conversion
of manufacturer units, the APC 6000-RPM static point, selected-design hover
equilibrium, and component compatibility.

The dt=0.005 s run also passed all three missions. The
[electrical-conservation and timestep comparison](results/design/validation.json)
passed all 28 checks. The largest mission energy difference was 0.000185 Wh;
the largest final charge-fraction difference was 2.10e-6. Energy/current
integration and charge balance were checked independently from the saved logs.
Touchdown speed differed by less than 2e-17 m/s, touchdown time by 0.01 s, and
five-second leg displacement by less than 0.000014 m between runs.

## What the evidence establishes

The component selection and calculations provide a reproducible preliminary
physical design. They do not establish measured thrust, thermal endurance,
assembled inertia, clamp/joint strength, estimator performance, or manufacturing
fit. The geometric drawing is a layout drawing; custom frame joints require
their own fabrication drawings. Sensor errors, body aerodynamic drag, rotor
gyroscopic coupling, and actual landing-gear contact are not simulated.
