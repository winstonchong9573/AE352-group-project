# AE352 D440 drone design

Prepared November 18, 2026. This is a preliminary engineering design for the
course digital twin, with selected catalog components, a specified custom frame,
calculated mass properties, and simulation verification. It is not a
manufacturing release or a physically tested aircraft.

The selected configuration uses a **440 mm plus frame, four 9-inch propellers,
and a 6S battery**. Calculated mass is **1.375 kg before allowance and 1.513 kg
with a 10% distributed allowance**. The existing 1.2 kg example remains available
as the baseline; `python run_design.py` runs this selected physical design.

## Design files and reproduction

- [Dimensioned layout, PDF](design_output/drone_layout.pdf)
- [Dimensioned layout, PNG](design_output/drone_layout.png)
- [Component mass and placement spreadsheet](design_output/mass_budget.csv)
- [Bill of materials](design_output/bill_of_materials.csv)
- [Calculated parameters and compatibility checks](design_output/design_parameters.json)
- [Propeller fit plot](design_output/propeller_fit.png)
- [Flight results](results/design/summary.json)
- [Independent timestep and electrical-accounting checks](results/design/validation.json)

```powershell
python design.py
python run_design.py
python run_design.py --dt 0.005 --output results/design/fine
python -c "from pathlib import Path; from validate_results import validate; validate(Path('results/design'))"
python -m unittest -v
```

`design.py` calculates the physical parameters and supplies real `DroneParams`
and `BatteryParams` objects to the existing simulation. It does not overwrite the
earlier results. The source propeller file is included, so reproduction needs no
internet access. No new Python dependencies are required.

## Selected components

Masses below are installed estimates, not shipping weights. Published quantities
are distinguished from allowances. Sources were consulted on November 18, 2026;
stock and price are not guaranteed. Dimensions and ratings must match the exact
variant identified here.

| Item | Quantity | Selection and useful parameters | Mass used |
|---|---:|---|---:|
| Motor | 4 | [KDE2315XF-885](https://www.kdedirect.com/products/kde2315xf-885): 885 rpm/V; Kt 0.0108 N m/A; phase-to-phase resistance 0.127 ohm; idle current 0.5 A at 10 V; rotor inertia 0.041 kg cm2; body diameter 28.3 mm, length 28.5 mm | 75 g each, including leads/connectors |
| ESC | 4 | [KDEXF-UAS20LV](https://www.kdedirect.com/products/kdexf-uas20lv): 7.4–26.1 V; 20 A/180 s continuous rating; 35 A/5 s peak; no BEC; 22.5 x 32 mm PCB | 26 g each with leads/connectors |
| Normal propeller | 2 | [APC LP09045MR](https://www.apcprop.com/product/9x4-5mr/): 9 x 4.5 inches, two blades, hub thickness 8.636 mm, shaft hole 6.35 mm | 0.39 oz = 11.056 g each |
| Reverse propeller | 2 | [APC LP09045MRP](https://www.apcprop.com/product/9x4-5mrp/): matching reverse-rotation version | 11.056 g each |
| Prop retention | 4 sets | Supplied KDE M6 prop adapters, appropriate RH/LH retention hardware, and APC LPAR06E centering rings specified by APC; verify centering and thread engagement on the actual assembly | 5 g extra per rotor, allowance |
| Battery | 1 | [GNB45006S10M](https://www.gaoneng.shop/products/gaoneng-gnb-6s-22.2v-4500mah-10c-xt60-li-ion-battery-made-with-molicel-21700-p45b): assembled Molicel P45B 6S1P pack; 4.5 Ah advertised, 10C, XT60, 78 x 64 x 43 mm | 475 g, upper end of 460 +/-15 g specification |
| Flight controller | 1 | [Pixhawk 6C Mini Model A, current revision](https://docs.holybro.com/autopilot/pixhawk-6c-mini/technical-specification): 54.3 x 39 x 17.5 mm; IMUs and barometer included; maximum FC input 6 V | 42.4 g |
| Power module/PDB | 1 | [Holybro PM06 V2 14S, SKU15019](https://holybro.com/products/micro-power-module-pm06-v2): 5.2 V/3 A regulated output; analog voltage/current sensing; 35 x 35 x 5 mm; four ESC distribution connections | 24 g |
| GNSS/compass | 1 | [Holybro M10, standard 10-pin](https://holybro.com/products/m10-gps): 50 mm diameter, 14.4 mm high; less than 200 mA at 5 V; stated position accuracy 2 m CEP | 32 g |
| Local flow/range sensing | 1 | [Holybro H-Flow, SKU19006](https://docs.holybro.com/peripherals/h-flow-dronecan/overview): DroneCAN optical flow plus distance sensing; downward mounting; flow range starts at 80 mm | 15.2 g with enclosure |
| RC receiver | 1 | [RadioMaster RP1 V2](https://radiomasterrc.com/products/rp1-expresslrs-2-4ghz-nano-receiver): 2.4 GHz ExpressLRS, CRSF UART, 5 V; PCB 13 x 11 x 3 mm | 2.2 g with antenna |
| Carbon tube stock | 2 x 1 m | [Easy Composites CFT-WF-20-18-1](https://www.easycomposites.co.uk/20mm-woven-finish-carbon-fibre-tube): 20 mm OD, 18 mm ID, 86.5 g/m; cut four 180 mm arms and four 160 mm legs | 117.64 g installed |
| Carbon sheet stock | 1 sheet | [Easy Composites high-strength carbon sheet](https://www.easycomposites.co.uk/high-strength-carbon-fibre-sheet): select nominal 2 mm, 250 x 230 mm; cut two 120 mm squares and four 40 mm squares | 98.56 g installed using 2.80 kg/m2 |
| Custom root/end clamps | 8 | PA12 split-clamp blocks with M3 through-bolts/inserts; four root and four motor-end interfaces; final holes matched to motor drawings | 40 g total allowance |
| Other installed hardware | 1 lot | Landing-leg joints and feet; battery straps/pad; GPS mast; electronics standoffs; wiring; insulation; signal leads; M3 fasteners | 60 g allowance |

The KDE motor supports 3S–6S and limits propellers to 9 inches on 6S, matching this
selection. Its published 24 A/355 W limits apply for 180 seconds with specified
cooling. The ESC ratings also require airflow. The model uses lower operational
limits; this does not establish a thermal qualification.

The battery page has an inconsistent title/nominal-voltage label. Selection is
specifically the **GNB45006S10M 6S1P P45B** described in its specification table.
The model uses the [cell manufacturer's](https://www.molicel.com/wp-content/uploads/INR21700P45B_1.2_Product-Data-Sheet-of-INR-21700-P45B-80109.pdf)
3.6 V nominal and 4.3 Ah minimum values: 21.6 V nominal, 25.2 V full, and
92.88 Wh nominal minimum-capacity reference. Do not substitute a different cell
chemistry based only on a similar product name.

## Mechanical arrangement

Construction origin is the intersection of the arms, with x forward, y left,
z up. Motor centers are (+220,0), (0,+220), (-220,0), (0,-220) mm.
Rotors 1 and 3 rotate CCW viewed from above; rotors 2 and 4 rotate CW.
Mount each propeller with its intended top face upward and verify upward thrust.

The arms run radially from 40 to 220 mm. Their 180/20 = 9 length/diameter ratio
follows the lecture's suggested maximum of 10. Tube roots are captured between
the center plates using split clamps; motor plates attach through end clamps.
Avoid treating the tube wall as a threaded fastening material.

Nominal 2 mm sheet has published typical thickness 1.7 mm. Use that for the mass
and stack estimate, and measure actual stock before choosing clamp/standoff
lengths. Plate inner faces lie at z = +/-10 mm, accommodating 20 mm tubes.
Motor plate centers lie at z = +10.85 mm; propeller plane is approximately
z = +46 mm, with the adapter stack height treated as a layout estimate.

| Geometry | Selected/calculated value |
|---|---:|
| Opposite motor-center distance | 440 mm |
| Neighboring motor-center distance | 311.127 mm |
| Propeller diameter / pitch | 228.6 / 114.3 mm |
| Neighboring propeller tip gap | 82.527 mm |
| Lecture's 0.1R clearance target | 11.430 mm |
| Overall propeller envelope | 668.6 x 668.6 mm |
| Battery center | (0,0,-36.5) mm |
| Flight-controller center | (0,0,+24) mm |
| GPS center on mast | (0,0,+80) mm |
| H-Flow center, facing downward | (0,0,-75) mm |
| Landing legs | x/y-axis locations at radius 100 mm; feet z=-170 mm |

The flow sensor's assumed lower face is z=-80 mm, giving approximately 90 mm
clearance on the ground. Its 40 x 40 x 10 mm envelope is an allowance pending the
mounting drawing. Arrange connectors, straps, and the mast without obscuring
sensors or propellers. Exact clamps, holes, sensor brackets, and fastener
engagement are fabrication details still requiring a CAD/fit review.

## Mass and inertia calculation

The spreadsheet lists every modeled part's installed mass and location.
The battery is counted once. Another 137.523 g is included as an unallocated 10%
design allowance, distributed proportionally for mass-property calculations.
It is not an additional battery or additional available energy.

For each part, use an approximate local inertia tensor and shift it to the
combined center of mass:

```text
m = sum(m_i)
r_CG = sum(m_i*r_i) / m
I = sum(I_i + m_i*((d_i dot d_i)*Identity - outer(d_i,d_i)))
d_i = r_i - r_CG
```

Tubes use hollow-cylinder formulas; electronics and the battery use uniform
boxes; motors use bounding boxes. Propeller mass is approximated as a uniform
rod across the diameter. Its airframe inertia is averaged over rotation angle.
The 10% allowance multiplies both total mass and inertia by 1.1.

| Quantity | Value used |
|---|---:|
| Total mass | 1.5127478 kg |
| CG relative to arm plane | (0,0,-8.716) mm |
| Ixx | 0.01531776 kg m2 |
| Iyy | 0.01540988 kg m2 |
| Izz | 0.02640206 kg m2 |
| Rotating motor inertia | 4.1e-6 kg m2, manufacturer value converted from kg cm2 |
| Combined propeller + motor spinning inertia | 5.2248368e-5 kg m2, includes estimated propeller inertia |

The four arms/rotors are symmetric. Rectangular central equipment produces a
small Ixx/Iyy difference, which the equations support. Products of inertia are
zero to numerical precision in the centered layout. These are calculated
approximations, not measured assembled-aircraft inertias.

For a preliminary arm bending check, use the tube's published 64 GPa lengthwise
modulus, an ideal fixed root, a 170 mm effective free span, and maximum rotor
thrust. Predicted tube-only tip deflection is 0.074 mm and bending stress is
4.90 MPa. This calculation does not validate clamp stiffness, joint strength,
landing impact, fatigue, or vibration.

## Propulsion calculation

APC describes its published performance files as computational predictions using
propeller geometry, not measured test results. The included
[APC file](design_data/PER3_9x45MR.dat) is version v2022-0915, simulated
September 22, 2022. The selected rows have zero forward speed and 4000–9000 RPM.

Convert RPM to rad/s, obtain aerodynamic torque from shaft power/omega (avoiding
the coarsely rounded torque column), and fit through the origin:

```text
T = k_thrust * omega^2
Q = k_drag * omega^2
k = sum(omega^2 * measured_or_tabulated_quantity) / sum(omega^4)
```

The resulting values are **k_thrust = 9.8195211e-6 N/(rad/s)^2** and
**k_drag = 1.3611587e-7 N m/(rad/s)^2**. Maximum fitting errors over the selected
rows are 0.875% for thrust and 4.37% for torque. These errors describe fitting
the data, not physical prediction uncertainty. Reverse props are assumed to
have matching aerodynamic characteristics. Inflow, rotor interference, and
ground effect are not included.

| Operating quantity | Calculated result |
|---|---:|
| Hover thrust per rotor | 3.710 N |
| Hover speed | 5869.7 RPM = 614.67 rad/s |
| Hover shaft torque per motor | 0.05143 N m |
| Total hover shaft power | 126.44 W |
| Total modeled electrical hover power | 177.37 W |
| Operational speed cap | 8500 RPM = 890.118 rad/s |
| Operational shaft torque cap | 0.15 N m |
| Maximum thrust per rotor at speed cap | 7.780 N |
| Collective thrust / design weight at speed cap | 2.097 |
| Tip Mach at speed cap, assuming sound speed 343 m/s | 0.297 |

APC's MR limit is 105000/diameter-in-inches, or 11667 RPM for this propeller.
The 8500 RPM cap is lower. The torque cap is an engineering command limit, not
a published KDE continuous-torque rating.

For an approximate motor envelope check use I_motor = torque/Kt + I_idle and
V_motor = Ke*omega + R*I_motor. Simultaneously requesting both selected caps
requires approximately 14.39 A and 11.43 V. This is below the chosen 15 A motor
screening limit and the 18 V pack cutoff. The resistance conversion and
constant idle-current approximation need bench confirmation; this calculation
is not a complete BLDC/ESC electrical model.

## Electrical connections and battery assumptions

```text
GNB 6S pack / XT60
        |
        v
PM06 V2: current sensing + power distribution
        |---- raw battery voltage ----> 4 KDE ESCs ----> 4 KDE motors
        |
        +---- regulated 5.2 V --------> Pixhawk POWER input
                                           |---- PWM/control ----> ESC control inputs
                                           |---- GPS1 -----------> M10
                                           |---- CAN ------------> H-Flow
                                           +---- serial/CRSF ----> RP1
```

Follow the KDE/Pixhawk wiring instructions for the opto ESC control supply and
common reference. No BEC is present in these ESCs. Raw 6S voltage must not feed
the flight controller power input. Keep ESC battery leads within KDE's 12-inch
limit; the arm-mounted ESC placement permits this.

The PM06 PCB rating is higher, but Holybro rates the supplied XT60/12 AWG
assembly at **30 A continuous**. Therefore the simulation limits pack current
to 30 A, despite the cell's larger 45 A rating. Other selected battery settings:

| Setting | Selection / basis |
|---|---|
| Capacity | 4.3 Ah, cell datasheet minimum; 4.5 Ah pack advertised typical |
| Pack mass | 475 g; represented as 6 x 70 g plus 55 g packaging equivalent |
| Loaded-voltage cutoff | 18 V (3.0 V/cell), chosen above cell absolute discharge endpoint |
| Reserve | 25% charge remaining |
| Effective capacity factor | 0.8; retained conservative assumption |
| Cell DC resistance | 15 milliohm; datasheet typical at 50% SOC, held constant as an approximation |
| Motor / ESC efficiency | 0.80 / 0.95; assumed constant loss approximation |
| Electronics and idle allowance | 7 W electronics + 1 W per motor |
| OCV curve | Existing piecewise approximation, not a measured open-circuit curve |

The 25% reserve is deliberate: with the 18 V cutoff and supply losses, the
20%-SOC case gives only about 1.996 thrust/weight at the design mass. At 25%
SOC the model supports the selected 2.097 ratio. The body/rotor simulation
still checks actual supplied torque after battery limiting.

Shaft-power-based electrical accounting under-resolves high-current, zero-speed
startup and does not model temperature or voltage-dependent motor losses. The
operating-envelope estimate and low mission currents are useful checks, not a
replacement for a motor test. The initial airborne rotor energy in hover/circle
and repositioning between missions are not charged to the battery.

## Verification and scope

`run_design.py` runs hover, circle and maneuver with the same selected battery's
charge carried between missions. Flight states reset between demonstrations.
It exports the existing flight plots plus velocity/error and pre-contact
landing-speed figures. The finer run uses dt=0.005 s. All numerical outcomes
are recorded in the linked JSON files; the human-readable result digest is
[DESIGN_RESULTS.md](DESIGN_RESULTS.md).

The current maneuver evaluates its nominal five-second translation separately
from settling. Both averages are reported. Team-selected error tolerances and
the averaging interval still need agreement with the instructor before calling
the 1 m/s requirement formally satisfied.

M10 alone cannot establish centimeter-level path accuracy: its stated accuracy
is 2 m CEP. H-Flow provides local flow/range measurements, but neither hardware
has a simulated estimator/noise model here. Real path repeatability and the
1 cm/s landing limit remain physical validation tasks. The simulation's ideal
state feedback and point ground contact are explicitly retained.

This package supplies component selection and calculated support for ER4;
simulation supports ER1–ER3 under the documented assumptions. Custom joints,
assembled mass/inertia, prop retention fit, sensor integration, and actual
propulsion tests remain necessary before a hardware build is considered
qualified. A complete off-the-shelf-aircraft bonus is not claimed: the frame
uses purchased stock with custom fabricated clamps and plates.
