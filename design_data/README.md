# Design source data

`PER3_9x45MR.dat` was downloaded directly from
https://www.apcprop.com/files/PER3_9x45MR.dat on November 18, 2026.
It is manufacturer computational aerodynamic data, not an experimental thrust
stand measurement. Version/date are retained in the file header.

`design.py` extracts zero-airspeed rows from 4000 through 9000 RPM. Columns used:
RPM from the section header, thrust in N, power in W, and omega = RPM*2*pi/60.
Torque is power/omega. Fitting is least squares through the origin versus
omega squared. The reverse-rotation propeller is assumed aerodynamically equal.

Source URLs for all other selected components are in `design.py:SOURCES` and
the linked component table in `DRONE_DESIGN.md`. Quantities without manufacturer
support are labeled as geometry choices, simplified mass distributions, or
engineering allowances. Source retrieval does not imply stock reservation.
