RepRapFirmware 3.3beta1 (in preparation)
=======================

Upgrade notes:
- All extruders must be declared explicitly using M584. In previous firmware versions, one default extruder was assign to driver 3.
- [Duet 3 + expansion/tool boards] You must update the expansion and/or tool board firmware to 3.3beta1 also, otherwise movement will not work

New features:
- M17 is implemented
- [Duet 3 Mini only] M954 is partially implemented. A Duet 3 Mini used as an expansion board can support axis motors, extruder motors (but may not work if nonzero pressure advance is used), thermistor, PT100 and thermocouple temperature sensors, GpIn and GpOut pins (including servos). Heaters, fans, filament monitors, endstop switches, Z probes and other types of temperature sensor are not yet supported.
- [Duet 3 MB6HC] The maximum number of axes supported on Duet 3 MB6HC is increased to 15. Axis letters abcdefghijkl may be used in addition to XYZUVWABCD. Because GCode is normally case insensitive, these must be prefixed with a single quote character in GCode commands. For example, M584 'A1.2 would assign axis 'a' to driver 1.2, and G1 'A10 would move the 'a' axis to the 10mm or 10 degree position.

Bug fixes:
- It was not possible to delete a temperature sensor using M308 S# P"nil"
- When a sensor was configured on a CAN expansion board and M308 was subsequently used to create a sensor with the same number on a different board, the old sensor was not deleted
