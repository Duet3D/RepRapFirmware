; RepRapPro Ormerod
; Standard configuration G Codes
M111 S1; Debug on
M501 POrmerod
M500 Preprap
G21 ; mm
G90 ; Absolute positioning
M83 ; Extrusion relative
G31 Z0.0 P500 ; Set Z probe height and threshold
M906 X800 Y800 Z800 E800 ; Motor currents (mA)
T0 ; Extruder 0


