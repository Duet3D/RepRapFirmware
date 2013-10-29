; RepRapPro Ormerod
; Standard configuration G Codes
M111 S1; Debug on
G21 ; mm
G90 ; Absolute positioning
M83 ; Extrusion relative
M107; Fan off
G31 Z0.7 P344 ; Set Z probe height and threshold
M906 X500 Y500 Z500 E500 ; Motor currents (mA)
;M201 X4000 Y4000 E4000 ; Accelerations (mms-2)
T0 ; Extruder 0


