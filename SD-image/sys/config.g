; RepRapPro Ormerod
; Standard configuration G Codes
M111 S1; Debug on
G21 ; mm
G90 ; Absolute positioning
M83 ; Extrusion relative
M107; Fan off
G31 Z0.7 P400 ; Set Z probe height and threshold
M906 X400 Y400 Z400 E400 ; Motor currents (mA)
;M201 X4000 Y4000 E4000 ; Accelerations (mms-2)
T0 ; Extruder 0


