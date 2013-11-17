; RepRapPro Ormerod
; Standard configuration G Codes
M111 S1; Debug on
M500 Preprap; Set the password
M501 POrmerod; Set the machine's name
M502 P192.168.1.14; Set the IP address
G21 ; Work in mm
G90 ; Absolute positioning
M83 ; Extrusions relative
G31 Z0.0 P500 ; Set Z probe height and threshold
M906 X800 Y800 Z800 E800 ; Motor currents (mA)
T0 ; Select extruder 0


