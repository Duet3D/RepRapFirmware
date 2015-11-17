; RepRapPro Ormerod
; Standard configuration G Codes
<<<<<<< HEAD
M111 S0; Debug off
=======
>>>>>>> duet
M550 POrmerod; Set the machine's name
M551 Preprap; Set the password
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0xED ; Set the MAC address
M552 P192.168.1.14; Set the IP address
M553 P255.255.255.0; Set netmask
M554 P192.168.1.1; Set the gateway
M555 P2; Emulate Marlin USB output
T1 ; Select head 1
M92 E420; Set extruder steps/mm
G21 ; Work in mm
G90 ; Absolute positioning
M83 ; Extrusions relative
M558 P1 ; Turn Z Probe on
G31 Z0.5 P500 ; Set Z probe height and threshold
M906 X800 Y800 Z800 E800 ; Motor currents (mA)
M201 X1000 Y1000 Z15 E1000; acceleration tweaks for improved finish
M203 X15000 Y15000 Z300 E3600; speed tweaks for improved finish

