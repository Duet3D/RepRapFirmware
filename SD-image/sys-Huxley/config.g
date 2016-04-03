; Huxley Duo config file for dc42 Duet firmware
; Based on RepRapPro version 1.04
M111 S0								; Debug off
M550 PMyHuxleyDuo					; Machine name (can be anything you like)
M551 Preprap						; Machine password (used for FTP access)
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0x14	; MAC Address
M552 P0.0.0.0						; IP address (0 = use DHCP)
M553 P255.255.255.0					; Netmask
M554 P192.168.1.1					; Gateway
M555 P2								; Set output to look like Marlin
G21									; Work in millimetres
G90									; Send absolute coordinates...
M83									; ...but relative extruder moves
M574 X0 Y1 Z0 S1					; set endstop configuration (Y endstop only, at low end, active high)
M569 P0 S1							; Reverse the X motor
M569 P3 S0							; Don't reverse the extruder motor
M92 X87.4890 Y87.4890 Z4000			; Set axis steps/mm
M906 X600 Y600 Z600 E600			; Set motor currents (mA)
M305 P0 T100000 R4700               ; bed thermistor is 100K with 4K7 series resistor
M305 P1 R4700                       ; first nozzle thermistor has 4K7 series resistor
M305 P2 R4700                       ; second nozzle thermistor has 4K7 series resistor
M92 E660							; Set extruder steps per mm
M558 P2								; Use a modulated Z probe (change to P1 if you have a dc42 diff IR zprobe)
G31 Z0.6 P550						; Set the probe height and threshold (deliberately too high to avoid bed crashes on initial setup)
M556 S78 X0 Y0 Z0					; Put your axis compensation here
M201 X3000 Y3000 Z150 E500			; Accelerations (mm/s^2)
M203 X15000 Y15000 Z100 E3600		; Maximum speeds (mm/min)
M566 X200 Y200 Z30 E20				; Maximum jerk rates mm/minute
M563 P0 D0 H1						; Define tool 0
G10 P0 S-273 R-273					; Set tool 0 operating and standby temperatures
;M563 P1 D1 H2						; Define tool 1 Uncomment if you have a dual colour upgrade
;G10 P1 X19 S-273 R-273				; Set tool 1 operating and standby temperatures Uncomment if you have a dual colour upgrade
M208 X130 Y138 Z95					; set axis maxima (adjust to suit your machine)
M208 X-13 Y-4 Z-0.5 S1				; set axis minim (adjust to make X=0 and Y=0 the edge of the bed after homing)
;
T0									; select first hot end
