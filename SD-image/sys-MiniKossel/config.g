; Configuration file for Mini Kossel kit from Think3DPrint3D

; Communication and general
M111 S0                             ; Debug off
M550 PMiniKossel		        	; Machine name and Netbios name (can be anything you like)
M551 Preprap                        ; Machine password (used for FTP)
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0xED ; MAC Address
;*** Adjust the IP address and gateway in the following 2 lines to suit your network
M552 P0.0.0.0						; IP address (0 = use DHCP)
M554 P192.168.1.1                   ; Gateway
M553 P255.255.255.0                 ; Netmask
M555 P2                             ; Set output to look like Marlin
M575 P1 B57600 S1					; Comms parameters for PanelDue

G21                                 ; Work in millimetres
G90                                 ; Send absolute coordinates...
M83                                 ; ...but relative extruder moves

; Axis and motor configuration
M569 P0 S1							; Drive 0 goes forwards
M569 P1 S1							; Drive 1 goes forwards
M569 P2 S1							; Drive 2 goes forwards
M569 P3 S1							; Drive 3 goes forwards
M569 P4 S1							; Drive 4 goes forwards
M574 X2 Y2 Z2 S1					; set endstop configuration (all endstops at high end, active high)
;*** The homed height is deliberately set too high in the following - you will adjust it during calibration
M665 R105.6 L215.0 B85 H240			; set delta radius, diagonal rod length, printable radius and homed height
M666 X0 Y0 Z0						; put your endstop adjustments here, or let auto calibration find them
M92 X80 Y80 Z80						; Set axis steps/mm
M906 X1000 Y1000 Z1000 E500 I60		; Set motor currents (mA) and increase idle current to 60%
M201 X1000 Y1000 Z1000 E1000		; Accelerations (mm/s^2)
M203 X20000 Y20000 Z20000 E3600		; Maximum speeds (mm/min)
M566 X1200 Y1200 Z1200 E1200		; Maximum instant speed changes mm/minute

; Thermistors
;*** If you have a Duet board stickered "4.7K", change R1000 to R4700 to the following M305 commands
M305 P0 T100000 B3950 R1000 H30 L0	; Put your own H and/or L values here to set the bed thermistor ADC correction
M305 P1 T100000 B3974 R1000 H30 L0	; Put your own H and/or L values here to set the first nozzle thermistor ADC correction
M305 P2 T100000 B3974 R1000 H30 L0	; Put your own H and/or L values here to set the second nozzle thermistor ADC correction
M570 S180							; Hot end may be a little slow to heat up so allow it 180 seconds

; Tool definitions
M563 P0 D0 H1                       ; Define tool 0
G10 P0 S0 R0                        ; Set tool 0 operating and standby temperatures
;*** If you have a dual-nozzle build, un-comment the next 2 lines
;M563 P1 D1 H2                      ; Define tool 1
;G10 P1 S0 R0                       ; Set tool 1 operating and standby temperatures
M92 E663:663                       	; Set extruder steps per mm

; Z probe and compensation definition
;*** If you have an IR zprobe instead of a switch, change P4 to P1 in the following M558 command
M558 P4 X0 Y0 Z0					; Z probe is a switch and is not used for homing any axes
G31 X0 Y0 Z4.80 P500				; Set the zprobe height and threshold (put your own values here)

;*** If you are using axis compensation, put the figures in the following command
M556 S78 X0 Y0 Z0                   ; Axis compensation here

M208 S1 Z-0.2						; set minimum Z
;
T0									; select first hot end
