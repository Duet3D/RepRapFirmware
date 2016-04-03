; Configuration file for RepRapPro Fisher - delta 3D printer

; Communication and general
M111 S0                             ; Debug off
M550 PRepRapProFisher1		        ; Machine name (can be anything you like)
M551 Preprap                        ; Machine password (used for FTP)
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0x43 ; MAC Address
;*** Adjust the IP address and gateway in the following 2 lines to suit your network
M552 P0.0.0.0                  ; IP address
;M554 P192.168.1.1                   ; Gateway
M553 P255.255.255.0                 ; Netmask
M555 P2                             ; Set output to look like Marlin
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
M665 R81.0 L160.0 B75 H169			; set delta radius, diagonal rod length, printable radius and homed height
M666 X0.0 Y0.0 Z0.0  			; put your endstop adjustments here
M92 X87.489 Y87.489 Z87.489  		; Set axis steps/mm
M906 X800 Y800 Z800 E1000			; Set motor currents (mA)
M201 X4000 Y4000 Z4000 E4000		; Accelerations (mm/s^2)
M203 X15000 Y15000 Z15000 E9000		; Maximum speeds (mm/min)
M210 Z50                               ; Homing feedrate
M566 X1200 Y1200 Z1200 E1200		; Maximum instant speed changes mm/minute

; Thermistors
M305 P1 R4700                   	

; Tool definitions
M140 H-1                            ; Disable heatbed
M563 P0 D0 H1                       ; Define tool 0
G10 P0 S210 R-273                        ; Set tool 0 operating and standby temperatures
M301 H1 P12 I0.4 D80 W180 B300      ; Set tool 0 PID parameters
M92 E310 	                       	; Set to 137 for direct drive. Set extruder steps per mm

;// Z probe and compensation definition
;*** If you have an IR zprobe instead of a switch, change P4 to P1 in the following M558 command
M558 P4 X0 Y0 Z0 H4					; Z probe is a switch and is not used for homing any axes
G31 X0 Y0 Z-0.1 P200			; Set the zprobe height and threshold (put your own values here)

;*** If you are using axis compensation, put the figures in the following command
M556 S78 X0 Y0 Z0                   ; Axis compensation here

