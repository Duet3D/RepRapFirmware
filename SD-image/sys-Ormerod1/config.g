; Ormerod 1 config file for dc42 Duet firmware
M111 S0                             ; Debug off
M550 PMy Ormerod			        ; Machine name (can be anything you like)
M551 Preprap                        ; Machine password (currently not used)
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
M574 X0 Y2 Z0 S1					; set endstop configuration (Y endstop only, at high end, active high)
M92 X87.4890 Y87.4890 Z4000			; Set axis steps/mm
M906 X800 Y1000 Z800 E800           ; Set motor currents (mA)
M563 P0 D0 H1                       ; Define tool 0
G10 P0 S0 R0                        ; Set tool 1 operating and standby temperatures
M92 E420                        	; Set extruder steps per mm (single nozzle)
;*** If you have a dual-nozzle build, remove or comment out the previous line, and un-comment the following 3 lines
;M563 P1 D1 H2                      ; Define tool 1
;G10 P1 S0 R0                       ; Set tool 1 operating and standby temperatures
;M92 E420:420						; Set extruder steps/mm (dual nozzle)
;*** If you have a modulated IR probe without on-board microcontroller, change P1 to P2 in the following
M558 P1                             ; Use an unmodulated Z probe or an intelligent Z probe
G31 Z1.20 P500                      ; Set the probe height and threshold (put your own values here)
;*** If you have a Duet board with 4.7K thermistor series resistors, change R1000 to R4700 to the following M305 commands
M305 P0 R1000 H0 L0					; Put your own H and/or L values here to set the bed thermistor ADC correction
M305 P1 R1000 H0 L0					; Put your own H and/or L values here to set the first nozzle thermistor ADC correction
M305 P2 R1000 H0 L0					; Put your own H and/or L values here to set the second nozzle thermistor ADC correction
;*** Adjust the XY coordinates in the following M557 commands to suit your build and the position of the IR sensor
M557 P0 X60 Y0                      ; Four... 
M557 P1 X60 Y165                    ; ...probe points...
M557 P2 X222 Y165                   ; ...for bed...
M557 P3 X222 Y0                     ; ...levelling
;M557 P4 X141 Y82.5                 ; 5th probe point for levelling (un-comment this if you are using a dc42 differential IR probe)
;*** if you are using axis compensation, put the figures in the following command
M556 S78 X0 Y0 Z0                   ; Axis compensation here
M201 X800 Y800 Z15 E1000            ; Accelerations (mm/s^2)
M203 X15000 Y15000 Z100 E3600       ; Maximum speeds (mm/min)
M566 X600 Y600 Z30 E20              ; Minimum speeds mm/minute
M208 X214 Y210 Z200					; set axis maxima (adjust to suit your machine)
M208 X-8 Y0 Z-1 S1					; set axis minimum (adjust to make X=0 the edge of the bed)
;
T0									; select first hot end
