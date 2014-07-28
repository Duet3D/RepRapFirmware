; Configuration file for RepRap Ormerod
; RepRapPro Ltd
;
M111 S0                             ; Debug off
M550 PMy RepRapPro Ormerod          ; Machine name (can be anything you like)
M551 Preprap                        ; Machine password (currently not used)
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0xED ; MAC Address
M552 P192.168.1.14                  ; IP address
M553 P255.255.255.0                 ; Netmask
M554 P192.168.1.1                   ; Gateway
M555 P2                             ; Set output to look like Marlin
G21                                 ; Work in millimetres
G90                                 ; Send absolute coordinates...
M83                                 ; ...but relative extruder moves
M906 X800 Y1000 Z800 E800           ; Set motor currents (mA)
M563 P1 D0 H1                       ; Define tool 1
G10 P1 S0 R0                        ; Set tool 1 operating and standby temperatures
M92 E420                            ; Set extruder steps per mm
M558 P1                             ; Use an unmodulated Z probe or an intelligent Z probe
G31 Z1.20 P500                      ; Set the probe height and threshold (put your own values here)
M305 P0 H0 L0						; Put your own H and/or L values here to set the bed thermistor ADC correction
M305 P1 H0 L0						; Put your own H and/or L values here to set the first nozzle thermistor ADC correction
M305 P2 H0 L0						; Put your own H and/or L values here to set the second nozzle thermistor ADC correction
M557 P0 X60 Y0                      ; Four... 
M557 P1 X60 Y165                    ; ...probe points...
M557 P2 X222 Y165                   ; ...for bed...
M557 P3 X222 Y0                     ; ...levelling
;M557 P3 X141 Y82.5                 ; 5th probe point for levelling (un-comment this if you are using a differential IR probe)
M556 S78 X0 Y0 Z0                   ; Put your axis compensation here
M201 X800 Y800 Z15 E1000            ; Accelerations (mm/s^2)
M203 X15000 Y15000 Z100 E3600       ; Maximum speeds (mm/min)
M566 X600 Y600 Z30 E20              ; Minimum speeds mm/minute
M208 X214 Y210						; set axis travel
M208 X-8 S1							; set axis minimum
