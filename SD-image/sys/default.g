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
G90                                 ; Send absolute corrdinates...
M83                                 ; ...but relative extruder moves
M906 X800 Y1000 Z800 E800           ; Set motor currents (mA)
M563 P1 D0 H1                       ; Define tool 1
G10 P1 S0 R0                        ; Set tool 1 operating and standby temperatures
M92 E420                            ; Set extruder steps per mm
M558 P1                             ; Use an unmodulated Z probe
G31 Z2.5 P370                       ; Set the probe height and threshold
M557 P0 X60 Y0                      ; Four... 
M557 P1 X60 Y165                    ; ...probe points...
M557 P2 X222 Y165                   ; ...for bed...
M557 P3 X222 Y0                     ; ...levelling
M556 S78 X0 Y0 Z0                   ; Put your axis compensation here
M201 X800 Y800 Z15 E1000            ; Accelerations (mm/s^2)
M203 X15000 Y15000 Z100 E3600       ; Maximum speeds (mm/min)
M566 X600 Y600 Z30 E20              ; Minimum speeds mm/minute

