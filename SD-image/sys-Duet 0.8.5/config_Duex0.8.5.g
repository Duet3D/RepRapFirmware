; Configuration file for Duet 0.8.5 setup for a delta printer 20150720

; Communication and general
M111 S0                             	; Debug off
M550 PKossel2020                    	; Machine name and Netbios name (can be anything you like)
M551 Preprap                        	; Machine password (used for FTP)
M540 P0xBE:0xEF:0xDE:0xAD:0xFE:0xED 	; MAC Address
;*** Adjust the IP address and gateway in the following 2 lines to suit your network
M552 P192.168.1.14                  	; IP address
M554 P192.168.1.1                   	; Gateway
M553 P255.255.255.0                 	; Netmask
M555 P2                             	; Set output to look like Marlin
G21                                 	; Work in millimetres
G90                                 	; Send absolute coordinates...
M83                                 	; ...but relative extruder moves

;Turn off fans
M106 I1 ; fans are inverting
M106 S0 ; Turn off Fan 0 (backward compatible with older firmware)
M106 P1 S0 ; Turn off Fan 1

; Axis and motor configuration
M569 P0 S1				; Drive 0 goes forwards
M569 P1 S1				; Drive 1 goes forwards
M569 P2 S1				; Drive 2 goes forwards
M569 P3 S1				; Drive 3 goes forwards
M569 P4 S1				; Drive 4 goes forwards
M569 P5 S1				; Drive 5 goes forwards
M569 P6 S1				; Drive 6 goes forwards
M569 P7 S1				; Drive 7 goes forwards
M569 P8 S1				; Drive 8 goes forwards


M574 X2 Y2 Z2 S1			; set endstop configuration (all endstops at high end, active high)
;*** The homed height is deliberately set too high in the following - you will adjust it during calibration. 
M665 R174.2 L322.5 B130 H423.69		; set delta radius, diagonal rod length, printable radius and homed height
M666 X0.00 Y0.00 Z0.00			; put your endstop adjustments here, as given by auto calibration 22/05/15
M92 X80 Y80 Z80				; Set axis steps/mm
M906 X1000 Y1000 Z1000 E1000		; Set motor currents (mA)(Changed DC's E500 to E1000)
M201 X1000 Y1000 Z1000 E1000		; Accelerations (mm/s^2)
M203 X20000 Y20000 Z20000 E3600		; Maximum speeds (mm/min)
M566 X1200 Y1200 Z1200 E1200		; Maximum instant speed changes mm/minute


; Thermistors
; Duet0.8.5 uses 4.7K resistors
M305 P0 T100000 B3950 R4700 H30 L0	; Put your own H and/or L values here to set the bed thermistor ADC correction
M305 P1 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the first heater thermistor ADC correction
M305 P2 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the second heater thermistor ADC correction
M305 P3 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the third heater thermistor ADC correction
M305 P4 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the fourth heater thermistor ADC correction
M305 P5 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the fifth heater thermistor ADC correction
M305 P6 T100000 B4267 R4700 H30 L0	; Put your own H and/or L values here to set the sixth heater thermistor ADC correction
M570 S180				; Hot end may be a little slow to heat up so allow it 180 seconds

; Tool definitions
;*** Extruder 0
M563 P0 D0 H1                       	; Define tool 0
G10 P0 S0 R0                        	; Set tool 0 operating and standby temperatures

;*** If you have a 2 or greater extruder build, un-comment the next 2 lines
M563 P1 D1 H2                      	; Define tool 1
G10 P1 S0 R0                       	; Set tool 1 operating and standby temperatures

;*** If you have a 3 or greater extruder build, un-comment the next 2 lines
M563 P2 D2 H3                      	; Define tool 2
G10 P2 S0 R0                       	; Set tool 1 operating and standby temperatures

;*** If you have a 4 or greater extruder build, un-comment the next 2 lines
M563 P3 D3 H4                      	; Define tool 3
G10 P2 S0 R0                       	; Set tool 1 operating and standby temperatures

;*** If you have a 5 or greater extruder build, un-comment the next 2 lines
M563 P4 D4 H5                      	; Define tool 4
G10 P4 S0 R0                       	; Set tool 1 operating and standby temperatures

;*** If you have a 6 extruder build, un-comment the next 2 lines
M563 P5 D5 H6                      	; Define tool 5
G10 P5 S0 R0                       	; Set tool 1 operating and standby temperatures

M92 E663:663:663:663:663:663          ; Set extruder steps per mm for each extruder explicitly

// Z probe and compensation definition
;*** If you have an IR zprobe instead of a switch, change P4 to P1 in the following M558 command
M558 P1 X0 Y0 Z0			; Z probe is a DC42 Differential IR PCB and is not used for homing any axes
G31 X9.0 Y22.0 Z1.9 P500		; Set the zprobe height and threshold (put your own values here)

;*** If you are using axis compensation, put the figures in the following command
M556 S78 X0 Y0 Z0         	        ; Axis compensation here



