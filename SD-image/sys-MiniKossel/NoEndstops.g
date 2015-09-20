; Home without endstops
M906 X300 Y300 Z300			; reduce motor current to be gentle on the endstops
G91							; use relative positioning
G1 S2 X320 Y320 Z320 F5000	; move all carriages up 320mm
G90							; back to absolute positioning
M906 X800 Y800 Z800			; motor currents back to normal
G92 X0 Y0 Z497.47			; define position

; Do 3-factor calibration to remove the endstop errors
G1 X-117 Y-67.5 Z10 F10000	; go to just above the first probe point to speed up probing
G30 P0 X-117 Y-67.5 Z-99999	; X tower
G30 P1 X117 Y-67.5 Z-99999	; Y tower
G30 P2 X0 Y135 Z-99999 S3	; Z tower, and 3-factor calibrate

G1 X-117 Y-67.5 Z10 F15000	; go to just above the first probe point to speed up probing
G30 P0 X-117 Y-67.5 Z-99999	; X tower
G30 P1 X0 Y-135 Z-99999		; between X and Y towers
G30 P2 X117 Y-67.5 Z-99999	; Y tower
G30 P3 X117 Y67.5 Z-99999	; between Y and Z towers
G30 P4 X0 Y135 Z-99999		; Z tower
G30 P5 X-117 Y67.5 Z-99999	; between Z and X towers
G30 P6 X-65 Y37.5 Z-99999	; half way to between Z and X towers
G30 P7 X-65 Y-37 Z-99999	; half way to X tower
G30 P8 X0 Y-75 Z-99999		; half way to between X and Y towers
G30 P9 X65 Y-37.5 Z-99999	; half way to Y tower
G30 P10 X65 Y37.5 Z-99999	; half way to between Y and Z towers
G30 P11 X0 Y75 Z-99999		; half way to Z tower
G30 P12 X0 Y0 Z-99999 S6	; centre, and 6-factor calibrate

G1 X0 Y0 Z150 F15000		; get the head out of the way of the bed
