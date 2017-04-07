; Auto calibration routine for large delta printer with IR probe

M561						; clear any bed transform, otherwise homing may be at the wrong height
G28							; home the printer

; Probe the bed and do auto calibration
G1 X0 Y130 Z10 F15000		; go to just above the first probe point to speed up probing

; Adjust the H parameters in the following commands if neeeded to correct for probe height errors caused by effector tilt etc.
G30 P0 X0 Y130 Z-99999 H0
G30 P1 X-112.6 Y65 Z-99999 H0
G30 P2 X-112.6 Y-65 Z-99999 H0
G30 P3 X0 Y-130 Z-99999 H0
G30 P4 X112.6 Y-65 Z-99999 H0
G30 P5 X112.6 Y65 Z-99999 H0
G30 P6 X0 Y67.5 Z-99999 H0
G30 P7 X-58.5 Y33.8 Z-99999 H0
G30 P8 X-58.5 Y-33.8 Z-99999 H0
G30 P9 X0 Y-67.5 Z-99999 H0
G30 P10 X58.5 Y-33.8 Z-99999 H0
G30 P11 X58.5 Y33.8 Z-99999 H0
G30 P12 X0 Y0 Z-99999 H0 S6

G1 X0 Y0 Z150 F15000		; get the head out of the way of the bed
