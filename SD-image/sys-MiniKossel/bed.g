; Auto calibration routine for delta printers
; Before running this, you should have set up your zprobe Z offset to suit your build, in the G31 command in config.g.

M561						; clear any bed transform, otherwise homing may be at the wrong height
G31 X0 Y0					; don't want any probe offset for this
G28							; home the printer

;*** Remove the following line if your Z probe does not need to be deployed
M98 Pdeployprobe.g			; deploy the mechanical Z probe

; The first time the mechanical probe is used after deployment, it gives slightly different results.
; So do an extra dummy probe here. The value stored gets overwritten later. You can remove this if you use an IR probe.
G30 P0 X0 Y0 Z-99999

; Probe the bed and do 6- or 7-factor auto calibration
G30 P0 X-73.6 Y-42.5 Z-99999	; X tower
G30 P1 X0 Y-85 Z-99999			; between X and Y towers
G30 P2 X73.6 Y-42.5 Z-99999		; Y tower
G30 P3 X73.6 Y20 Z-99999		; between Y and Z towers
G30 P4 X0 Y67 Z-99999			; Z tower
G30 P5 X-73.6 Y20 Z-99999		; between Z and X towers
G30 P6 X-36.8 Y-21.25 Z-99999	; half way to X tower
G30 P7 X36.8 Y-21.25 Z-99999	; half way to Y tower
G30 P8 X0 Y42.5 Z-99999			; half way to Z tower
G30 P9 X0 Y0 Z-99999 S6			; centre, and auto-calibrate 6 factors

;*** Remove the following line if your Z probe does not need to be retracted
M98 Pretractprobe.g				; retract the mechanical Z probe

G1 X0 Y0 Z150 F15000			; get the head out of the way of the bed
