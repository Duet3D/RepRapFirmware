; Bed probing routine for Mini Kossel
M561						; clear bed transform, otherwise homing may be at the wrong height

; Macro files don't nest, so we can't do a G28 here. Include the code from homedelta.g instead.
G91							; use relative positioning
G1 S1 X320 Y320 Z320 F5000	; move all carriages up 320mm, stopping at the endstops
G1 S2 X-4 Y-4 Z-4 F1000		; move all towers down 5mm
G1 S1 X8 Y8 Z8 F500			; move towers up 8mm, stopping at the endstops
G90							; back to absolute positioning

M564 S0						; don't apply limits

; Deploy probe
G1 X25 Y93 Z50 F10000		; put probe arm next to belt
G1 X-7 F500
G1 X12 F2000

G1 X0 Y0 Z10 F5000

;*** Adjust the XY coordinates in the following M557 commands if necessary to suit your build and the position of the zprobe
G30 P0 X-50 Y-50 Z-99999
G30 P1 X-50 Y50 Z-99999
G30 P2 X50 Y50 Z-99999
G30 P3 X50 Y-50 Z-99999
G30 P4 X0 Y0 Z-99999 S

; Retract probe
G1 Z40 F10000				; raise head
G1 X-59 Y66 Z35				; move over the post
G1 Z7 F500					; push probe down on post
G1 Z35 F10000				; raise head again
G1 X0 Y0					; move to somewhere sensible

M564 S1						; apply limits again