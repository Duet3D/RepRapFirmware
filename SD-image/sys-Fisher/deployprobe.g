; Probe deployment routine for Mini Kossel
M564 S0					; don't apply limits
G1 X25 Y93 Z40 F10000	; put probe arm next to belt
G1 X-5 F500				; move probe arm across belt
G1 X12 F1000			; move probe back
G1 X0 Y0 F10000			; move to somewhere sensible
M564 S1					; apply limits again