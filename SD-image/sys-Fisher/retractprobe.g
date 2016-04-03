; Probe retraction routine for Mini Kossel
M564 S0					; don't apply limits
G1 Z40 F10000			; raise head
G1 X-59 Y66 Z35			; move over the post
G1 Z7 F500				; push probe down on post
G1 Z35 F10000			; raise head again
G1 X0 Y0				; move to somewhere sensible
M564 S1					; apply limits again