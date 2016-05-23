; Homing file for RepRapFirmware on Kossel
G91							; use relative positioning
G1 S1 X700 Y700 Z700 F2500	; move all carriages up 700mm, stopping at the endstops
G1 S2 X-3 Y-3 Z-3			; move all carriages down 3mm
G1 S1 X6 Y6 Z6 F250			; move carriages slowly up 6mm, stopping at the endstops
G1 Z-5 F2000				; down a few mm so that we can centre the head
G90							; back to absolute positioning
G1 X0 Y0 F2000				; centre the head and set a reasonable feed rate
