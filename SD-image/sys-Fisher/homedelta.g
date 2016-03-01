; Homing file for RepRapFirmware on Mini Kossel
G91							; use relative positioning
;******* Change F500 in the following line by F5000 when you are finished commissioning
G1 S1 X180 Y180 Z180 F5000	; move all carriages up 320mm, stopping at the endstops
G1 S2 X-4 Y-4 Z-4 F1000		; move all towers down 5mm
G1 S1 X8 Y8 Z8 F500			; move towers up 8mm, stopping at the endstops
G1 Z-5						; down a few mm so that we can centre the head
G90							; back to absolute positioning
G1 X0 Y0 F2000				; centre the head and set a reasonable feed rate
G1 Z4 F12000
