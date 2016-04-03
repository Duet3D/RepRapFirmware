; Resume macro file
G1 R1 Z2 F5000	; move to 2mm above resume point
G1 R1			; lower nozzle to resume point
M83				; relative extruder moves
G1 E4 F2500		; undo the retraction