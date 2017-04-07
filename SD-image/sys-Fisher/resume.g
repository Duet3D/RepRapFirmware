; Resume macro file
M83				; relative extruder movement
G1 R1 Z2		; move to 2mm above resume point
G1 R1			; lower nozzle to resume point
G1 E4 F2500		; undo the retraction