; Prusa i3 homeall file for use with dc42 Duet firmware
; Adjust the bed upper and lower limits in config.g (M208 commands) to get the correct homing positions

G91
G1 Z4 F200					; raise head 4mm to keep it clear of the bed
G1 X-240 Y-240 F3000 S1		; course home X and Y
G1 X4 Y4 F600				; move 4mm away from the homing switches
G1 X-10 Y-10 S1				; fine home X and Y
G90

G1 X100 Y100 F2000			; move to bed centre for probing
G30
; This file leaves the head at the zprobe trigger height so that you can slip a piece of paper under it and then do G0 Z0 to check the height.
; If you prefer to send the printer to X0Y0Z0, un-comment the following lines
;G1 X0 Y0 Z0 F5000
