; Ormerod 2 homeall file for use with dc42 Duet firmware
; Adjust the bed upper and lower limits in config.g (M208 commands) to get the correct homing positions
G91
G1 Z4 F200
G1 X-240 Y-240 F3000 S1
G1 X4 Y4 F600
G1 X-10 Y-10 S1
G90
; Adjust the XY coordinates in the following to place the IR sensor over a suitable spot
; If you are using a dc42 IR sensor then you can change the coordinates to be near the centre of the bed
G1 X45 Y5 F2000
G30
; This file leaves the head at the zprobe trigger height so that you can slip a piece of paper under it and then do G0 Z0 to check the height.
; If you prefer to send the printer to X0Y0Z0, un-comment the following lines
;G1 X0 Y0 F5000
;G1 Z0
