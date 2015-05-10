; homeall file for use with dc42 Duet firmware on CoreXY printers
; This file assumes the endstop switches are at the low end of ach axis.
; Reverse the X and Y movement for high-end switches.
; Adjust the bed upper and lower limits in config.g (M208 commands) to get the correct homing positions
G91							; relative movement

G1 Z4 F200					; make sure the head is off the bed
G1 X-240 F3000 S1			; course home X
G1 X4 F600					; move X away 4mm

G1 X-10 S1					; fine home X
G1 Y-240 F3000 S1			; course home Y
G1 Y4 F600					; move Y away 4mm
G1 Y-10 S1					; fine home Y

; If you are using a microswitch for Z homing, insert similar code for the Z axis here,
; but use lower feed rates suitable for your Z axis.
G90

; If you homed the Z axis using an endstop switch, you can insert a G92 command here to correct the height.

; The following code assumes you are using a Z probe to do Z homing. Remove it if you are using a microswitch.
; Adjust the XY coordinates in the following to place the Z probe over a suitable spot,
; preferably neat the centred of the bed if your Z probe supports that
G1 X100 Y100 F2000
G30
; This file leaves the head at the zprobe trigger height so that you can slip a piece of paper under it and then do G0 Z0 to check the height.
; If you prefer to send the printer to X0Y0Z0, un-comment the following lines
;G1 X0 Y0 F5000
;G1 Z0
