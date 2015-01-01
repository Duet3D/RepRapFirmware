G91
G1 Z5 F200
G90
M558 P1
G1 X-240 F2000 S1
G92 X0
G1 X3 F200
G1 X-30 S1
G92 X0
G1 X15 F500 ; adjust the X value to put the nozzle on the edge of the bed
G92 X0
M558 P2
G1 X45 F2000
G92 Y0
G1 Y-240 F2000 S1
G92 Y0
G1 Y3 F200
G1 Y-30 S1
G92 Y0
G1 Y3 F500 ; adjust the Y value to put the nozzle on the edge of the bed
G92 Y0
G30
G1 Z5 F200
G1 X0 Y0
G1 Z0
