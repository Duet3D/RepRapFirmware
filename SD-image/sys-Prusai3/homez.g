; Z homing file for Prusa i3 or similar printer using a Z probe for Z homing
G91
G1 Z5 F200
G90
G1 X100 Y100 F9000
G30
; Un-comment the following line if you want the head to actually move to Z=0 after homing
;G1 Z0 F200
