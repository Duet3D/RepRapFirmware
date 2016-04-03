; Homing and height setting routine for delta printers
G28							; home the printer

;*** Remove the following line if your Z probe does not need to be deployed
M98 Pdeployprobe.g			; deploy the mechanical Z probe

G1 X0 Y0 Z8 F5000			; put head just above bed centre
G30							; probe bed, set height

;*** Remove the following line if your Z probe does not need to be deployed
M98 Pretractprobe.g			; deploy the mechanical Z probe
