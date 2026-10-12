; Cura-style sample: two layers of a 40 mm square, retraction, travel, G92 E0 reset
M82 ; absolute extrusion
G90
G28 ; home (ignored by the robot)
M104 S200
G92 E0
G0 F3000 X0 Y0 Z0.3
G1 F1200 X40 Y0 E1.6
G1 X40 Y40 E3.2
G1 X0 Y40 E4.8
G1 X0 Y0 E6.4
G1 F2400 E5.4 ; retract
G0 F3000 X5 Y5 Z0.6 ; travel
G1 F2400 E6.4 ; unretract (prime in place)
G92 E0 ; new layer, E reset
G1 F1200 X35 Y5 E0.6
G1 X35 Y35 E2.4 ; faster flow on this edge
G1 X5 Y35 E3.0
G1 X5 Y5 E3.6
G1 F2400 E2.6 ; retract
G0 F3000 Z10
M107
