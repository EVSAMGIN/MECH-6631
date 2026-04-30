This solution tracks 6 user selected targets

Tracking thread applies six separate colour masks (RGBY) using HSV filters and then merges the image

Run the solution and follow the instructions in the console

Be mindful of the nomenclature: 

-C1F = Car 1 Front
-C1R = Car 1 Rear
-OBS1= Obstacle 1
etc...

-place cursor on targets and press 'C'
-press 'V' to cycle through the different masks

-Centroid positions are stored in the TargetPositions struct (defined in vision_custom.h), accessed via the global target_positions variable

-The struct holds three arrays: ic[6] (horizontal centroid), jc[6] (vertical centroid), and valid[6] (whether the target was successfully tracked that frame)

Array index order maps to targets as follows:

[0] = C1F (Car 1 Front)

[1] = C1R (Car 1 Rear)

[2] = C2F (Car 2 Front)

[3] = C2R (Car 2 Rear)

[4] = OBS1 (Obstacle 1)

[5] = OBS2 (Obstacle 2)

ic is the column (x) coordinate and jc is the row (y) coordinate, both in pixels relative to the top-left of the 640×480 image

Positions are updated each frame inside track_objects() and are only valid when the corresponding valid[t] flag is true