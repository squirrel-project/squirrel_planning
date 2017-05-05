
ff: parsing domain file
domain 'CLASSIFY_OBJECTS' defined
 ... done.
ff: parsing problem file
problem 'SQUIRREL' defined
 ... done.



Cueing down from goal distance:   13 into depth [1]
                                  12            [1][2]
                                  11            [1]
                                  10            [1]
                                   9            [1]
                                   7            [1]
                                   6            [1]
                                   5            [1]
                                   4            [1]
                                   3            [1]
                                                H search space got empty !
                                                switching to complete search space now.
                                                [1][2][3][4][5][6][7][8][9][10][11][12][13][14]
                                   2            [1][2][3][4][5][6][7][8][9][10][11][12][13][14][15]
                                   1            [1]
                                   0            

ff: found legal plan as follows

step    0: RAMIFICATE
        1: GOTO_WAYPOINT ROBOT KENNY_WP NEAR_BROWN_BEAR_WP
        2: LOOK_AT_OBJECT ROBOT BROWN_BEAR_WP NEAR_BROWN_BEAR_WP BROWN_BEAR
        3: PICKUP_OBJECT ROBOT NEAR_BROWN_BEAR_WP BROWN_BEAR_WP BROWN_BEAR
        4: GOTO_WAYPOINT ROBOT NEAR_BROWN_BEAR_WP KENNY_WP
        5: GIVE_OBJECT ROBOT END_FOLLOW_WP BROWN_BEAR CHILD
        6: FOLLOW_CHILD ROBOT CHILD KENNY_WP BROWN_BEAR
        7: INSPECT_OBJECT ROBOT BROWN_BEAR
        8: OBSERVE-BELONGS_IN BROWN_BEAR BOX1 ROBOT L0 L1 BASIS_KB
        9: RAMIFICATE
       10: GOTO_WAYPOINT ROBOT END_FOLLOW_WP NEAR_YELLOW_DINOSAUR_WP
       11: NEXT_OBSERVATION BROWN_BEAR YELLOW_DINOSAUR BOX1 BOX2
       12: LOOK_AT_OBJECT ROBOT YELLOW_DINOSAUR_WP NEAR_YELLOW_DINOSAUR_WP YELLOW_DINOSAUR
       13: PICKUP_OBJECT ROBOT NEAR_YELLOW_DINOSAUR_WP YELLOW_DINOSAUR_WP YELLOW_DINOSAUR
       14: GOTO_WAYPOINT ROBOT NEAR_YELLOW_DINOSAUR_WP KENNY_WP
       15: GIVE_OBJECT ROBOT END_FOLLOW_WP YELLOW_DINOSAUR CHILD
       16: FOLLOW_CHILD ROBOT CHILD KENNY_WP YELLOW_DINOSAUR
       17: INSPECT_OBJECT ROBOT YELLOW_DINOSAUR
       18: OBSERVE-BELONGS_IN YELLOW_DINOSAUR BOX2 ROBOT L1 L2 BASIS_KB
       19: RAMIFICATE
       20: FINISH YELLOW_DINOSAUR BOX2
       21: POP L2 L1
       22: RAMIFICATE
       23: FINISH YELLOW_DINOSAUR BOX2
       24: POP L1 L0
       25: RAMIFICATE
       26: GOTO_WAYPOINT ROBOT END_FOLLOW_WP NEAR_CALF_WP
       27: LOOK_AT_OBJECT ROBOT CALF_WP NEAR_CALF_WP CALF
       28: NEXT_OBSERVATION BROWN_BEAR CALF BOX1 BOX2
       29: PICKUP_OBJECT ROBOT NEAR_CALF_WP CALF_WP CALF
       30: GOTO_WAYPOINT ROBOT NEAR_CALF_WP KENNY_WP
       31: GIVE_OBJECT ROBOT END_FOLLOW_WP CALF CHILD
       32: FOLLOW_CHILD ROBOT CHILD KENNY_WP CALF
       33: INSPECT_OBJECT ROBOT CALF
       34: OBSERVE-BELONGS_IN CALF BOX2 ROBOT L0 L1 BASIS_KB
       35: RAMIFICATE
       36: FINISH CALF BOX2
       37: POP L1 L0
       38: RAMIFICATE
       39: FINISH DOLFIN BOX2
     

time spent:    0.08 seconds instantiating 9679 easy, 40 hard action templates
               0.25 seconds reachability analysis, yielding 4397 facts and 458 actions
               0.00 seconds creating final representation with 1325 relevant facts
               0.00 seconds building connectivity graph
              11.49 seconds searching, evaluating 7166 states, to a max depth of 15
              11.82 seconds total time



Can't open output file!



