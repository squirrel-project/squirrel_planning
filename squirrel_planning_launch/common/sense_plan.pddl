
ff: parsing domain file
domain 'CLASSIFY_OBJECTS' defined
 ... done.
ff: parsing problem file
problem 'SQUIRREL' defined
 ... done.



Cueing down from goal distance:   12 into depth [1]
                                  11            [1]
                                  10            [1]
                                   9            [1]
                                   8            [1]
                                   7            [1]
                                   6            [1]
                                   5            [1]
                                   4            [1]
                                   3            [1]
                                                H search space got empty !
                                                switching to complete search space now.
                                                [1][2][3][4][5][6][7][8][9][10][11][12][13]
                                   2            [1][2][3][4][5][6][7][8][9][10][11][12][13][14]
                                   1            [1]
                                   0            

ff: found legal plan as follows

step    0: RAMIFICATE
        1: GOTO_WAYPOINT ROBOT KENNY_WP NEAR_RED_CAR_SOFT_WP
        2: LOOK_AT_OBJECT ROBOT RED_CAR_SOFT_WP NEAR_RED_CAR_SOFT_WP RED_CAR_SOFT
        3: PICKUP_OBJECT ROBOT NEAR_RED_CAR_SOFT_WP RED_CAR_SOFT_WP RED_CAR_SOFT
        4: GOTO_WAYPOINT ROBOT NEAR_RED_CAR_SOFT_WP NEAR_BOX3_LOCATION
        5: DROP_OBJECT ROBOT NEAR_BOX3_LOCATION BOX3_LOCATION RED_CAR_SOFT BOX3
        6: LISTEN_TO_FEEDBACK ROBOT RED_CAR_SOFT BOX3 BOX3_LOCATION NEAR_BOX3_LOCATION
        7: OBSERVE-BELONGS_IN RED_CAR_SOFT BOX3 ROBOT L0 L1 BASIS_KB
        8: RAMIFICATE
        9: GOTO_WAYPOINT ROBOT NEAR_BOX3_LOCATION NEAR_TRICERATOPS_WP
       10: NEXT_OBSERVATION RED_CAR_SOFT TRICERATOPS BOX3 BOX1
       11: LOOK_AT_OBJECT ROBOT TRICERATOPS_WP NEAR_TRICERATOPS_WP TRICERATOPS
       12: PICKUP_OBJECT ROBOT NEAR_TRICERATOPS_WP TRICERATOPS_WP TRICERATOPS
       13: GOTO_WAYPOINT ROBOT NEAR_TRICERATOPS_WP NEAR_BOX1_LOCATION
       14: DROP_OBJECT ROBOT NEAR_BOX1_LOCATION BOX1_LOCATION TRICERATOPS BOX1
       15: LISTEN_TO_FEEDBACK ROBOT TRICERATOPS BOX1 BOX1_LOCATION NEAR_BOX1_LOCATION
       16: OBSERVE-BELONGS_IN TRICERATOPS BOX1 ROBOT L1 L2 BASIS_KB
       17: RAMIFICATE
       18: FINISH TRICERATOPS BOX1
       19: POP L2 L1
       20: RAMIFICATE
       21: FINISH TRICERATOPS BOX1
       22: POP L1 L0
       23: RAMIFICATE
       24: GOTO_WAYPOINT ROBOT NEAR_BOX3_LOCATION NEAR_WINNIE_POOH_WP
       25: LOOK_AT_OBJECT ROBOT WINNIE_POOH_WP NEAR_WINNIE_POOH_WP WINNIE_POOH
       26: NEXT_OBSERVATION RED_CAR_SOFT WINNIE_POOH BOX3 BOX2
       27: PICKUP_OBJECT ROBOT NEAR_WINNIE_POOH_WP WINNIE_POOH_WP WINNIE_POOH
       28: GOTO_WAYPOINT ROBOT NEAR_WINNIE_POOH_WP NEAR_BOX2_LOCATION
       29: DROP_OBJECT ROBOT NEAR_BOX2_LOCATION BOX2_LOCATION WINNIE_POOH BOX2
       30: LISTEN_TO_FEEDBACK ROBOT WINNIE_POOH BOX2 BOX2_LOCATION NEAR_BOX2_LOCATION
       31: OBSERVE-BELONGS_IN WINNIE_POOH BOX2 ROBOT L0 L1 BASIS_KB
       32: RAMIFICATE
       33: FINISH WINNIE_POOH BOX2
       34: POP L1 L0
       35: RAMIFICATE
       36: FINISH WINNIE_POOH BOX2
     

time spent:    0.09 seconds instantiating 9496 easy, 40 hard action templates
               0.17 seconds reachability analysis, yielding 3849 facts and 536 actions
               0.00 seconds creating final representation with 1437 relevant facts
               0.01 seconds building connectivity graph
             421.99 seconds searching, evaluating 217885 states, to a max depth of 14
             422.26 seconds total time



Can't open output file!



