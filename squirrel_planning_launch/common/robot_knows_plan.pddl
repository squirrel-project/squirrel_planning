ff: found legal plan as follows

step    0: RAMIFICATE
        1: ASSUME_KNOWLEDGE BASIS_KB KB_TOY1
        2: RAMIFICATE
        3: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT NEAR_CHILD1
        4: GOTO_WAYPOINT ROBOT NEAR_CHILD1 CHILD1_LOCATION
        5: TAKE_OBJECT ROBOT CHILD1_LOCATION TOY1 CHILD1
        6: GOTO_WAYPOINT ROBOT CHILD1_LOCATION NEAR_CHILD1
        7: GOTO_WAYPOINT ROBOT NEAR_CHILD1 NEAR_BOX1
        8: DROP_OBJECT ROBOT NEAR_BOX1 NEAR_CHILD1 TOY1
        9: EXAMINE_AREA ROBOT AREA1
       10: OBSERVE-IS_OF_TYPE TOY1 CAR L0 L1 KB_TOY1
       11: RAMIFICATE
       12: GOTO_WAYPOINT ROBOT NEAR_BOX1 NEAR_BOX2
       13: PICKUP_OBJECT ROBOT NEAR_CHILD1 NEAR_BOX2 TOY1 CAR
       14: PUT_OBJECT_IN_BOX ROBOT BOX2_LOCATION NEAR_BOX2 TOY1 BOX2 CAR
       15: GOTO_WAYPOINT ROBOT NEAR_BOX2 NEAR_BOX1
       16: TIDY_OBJECT ROBOT TOY1 BOX2 CAR
       17: POP L1 L0
       18: RAMIFICATE
       19: PICKUP_OBJECT ROBOT NEAR_CHILD1 NEAR_BOX1 TOY1 DINOSAUR
       20: PUT_OBJECT_IN_BOX ROBOT BOX1_LOCATION NEAR_BOX1 TOY1 BOX1 DINOSAUR
       21: TIDY_OBJECT ROBOT TOY1 BOX1 DINOSAUR
       22: SHED_KNOWLEDGE KB_TOY1 BASIS_KB
     

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

