ff: found legal plan as follows

step    0: RAMIFICATE
        1: DETECT_CHILDREN
        2: EMOTE ROBOT SOUND_HELLO WIGGLE_ERROR
        3: DETECT_RESPONSE
        4: EMOTE ROBOT SOUND_SURPRISE WIGGLE_NO
        5: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT PICKUP_WAYPOINT
        6: TAKE_OBJECT ROBOT PICKUP_WAYPOINT TOY1 CHILD1
        7: EXAMINE_OBJECT_IN_HAND ROBOT TOY1
        8: ASSUME_KNOWLEDGE BASIS_KB KB_TOY1
        9: RAMIFICATE
       10: OBSERVE-BELONGS_IN TOY1 BOX2 L1 L2 KB_TOY2
       11: RAMIFICATE
       12: GOTO_WAYPOINT ROBOT PICKUP_WAYPOINT NEAR_BOX2
       13: PUT_OBJECT_IN_BOX ROBOT BOX2_LOCATION NEAR_BOX2 TOY1 BOX2 CAR
       14: TIDY_OBJECT ROBOT TOY1 BOX2 CAR
       15: GOTO_WAYPOINT ROBOT NEAR_BOX2 PICKUP_WAYPOINT
       16: POP L1 L0
       17: RAMIFICATE
       18: GOTO_WAYPOINT ROBOT PICKUP_WAYPOINT NEAR_BOX1
       19: PUT_OBJECT_IN_BOX ROBOT BOX1_LOCATION NEAR_BOX1 TOY1 BOX1 DINOSAUR
       20: TIDY_OBJECT ROBOT TOY1 BOX1 DINOSAUR
       21: GOTO_WAYPOINT ROBOT NEAR_BOX1 PICKUP_WAYPOINT
       22: SHED_KNOWLEDGE KB_TOY1 BASIS_KB
       23: RAMIFICATE
       24: OBSERVE-SORTING_DONE L0 L1 BASIS_KB
       25: RAMIFICATE
       26: START_PHASE3
       27: POP L1 L0
       28: RAMIFICATE
       29: JUMP 5
     

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

