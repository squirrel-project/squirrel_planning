ff: found legal plan as follows

step    0: RAMIFICATE
        1: DETECT_CHILDREN
        2: EMOTE ROBOT SOUND_HELLO WIGGLE_ERROR
        3: FOLLOW_CHILD CLOSEST_CHILD
        4: GOTO_VIEW_WAYPOINT
        5: EXAMINE_OBJECT
        6: ASSUME_KNOWLEDGE BASIS_KB KB_TOY1
        7: RAMIFICATE
        8: OBSERVE-TOY_AT_RIGHT_BOX L0 L1 KB_TOY1
        9: RAMIFICATE
       10: EMOTE ROBOT SOUND_CHEERING WIGGLE_ERROR
       11: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT PICKUP_WAYPOINT
       12: POP L1 L0
       13: RAMIFICATE
       14: EMOTE ROBOT SOUND_NO WIGGLE_ERROR
       15: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT PICKUP_WAYPOINT
       16: DETECT_CHILDREN
       17: SHED_KNOWLEDGE KB_TOY1 BASIS_KB
       18: JUMP 2
     

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

