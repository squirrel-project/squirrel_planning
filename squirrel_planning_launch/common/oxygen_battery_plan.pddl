ff: found legal plan as follows

step    0: RAMIFICATE
        1: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT PICKUP_WAYPOINT
        2: PICKUP_OBJECT ROBOT PICKUP_WAYPOINT KENNY_WAYPOINT OXYGEN1 OBJECT
        3: CLASSIFY_OBJECT_IN_HAND ROBOT OXYGEN1
        4: GOTO_WAYPOINT ROBOT PICKUP_WAYPOINT BASE_WAYPOINT
	5: GIVE_OBJECT ROBOT BASE_WAYPOINT OXYGEN1 CHILD
	6: show_lights robot child 0_255_0
	7: emote robot sound_CHEERING wiggle_idle
	8: TAKE_OBJECT ROBOT BASE_WAYPOINT BATTERY CHILD
	9: CLASSIFY_OBJECT_IN_HAND ROBOT BATTERY
	10: assume_knowledge basis_kb battery1_kb
	11: ramificate
	12: observe-is_of_type battery battery l1 l0 battery1_kb
	13: ramificate
	14: show_lights robot child 0_255_0
	15: emote robot sound_CHEERING wiggle_idle
	16: pop l1 l0
	17: ramificate
	18: show_lights robot child wiggle_error
	19: emote robot sound_NO 255_0_0
	20: jump 7
	21: shed_knowledge battery1_kb basis_kb
     

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

