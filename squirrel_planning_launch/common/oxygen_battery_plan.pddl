ff: found legal plan as follows

step    0: RAMIFICATE
        1: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT OXYGEN1_WAYPOINT
        2: PICKUP_OBJECT ROBOT OXYGEN1_WAYPOINT KENNY_WAYPOINT OXYGEN1 OBJECT
        3: CLASSIFY_OBJECT_IN_HAND ROBOT OXYGEN1
        4: GOTO_WAYPOINT ROBOT OXYGEN1_WAYPOINT BASE_WAYPOINT
	5: GIVE_OBJECT ROBOT BASE_WAYPOINT OXYGEN1 CHILD
	6: show_lights robot child 0_255_0
	7: emote robot sound_CHEERING wiggle_idle
	8: assume_knowledge basis_kb battery1_kb
	9: ramificate
	10: observe-classifiable_from BASE_WAYPOINT BATTERY_WAYPOINT BATTERY ROBOT l1 l0 battery1_kb
	11: ramificate
	12: observe-is_of_type battery battery l2 l1 battery1_kb
	13: ramificate
	14: show_lights robot child 0_255_0
	15: emote robot sound_CHEERING wiggle_idle
	16: pop l2 l1
	17: ramificate
	18: show_lights robot child wiggle_error
	19: emote robot sound_NO 255_0_0
	20: jump 10
	21: ramificate
	22: show_lights robot child wiggle_error
	23: emote robot sound_NO 255_0_0
	24: jump 10
	25: RAMIFICATE
	26: shed_knowledge battery1_kb basis_kb
	27: RAMIFICATE
        28: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT OXYGEN2_WAYPOINT
        26: PICKUP_OBJECT ROBOT OXYGEN2_WAYPOINT KENNY_WAYPOINT OXYGEN2 OBJECT
        27: CLASSIFY_OBJECT_IN_HAND ROBOT OXYGEN2
        28: GOTO_WAYPOINT ROBOT OXYGEN2_WAYPOINT BASE_WAYPOINT
	29: GIVE_OBJECT ROBOT BASE_WAYPOINT OXYGEN2 CHILD
	30: show_lights robot child 0_255_0
	31: emote robot sound_CHEERING wiggle_idle
	32: assume_knowledge basis_kb battery1_kb
	33: ramificate
	34: observe-classifiable_from BASE_WAYPOINT BATTERY_WAYPOINT BATTERY ROBOT l1 l0 battery1_kb
	35: ramificate
	36: observe-is_of_type battery battery l2 l1 battery1_kb
	37: ramificate
	38: show_lights robot child 0_255_0
	39: emote robot sound_CHEERING wiggle_idle
	30: pop l2 l1
	40: ramificate
	41: show_lights robot child wiggle_error
	42: emote robot sound_NO 255_0_0
	43: jump 34
	44: ramificate
	45: jump 34
	46: shed_knowledge battery1_kb basis_kb
	47: RAMIFICATE
	48: GOTO_WAYPOINT ROBOT KENNY_WAYPOINT OXYGEN3_WAYPOINT
        49: PICKUP_OBJECT ROBOT OXYGEN3_WAYPOINT KENNY_WAYPOINT OXYGEN3 OBJECT
        50: CLASSIFY_OBJECT_IN_HAND ROBOT OXYGEN3
        51: GOTO_WAYPOINT ROBOT OXYGEN3_WAYPOINT BASE_WAYPOINT
	52: GIVE_OBJECT ROBOT BASE_WAYPOINT OXYGEN3 CHILD
	53: show_lights robot child 0_255_0
	54: emote robot sound_CHEERING wiggle_idle
	55: assume_knowledge basis_kb battery1_kb
	56: ramificate
	57: observe-classifiable_from BASE_WAYPOINT BATTERY_WAYPOINT BATTERY ROBOT l1 l0 battery1_kb
	58: ramificate
	59: observe-is_of_type battery battery l2 l1 battery1_kb
	60: ramificate
	61: show_lights robot child 0_255_0
	62: emote robot sound_CHEERING wiggle_idle
	63: pop l2 l1
	64: ramificate
	65: show_lights robot child wiggle_error
	66: emote robot sound_NO 255_0_0
	67: jump 57
	68: ramificate
	69: jump 57
	70: shed_knowledge battery1_kb basis_kb
	71: RAMIFICATE

time spent:    0.07 seconds instantiating 1743 easy, 45 hard action templates
               0.00 seconds reachability analysis, yielding 1141 facts and 148 actions
               0.00 seconds creating final representation with 318 relevant facts
               0.00 seconds building connectivity graph
               1.13 seconds searching, evaluating 11741 states, to a max depth of 22
               1.20 seconds total time

