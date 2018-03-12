(define (problem squirrel_robot_behaviour_problem)
(:domain squirrel_robot_behaviour)
(:objects
	o1 o2 b1 b2 - object
	kenny - robot
	kenny_wp oxygen1_wp oxygen2_wp battery1_wp battery2_wp - waypoint
	c1 c2 c3 - child
	nono error - wiggle
	s1 s2 - sound
)

(:init
	(not_busy)
	(not_flashing_lights)
	(not_playing_sound)
	(not_gazing)
	(robot_at kenny kenny_wp)
	(object_at o1 oxygen1_wp)
	(object_at o2 oxygen2_wp)
	(object_at b1 battery1_wp)
	(object_at b2 battery2_wp)

	(is_of_type o1 oxygen)
	(is_of_type o2 oxygen)
	(is_of_type b1 battery)
	(is_of_type b2 battery)

	;(not_child_knows_what_robot_wants c1)
	;(not_child_knows_what_robot_wants c2)
	;(not_child_knows_what_robot_wants c3)

	(gripper_empty kenny)

	(= (power kenny) 4.5)
	(= (reciprocal c1) 0.0)
	(= (reciprocal c2) 0.0)
	(= (reciprocal c3) 0.0)
)
(:goal (and
	(has_battery)
)))
