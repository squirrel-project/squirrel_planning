(define (problem squirrel_robot_behaviour_problem)
(:domain squirrel_robot_behaviour)
(:objects
	o1 o2 b1 b2 - object
	kenny - robot
	kenny_wp oxygen1_wp oxygen2_wp battery1_wp battery2_wp - waypoint
	c1 c2 c3 - child
)

(:init
	(not_busy)
	(robot_at kenny kenny_wp)
	(object_at o1 oxygen1_wp)
	(object_at o2 oxygen2_wp)
	(object_at b1 battery1_wp)
	(object_at b2 battery2_wp)

	(is_of_type o1 oxygen)
	(is_of_type o2 oxygen)
	(is_of_type b1 battery)
	(is_of_type b2 battery)

	(gripper_empty kenny)

	(= (power kenny) 4.5)
)
(:goal (and
	(child_has_oxygen c1)
	(child_has_oxygen c2)
)))

