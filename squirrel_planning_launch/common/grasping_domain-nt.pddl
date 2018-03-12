(define (domain squirrel_tidy_room)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions)

(:types
	robot
	waypoint
	object
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(grasped ?o - object)
)

(:action pickup_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:precondition (and
		(robot_at ?r ?wp)
	)
	:effect (and 
		(grasped ?o)
	)
)

(:action putdown_object
	:parameters (?v - robot ?wp - waypoint ?o - object)
	:precondition (and
		(robot_at ?r ?wp)
	)
	:effect (and 
		(not (grasped ?o))
	)
)
)

