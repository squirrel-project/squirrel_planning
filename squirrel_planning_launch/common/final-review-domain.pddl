(define (domain squirrel_robot_behaviour)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :duration-inequalities :conditional-effects :numeric-fluents :equality)

(:types
	robot
	child
	waypoint
	object
	type
)

;; Emotions range from [-1,1]. None of the emotions are alowed to fall above 0.
;; Emotions in the range (0,0.5] are 'low' and emotions (0.5,1) are 'high'.
(:functions
	(power ?r - robot)
)

(:constants
	battery oxygen - type
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(holding ?v - robot ?o - object)
	(gripper_empty ?v - robot)
	(not_busy)
	(is_of_type ?o - object ?t - type)
	(child_has_oxygen ?c - child)
	(battery_available ?o - object)
	(explored ?wp - waypoint)

	;; Tidy room predicates, we only assume there is a single area.
	(explored_room)
	(examined_room)
	(tidy_room)
)

(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration 10)
	:condition (and
		(over all (> (power ?v) 0))
		(at start (robot_at ?v ?from))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to))
		(at start (decrease (power ?v) (* ?duration 0.05)))
	)
)

(:durative-action pickup_object
	:parameters (?v - robot ?robot_wp ?object_wp - waypoint ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (robot_at ?v ?robot_wp))
		(at start (object_at ?o ?object_wp))
		(over all (> (power ?v) 0))
		(at start (not_busy))
		(over all (= ?robot_wp ?object_wp))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (gripper_empty ?v)))
		(at end (holding ?v ?o))
		(at start (not (object_at ?o ?object_wp)))
		(at start (decrease (power ?v) (* ?duration 0.05)))
	)
)

(:durative-action persuade-child-give-battery
	:parameters (?v - robot ?c - child ?o - object ?wp - waypoint)
	:duration (= ?duration 300)
	:condition (and
		(at start (object_at ?o ?wp))
		(over all (> (power ?v) 0))
		(over all (is_of_type ?o battery))
		(at start (not_busy))
		(over all (child_has_oxygen ?c))
		(at start (battery_available ?o))
	)
	:effect (and 
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (battery_available ?o)))
		(at end (increase (power ?v) 5))
	)
)

(:durative-action give_object
	:parameters (?v - robot ?wp - waypoint ?c - child ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (holding ?v ?o))

		(over all (> (power ?v) 0))
		(over all (is_of_type ?o oxygen))
		(at start (not_busy))
		(over all (robot_at ?v ?wp))
	)
	:effect (and 
		(at end (gripper_empty ?v))
		(at end (not (holding ?v ?o)))
		(at end (child_has_oxygen ?c))
		(at start (not (not_busy)))		
		(at end (not_busy))
		(at start (decrease (power ?v) (* ?duration 0.01)))
	)
)

;; Tidy room actions.

(:action explore_area
	:parameters ()
	:precondition (and
		()
	)
	:effect (and
		(explored_room)
	)
)

(:action examine_area
	:parameters ()
	:precondition (and
		(explored_room)
	)
	:effect (and
		(examined_room)
	)
)

(:action tidy_area
	:parameters ()
	:precondition (and
		(examined_room)
	)
	:effect (and
		(tidy_room)
	)
)
)

