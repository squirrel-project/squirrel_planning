(define (domain squirrel_robot_behaviour)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :duration-inequalities :conditional-effects :numeric-fluents :equality)

(:types
	robot
	child
	waypoint
	object
	type
	box
)

;; Emotions range from [-1,1]. None of the emotions are alowed to fall above 0.
;; Emotions in the range (0,0.5] are 'low' and emotions (0.5,1) are 'high'.
(:functions
	(power ?r - robot)
	(distance ?wp1 ?wp2 - waypoint)
	(frustration ?r - robot)
)

(:constants
	battery oxygen - type
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(box_at ?b - box ?wp - waypoint)
	(holding ?v - robot ?o - object)
	(gripper_empty ?v - robot)
	(not_busy)
	(is_of_type ?o - object ?t - type)
	(child_has_oxygen ?c - child)
	(battery_available ?o - object)
	(explored ?wp - waypoint)
	(belongs_in ?o - object ?b - box)
	(in_box ?o - object ?b - box)
	(near ?wp1 ?wp2 - waypoint)
	(examined ?o - object)
	(observed-from ?o - object ?wp - waypoint)

	;; Tidy room predicates, we only assume there is a single area.
	;(explored_room)
)

(:durative-action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration (distance ?from ?to))
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


(:durative-action ask_child_to_give
	:parameters (?v - robot ?robot_wp ?object_wp - waypoint ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (robot_at ?v ?robot_wp))
		(at start (object_at ?o ?object_wp))
		(over all (> (power ?v) 0))
		(at start (not_busy))
		(over all (near ?robot_wp ?object_wp))
		(over all (< (frustration ?v) 0))
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


(:durative-action pickup_object
	:parameters (?v - robot ?robot_wp ?object_wp - waypoint ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (robot_at ?v ?robot_wp))
		(at start (object_at ?o ?object_wp))
		(over all (> (power ?v) 0))
		(at start (not_busy))
		(over all (near ?robot_wp ?object_wp))
		(over all (>= (frustration ?v) 0))
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

;; This is a meta action, the class that deals with this action determines whether it is necessary to perform this action
;; if ?o is already examined then there is no need to examine this object.
(:durative-action attempt_to_examine_object
	:parameters (?v - robot ?view ?from - waypoint ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(over all (> (power ?v) 0))
		(over all (near ?from ?view))
;;		(over all (robot_at ?v ?from))
		(over all (object_at ?o ?view))
		(at start (not_busy))
	)
	:effect (and 
		(at end (observed-from ?o ?from))
		(at start (decrease (power ?v) (* ?duration 0.01)))
		(at start (not (not_busy)))
		(at end (not_busy))
	)
)

(:durative-action put_object_in_box
    :parameters (?v - robot ?wp ?near_wp - waypoint ?o - object ?b - box)
	:duration (= ?duration 60)
    :condition (and"
        (over all (box_at ?b ?wp))
        (over all (robot_at ?v ?near_wp))
        (over all (near ?near_wp ?wp))
        (at start (holding ?v ?o))
        (over all (belongs_in ?o ?b))
		(at start (not_busy))
    )
    :effect (and
        (at end (not (holding ?v ?o)))
        (at end (gripper_empty ?v))
        (at end (in_box ?o ?b))
		(at start (not (not_busy)))
		(at end (not_busy))
    )
)

;; Use perception actions to search for objects at the current waypoint
(:durative-action explore_waypoint
	:parameters (?v - robot ?wp - waypoint)
	:duration (= ?duration 60)
	:condition (and
		(over all (robot_at ?v ?wp))
	)
	:effect (and
		(at end (explored ?wp))
	)
)
)

