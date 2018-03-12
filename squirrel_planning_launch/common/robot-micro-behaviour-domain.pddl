(define (domain squirrel_robot_behaviour)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :duration-inequalities :conditional-effects :numeric-fluents)

(:types
	robot
	child
	waypoint
	object
	type
	wiggle
	sound
)

;; Emotions range from [-1,1]. None of the emotions are alowed to fall above 0.
;; Emotions in the range (0,0.5] are 'low' and emotions (0.5,1) are 'high'.
(:functions
	(power ?r - robot)
	(reciprocal ?c - child)
)

(:constants
	battery oxygen misc - type
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(holding ?v - robot ?o - object)
	(gripper_empty ?v - robot)
	(not_busy)
	(flashing_lights)
	(not_flashing_lights)
	(not_gazing)
	(playing_sound)
	(not_playing_sound)
	(is_of_type ?o - object ?t - type)
	(child_has_oxygen ?c - child)
	(has_battery)
	;(child_knows_what_robot_wants ?c - child)
	(child_knows_what_robot_wants ?c - child)
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

(:durative-action pickup
	:parameters (?v - robot ?o - object ?wp - waypoint)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(over all (> (power ?v) 0))
		(at start (not_busy))
		(over all (is_of_type ?o misc))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (gripper_empty ?v)))
		(at end (holding ?v ?o))
		(at start (not (object_at ?o ?wp)))
		(at start (decrease (power ?v) (* ?duration 0.05)))
	)
)

;; Mini behaviours.
;; - lights
;; - sounds
;; - movement
(:durative-action show_lights
	:parameters (?v - robot ?c - child)
	:duration (= ?duration 60)
	:condition (and 
		(at start (not_flashing_lights))
		(at start (playing_sound))
	)
	:effect (and
		(at start (flashing_lights))
		(at start (not (not_flashing_lights)))
		(at end (not (flashing_lights)))
		(at end (not_flashing_lights))
		(at end (increase (reciprocal ?c) 0.1))
	)
)

(:durative-action gaze
	:parameters (?v - robot ?c - child ?o - object)
	:duration (= ?duration 12)
	:condition (and
		(at start (not (child_knows_what_robot_wants ?c)))
		(at start (not_gazing))
	)
	:effect (and
		(at start (not (not_gazing)))
		(at end (not_gazing))
		(at end (increase (reciprocal ?c) 0.3))
		(at end (child_knows_what_robot_wants ?c))
		;(at start (not (not_child_knows_what_robot_wants ?c)))
	)
)

(:durative-action emote
	:parameters (?v - robot ?c - child ?s - sound ?w - wiggle)
	:duration (= ?duration 50)
	:condition (and
		(at start (not_playing_sound))
	)
	:effect (and
		(at end (increase (reciprocal ?c) 0.1))
		(at start (playing_sound))
		(at start (not (not_playing_sound)))
		(at end (not (playing_sound)))
		(at end (not_playing_sound))
	)
)

(:durative-action give-object
	:parameters (?v - robot ?c - child ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (holding ?v ?o))

		(over all (> (power ?v) 0))
		(at start (not_busy))
	)
	:effect (and 
		(at end (gripper_empty ?v))
		(at end (not (holding ?v ?o)))
		(at start (not (not_busy)))		
		(at end (not_busy))
		(at start (decrease (power ?v) (* ?duration 0.01)))
		(at end (increase (reciprocal ?c) 0.5))
	)
)

(:durative-action request-battery
	:parameters (?v - robot ?c - child ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (is_of_type ?o battery))

		(over all (> (power ?v) 0))
		(at start (> (reciprocal ?c) 1))
		(at start (not_busy))
		(at start (child_knows_what_robot_wants ?c))
	)
	:effect (and 
		(at end (not (gripper_empty ?v)))
		(at end (holding ?v ?o))
		(at start (not (not_busy)))		
		(at end (not_busy))
		(at end (has_battery))
	)
)
)

