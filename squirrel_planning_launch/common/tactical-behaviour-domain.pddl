;; Need to make sure that the goal of any of these problems to get to a state where 
;; the rest of the plan is still valid. So it can move around objects, but only if 
;; they are put back to their rightfull place or if these objects have no significance 
;; to us.

(define (domain squirrel_behaviour)

(:requirements :strips :typing :disjunctive-preconditions :negative-preconditions :durative-actions :duration-inequalities :conditional-effects :numeric-fluents)

(:types
	robot
	child
	waypoint
	box
	object
)

;; Emotions range from [-1,1]. None of the emotions are alowed to fall above 0.
;; Emotions in the range (0,0.5] are 'low' and emotions (0.5,1) are 'high'.
(:functions
	(pleasure ?c - child)
	(arousal ?c - child)
	(dominance ?c - child)
	(reciprocal ?c - child)
	(attentiveness ?c - child)
)

(:constants
	c1 c2 c3 - child
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(box_at ?b - box ?wp - waypoint)
	(classified ?o - object)
	(in_box ?b - box ?o - object)
	(holding ?v - robot ?o - object)
	(gripper_empty ?v - robot)
	(not_busy)
	(reachable ?o - object)
	(child_holding ?c - child ?o - object)
	(playing_sound)
	(not_playing_sound)
	(flashing_lights)
	(not_flashing_lights)
)

;; Mini behaviours.
(:durative-action flash-lights-w-sound
	:parameters (?v - robot ?c - child )
	:duration (= ?duration 5)
	:condition (and 
		(at start (not_flashing_lights))
		(at start (<= (attentiveness ?c) 0.1))
		(at start (playing_sound))
	)
	:effect (and
		(at start (flashing_lights))
		(at start (not (not_flashing_lights)))
		(at end (not (flashing_lights)))
		(at end (not_flashing_lights))
		(at end (increase (attentiveness ?c) 0.3))
	)
)

(:durative-action flash-lights
	:parameters (?v - robot ?c - child )
	:duration (= ?duration 5)
	:condition (and 
		(at start (not_flashing_lights))
		(at start (<= (attentiveness ?c) 0.1))
		(at start (not_playing_sound))
	)
	:effect (and
		(at start (flashing_lights))
		(at start (not (not_flashing_lights)))
		(at end (not (flashing_lights)))
		(at end (not_flashing_lights))
		(at end (increase (attentiveness ?c) 0.1))
	)
)

(:durative-action play-sound
	:parameters (?v - robot ?c - child)
	:duration (= ?duration 5)
	:condition (and
		(at start (> (attentiveness ?c) -0.1))
		(at start (< (attentiveness ?c) 0.5))
		(at start (not_playing_sound))
	)
	:effect (and
		(at end (increase (attentiveness ?c) 0.1))
		(at start (playing_sound))
		(at start (not (not_playing_sound)))
		(at end (not (playing_sound)))
		(at end (not_playing_sound))
	)
)

(:durative-action move-towards
	:parameters (?v - robot ?c - child)
	:duration (= ?duration 15)
	:condition (and
		(at start (> (attentiveness ?c) 0.4))
	)
	:effect (and
		(at end (increase (attentiveness ?c) 0.2))
	)
)

(:durative-action gaze
	:parameters (?v - robot ?c - child)
	:duration (= ?duration 12)
	:condition (and
		(at start (> (attentiveness ?c) 0.4))
	)
	:effect (and
		(at end (increase (attentiveness ?c) 0.1))
	)
)

(:durative-action move
	:parameters (?v - robot ?from ?to - waypoint)
	:duration (= ?duration 10)
	:condition (and
		(over all (> (pleasure c1) 0))
		(over all (> (pleasure c2) 0))
		(over all (> (pleasure c3) 0))
		(over all (> (arousal c1) 0))
		(over all (> (arousal c2) 0))
		(over all (> (arousal c3) 0))
		(over all (> (dominance c1) 0))
		(over all (> (dominance c2) 0))
		(over all (> (dominance c3) 0))
		(at start (robot_at ?v ?from))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (robot_at ?v ?from)))
		(at end (robot_at ?v ?to))
		(forall (?c - child)
			(at start (decrease (pleasure ?c) (* ?duration 0.0045)))
		)
	)
)

(:durative-action classify
	:parameters (?v - robot ?o - object ?wp - waypoint)
	:duration (= ?duration 60)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(over all (> (pleasure c1) 0))
		(over all (> (pleasure c2) 0))
		(over all (> (pleasure c3) 0))
		(over all (> (arousal c1) 0))
		(over all (> (arousal c2) 0))
		(over all (> (arousal c3) 0))
		(over all (> (dominance c1) 0))
		(over all (> (dominance c2) 0))
		(over all (> (dominance c3) 0))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (classified ?o))
		(forall (?c - child)
			(at end (decrease (pleasure ?c) (* ?duration 0.005)))
			;;(at end (decrease (bored ?c) 2))
		)
	)
)

(:durative-action pickup
	:parameters (?v - robot ?o - object ?wp - waypoint)
	:duration (= ?duration 60)
	:condition (and
		(at start (gripper_empty ?v))
		(over all (robot_at ?v ?wp))
		(at start (object_at ?o ?wp))
		(over all (> (pleasure c1) 0))
		(over all (> (pleasure c2) 0))
		(over all (> (pleasure c3) 0))
		(over all (> (arousal c1) 0))
		(over all (> (arousal c2) 0))
		(over all (> (arousal c3) 0))
		(over all (> (dominance c1) 0))
		(over all (> (dominance c2) 0))
		(over all (> (dominance c3) 0))
		(over all (reachable ?o))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (gripper_empty ?v)))
		(at end (holding ?v ?o))
		(at start (not (object_at ?o ?wp)))
		(forall (?c - child)
			(at end (decrease (pleasure ?c) (* ?duration 0.002)))
		)
	)
)

(:durative-action robot-gives-object-to-child
	:parameters (?v - robot ?c - child ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (holding ?v ?o))

		(over all (> (pleasure c1) 0))
		(over all (> (pleasure c2) 0))
		(over all (> (pleasure c3) 0))
		(over all (> (arousal c1) 0))
		(over all (> (arousal c2) 0))
		(over all (> (arousal c3) 0))
		(over all (> (dominance c1) 0))
		(over all (> (dominance c2) 0))
		(over all (> (dominance c3) 0))
		(at start (not_busy))
		(at start (> (attentiveness ?c) 0.5))
	)
	:effect (and 
		(at end (gripper_empty ?v))
		(at end (not (holding ?v ?o)))
		(at end (child_holding ?c ?o))
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (reciprocal ?c) 0.5))
	)
)
)

