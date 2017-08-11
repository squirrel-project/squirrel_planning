(define (domain squirrel_emotion)

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
)

(:durative-action accomodate-distress
	:parameters (?c - child)
	:duration (<= ?duration 30)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(over all (< (arousal ?c) 0))
		(at start (< (pleasure ?c) 0.5))
		(at start (> (arousal ?c) 0.5))
		(at start (> (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.01)))
		(at end (decrease (arousal ?c) (* ?duration 0.02)))
	)
)


(:durative-action improve-distress
	:parameters (?c - child)
	:duration (<= ?duration 30)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(over all (> (arousal ?c) 0))
		(at start (< (pleasure ?c) 0.5))
		(at start (> (arousal ?c) 0.5))
		(at start (> (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.02)))
		(at end (decrease (arousal ?c) (* ?duration 0.02)))
	)
)

(:durative-action accomodate-sadness
	:parameters (?c - child)
	:duration (<= ?duration 10)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(at start (< (pleasure ?c) 0.5))
		(at start (< (arousal ?c) 0.5))
		(at start (> (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.01)))
	)
)

(:durative-action improve-sadness
	:parameters (?c - child)
	:duration (<= ?duration 10)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(at start (< (pleasure ?c) 0.5))
		(at start (< (arousal ?c) 0.5))
		(at start (> (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.02)))
	)
)

(:durative-action improve-boredom
	:parameters (?c - child)
	:duration (<= ?duration 10)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(over all (< (arousal ?c) 1))
		(over all (< (dominance ?c) 1))
		(at start (< (pleasure ?c) 0.5))
		(at start (< (arousal ?c) 0.5))
		(at start (< (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.02)))
		(at end (increase (arousal ?c) (* ?duration 0.02)))
		(at end (increase (dominance ?c) (* ?duration 0.02)))
	)
)

(:durative-action maintain-happyness
	:parameters (?c - child)
	:duration (<= ?duration 60)
	:condition (and
		(over all (> (arousal ?c) 0))
		(at start (> (pleasure ?c) 0.5))
		(at start (> (arousal ?c) 0.5))
		(at start (> (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (decrease (arousal ?c) (* ?duration 0.01)))
	)
)

(:durative-action improve-introvert
	:parameters (?c - child)
	:duration (<= ?duration 30)
	:condition (and
		(over all (< (pleasure ?c) 1))
		(over all (< (arousal ?c) 1))
		(over all (< (dominance ?c) 1))
		(at start (> (pleasure ?c) 0.5))
		(at start (< (arousal ?c) 0.5))
		(at start (< (dominance ?c) 0.5))
		(at start (not_busy))
	)
	:effect (and
		(at start (not (not_busy)))
		(at end (not_busy))
		(at end (increase (pleasure ?c) (* ?duration 0.02)))
		(at end (increase (arousal ?c) (* ?duration 0.02)))
		(at end (increase (dominance ?c) (* ?duration 0.02)))
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

(:durative-action reciprocal-behaviour
	:parameters (?v - robot ?c - child)
	:duration (= ?duration 60)
	:condition (and
		(at start (< (reciprocal ?c) 0.5))
	)
	:effect (and 
		(at end (increase (reciprocal ?c) 0.5))
	)
)

(:durative-action child-give-object-to-robot
	:parameters (?v - robot ?c - child ?o - object)
	:duration (= ?duration 60)
	:condition (and
		(at start (child_holding ?c ?o))
		(at start (gripper_empty ?v))

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
		(at end (not (gripper_empty ?v)))
		(at end (holding ?v ?o))
		(at end (not (child_holding ?c ?o)))
		(at start (not (not_busy)))
		(at end (not_busy))
	)
)

(:durative-action child-pickup
	:parameters (?v - robot ?o - object ?wp - waypoint ?c - child)
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
		(at start (> (reciprocal ?c) 0.5))
	)
	:effect (and
		(at end (child_holding ?c ?o))
		(at start (not (not_busy)))
		(at end (not_busy))
		(at start (not (object_at ?o ?wp)))
		(forall (?c - child)
			(at end (increase (pleasure ?c) (* ?duration 0.002)))
		)
		(at end (decrease (reciprocal ?c) 0.5))
	)
)

(:durative-action tidy
	:parameters (?v - robot ?o - object ?b - box ?wp - waypoint)
	:duration (= ?duration 30)
	:condition (and
		(over all (robot_at ?v ?wp))
		(at start (holding ?v ?o))
		(over all (box_at ?b ?wp))
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
		(at start (not (holding ?v ?o)))
		(at end (gripper_empty ?v))
		(at end (in_box ?b ?o))
		(forall (?c - child)
			(at end (decrease (pleasure ?c) (* ?duration 0.0042)))
		)
	)
)

)

