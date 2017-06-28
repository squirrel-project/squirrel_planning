(define (domain find_key)
(:requirements :typing :conditional-effects :negative-preconditions :disjunctive-preconditions)

(:types
	waypoint robot object box type child
	area
	state
	wiggle
	sound
	jump_point
)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(object_at ?o - object ?wp - waypoint)
	(box_at ?b - box ?wp - waypoint)
	(gripper_empty ?v - robot)
	(holding ?v - robot ?o - object)
	(child_is_holding ?c - child ?o - object)
	(is_not_occupied ?wp - waypoint)
	(tidy ?o - object)
	(tidy_location ?t - type ?wp - waypoint)
	(push_location ?o - object ?wp - waypoint)
	(can_push ?v - robot ?t - type)
	(can_fit_inside ?t - type ?b - box)
	(inside ?o - object ?b - box)
	(examined ?o - object)
	(explored ?o - object)
	(child_at ?c - child ?wp - waypoint)
	(connected ?from ?to - waypoint)
	(is_of_type ?o - object ?t -type ?s - state)
	(is_examined ?o - object ?s - state)
	(part-of ?s - state ?kb - knowledgebase)
	(current_kb ?kb - knowledgebase)
	(parent ?kb ?kb2 - knowledgebase)
	(has_checked_type ?o - object ?t - type)
	(resolve-axioms)
	(sorting_done ?s - state)
	(toy_at_right_box ?s - state)
	(belongs_in ?o - object ?b - box ?s - state)
)

(:action goto_view_waypoint
	:parameters ()
	:precondition (and

	)
	:effect (and

	)
)

(:action explore_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(explored ?a)
	)
	:effect (and
		(explored ?a)
	)
)

(:action examine_object
	:parameters ()
	:precondition (and

	)
	:effect (and

	)
)

(:action examine_area
	:parameters (?v - robot ?a - area)
	:precondition (and
		(robot_in ?v ?a)
		(explored ?a)
	)
	:effect (and
		(examined ?a)
	)
)

(:action FOLLOW_CHILD
	:parameters(?c - child)
	:precondition (and

	)
	:effect (and

	)
)

(:action DETECT_CHILDREN
	:parameters()
	:precondition (and
		
	)
	:effect (and
		
	)
)

(:action DETECT_RESPONSE
	:parameters()
	:precondition (and
		
	)
	:effect (and
		
	)
)

(:action START_PHASE3
	:parameters()
	:precondition (and
		
	)
	:effect (and
		
	)
)

(:action jump
	:parameters(?n - jump_point)
	:precondition (and
		
	)
	:effect (and
		
	)
)

(:action emote
	:parameters (?v - robot ?s - sound ?w - wiggle)
	:precondition (and
		
	)
	:effect (and
		
	)
)

(:action give_object
	:parameters (?v - robot ?wp - waypoint ?o - object ?c - child)
	:precondition (and
		(child_at ?c ?wp)
		(robot_at ?v ?wp)
		(holding ?v ?o)
	)
	:effect (and
			(not (holding ?v ?o))
			(gripper_empty ?v)
			(child_is_holding ?c ?o)
		)
)

(:action take_object
	:parameters (?v - robot ?wp - waypoint ?o - object ?c - child)
	:precondition (and
		(connected ?v_wp ?c_wp)
		(child_at ?c ?c_wp)
		(robot_at ?v ?v_wp)
		(child_is_holding ?c ?o)
		(gripper_empty ?v)
	)
	:effect (and
			(not (child_is_holding ?c ?o))
			(not (gripper_empty ?v))
			(holding ?v ?o)
	)
)

(:action examine_object_in_hand
	:parameters (?v - robot ?o - object)
	:precondition (and
		(holding ?v ?o)
	)
	:effect (and 
	)
)

(:action put_object_in_box
	:parameters (?v - robot ?wp ?wp2 - waypoint ?o1 - object ?b - box ?t - type)
	:precondition (and
		(box_at ?b ?wp)
		(connected ?wp ?wp2)
		(robot_at ?v ?wp2)
		(holding ?v ?o1)
		(can_fit_inside ?t ?b)
	;;(is_of_type ?o1 ?t)
	)
	:effect (and
			(not (holding ?v ?o1))
			(gripper_empty ?v)
			(inside ?o1 ?b)
		)
	)

(:action pickup_object
	:parameters (?v - robot ?wp ?wp2 - waypoint ?o - object ?t - type)
	:precondition (and
		(connected ?wp ?wp2)
		(robot_at ?v ?wp2)
		(object_at ?o ?wp)
		(gripper_empty ?v)
		;;(is_of_type ?o ?t)
	)
	:effect (and
			(not (gripper_empty ?v))
			(not (object_at ?o ?wp))
			(is_not_occupied ?wp)
			(holding ?v ?o)
		)

	)

(:action drop_object
	:parameters (?v - robot ?wp ?wp2 - waypoint ?o - object)
	:precondition (and
		(connected ?wp ?wp2)
		(robot_at ?v ?wp)
		(holding ?v ?o)
		(is_not_occupied ?wp2)
	)
	:effect (and
			(not (holding ?v ?o))
			(not (is_not_occupied ?wp2))
			(gripper_empty ?v)
			(object_at ?o ?wp2)
		)
	)

(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and
		(connected ?from ?to)
		(robot_at ?v ?from)
		(is_not_occupied ?to)
	)
	:effect (and
			(not (robot_at ?v ?from))
			(not (is_not_occupied ?to))
			(robot_at ?v ?to)
			(is_not_occupied ?from)
		)
	)

(:action look_at_object
	:parameters (?v - robot ?view ?from - waypoint ?o - object)
	:precondition (and

	)
	:effect (and

	)
)

(:action push_object
	:parameters (?v - robot ?ob - object ?t - type ?from ?to ?obw - waypoint)
	:precondition (and
		(robot_at ?v ?from)
		(object_at ?ob ?obw)
		(push_location ?ob ?from)
		;;(is_of_type ?ob ?t)
	)
	:effect (and
		
			(not (robot_at ?v ?from))
			(not (object_at ?ob ?from))
			(robot_at ?v ?to)
			(object_at ?ob ?to)
		)
	)

(:action tidy_object
	:parameters (?v - robot ?o - object ?b - box ?t - type)
	:precondition (and
		;;(is_of_type ?o ?t)
		(inside ?o ?b)
		(can_fit_inside ?t ?b)
	)
	:effect (and
			(tidy ?o)
		)
	)

;; Sense the type of object.
(:action observe-is_of_type
	:parameters (?o - object ?t - type ?v - robot ?wp ?wp2 - waypoint ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		(connected ?wp ?wp2)
		(robot_at ?v ?wp)
		(object_at ?o ?wp2)
		(examined ?o)
	)
	:effect (and
		
	)
)

(:action examine_objects
	:parameters ()
	:precondition (and

	)
	:effect (and
		
	)
)

(:action observe-belongs_in
	:parameters (?o - object ?b - box ?wp ?wp2 - waypoint ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and

	)
	:effect (and
		
	)
)

(:action observe-sorting_done
	:parameters (?l ?l2 - level ?kb - knowledgebase)
	:precondition (and

	)
	:effect (and
		
	)
)

(:action observe-is_examined
	:parameters (?o - object ?l ?l2 - level ?kb - knowledgebase)
	:precondition (and

	)
	:effect (and
		
	)
)

(:action observe-toy_at_right_box
	:parameters (?l ?l2 - level ?kb - knowledgebase)
	:precondition (and
		
	)
	:effect (and
		
	)
)

;; Exit the current branch.
(:action pop
	:parameters (?l ?l2 - level)
	:precondition (and
		
	)
	:effect (and 
		
	)
)

;; Resolve the axioms manually.
(:action ramificate
	:parameters ()
	:precondition ()
	:effect (and 
		
	)
)

;; Move 'down' into the knowledge base.
(:action assume_knowledge
	:parameters (?old_kb ?new_kb - knowledgebase)
	:precondition (and
		
	)
	:effect (and
		
	)
)

;; Move 'up' into the knowledge base.
(:action shed_knowledge
	:parameters (?old_kb ?new_kb - knowledgebase)
	:precondition (and
		
	)
	:effect (and

	)
)
)
