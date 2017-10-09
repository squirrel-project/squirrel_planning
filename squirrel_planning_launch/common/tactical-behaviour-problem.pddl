(define (problem squirrel_behaviour_problem)
(:domain squirrel_behaviour)
(:objects
	toy1 toy2 toy3 - object
	box1 - box
	kenny - robot
	kenny_wp toy1_wp toy2_wp toy3_wp box1_wp - waypoint
	wiggle_error wiggle_no - wiggle
	sound_NEUTRAL sound_CHEERING - sound
)
(:init
	(not_busy)
	(robot_at kenny kenny_wp)
	(box_at box1 box1_wp)
	(object_at toy1 toy1_wp)
	(object_at toy2 toy2_wp)
	(object_at toy3 toy3_wp)
	(reachable toy1)
	(reachable toy2)
	(gripper_empty kenny)
	(not_playing_sound)
	(not_flashing_lights)

	(= (pleasure c1) 0.4)
	(= (arousal c1) 0.4)
	(= (dominance c1) 0.45)
	(= (reciprocal c1) 0.45)
	(= (attentiveness c1) 0.1)

	(= (pleasure c2) 1.0)
	(= (arousal c2) 1.0)
	(= (dominance c2) 1.0)
	(= (reciprocal c2) 0.0)
	(= (attentiveness c2) -0.2)

	(= (pleasure c3) 0.83)
	(= (arousal c3) 0.98)
	(= (dominance c3) 0.6)
	(= (reciprocal c3) 0.0)
	(= (attentiveness c3) 0.3)
)
(:goal (and
	(> (reciprocal c1) 0.5)
	(> (reciprocal c2) 0.5)
	(> (reciprocal c3) 0.5)
)))

