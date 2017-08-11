(define (problem squirrel_emotion_problem)
(:domain squirrel_emotion)
(:objects
	toy1 toy2 toy3 - object
	box1 - box
	kenny - robot
	kenny_wp toy1_wp toy2_wp toy3_wp box1_wp - waypoint
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

	(= (pleasure c1) 0.4)
	(= (arousal c1) 0.4)
	(= (dominance c1) 0.45)
	(= (reciprocal c1) 0.45)

	(= (pleasure c2) 1.0)
	(= (arousal c2) 1.0)
	(= (dominance c2) 1.0)
	(= (reciprocal c2) 0.0)

	(= (pleasure c3) 0.83)
	(= (arousal c3) 0.98)
	(= (dominance c3) 0.6)
	(= (reciprocal c3) 0.0)
)
(:goal (and
	(classified toy1)
	(classified toy2)
	(classified toy3)
	(in_box box1 toy1)
	(in_box box1 toy2)
	(in_box box1 toy3)
	(> (pleasure c1) 0)
	(> (pleasure c2) 0)
	(> (pleasure c3) 0)
	(> (arousal c1) 0)
	(> (arousal c2) 0)
	(> (arousal c3) 0)
	(> (dominance c1) 0)
	(> (dominance c2) 0)
	(> (dominance c3) 0)
)))

