#include <ros/ros.h>

#include "pddl_actions/GotoPDDLAction.h"
#include "pddl_actions/ExploreWaypointPDDLAction.h"
#include "pddl_actions/ChildrenPDDLAction.h"
#include "pddl_actions/ClearObjectPDDLAction.h"
#include "pddl_actions/ClassifyObjectPDDLAction.h"
#include "pddl_actions/EmotePDDLAction.h"
#include "pddl_actions/PutObjectInBoxPDDLAction.h"
#include "pddl_actions/TidyObjectPDDLAction.h"
#include "pddl_actions/PickupPDDLAction.h"
#include "pddl_actions/PushObjectPDDLAction.h"
#include "pddl_actions/DropObjectPDDLAction.h"
#include "pddl_actions/GiveObjectPDDLAction.h"
#include "pddl_actions/TakeObjectPDDLAction.h"
#include "pddl_actions/SimulatedObservePDDLAction.h"
#include "pddl_actions/ExamineObjectInHandPDDLAction.h"
#include "pddl_actions/FollowChildPDDLAction.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "rosplan_interface_SimluatedPDDLActionsNode");
	ros::NodeHandle nh("~");

	bool goto_waypoint,
		explore_waypoint,
		clear_object,
		classify_object,
		emote,
		put_object_in_box,
		pickup_object,
		drop_object,
		give_object,
		take_object,
		examine_object_in_hand,
		follow_child;

	nh.getParam("simulate_goto_waypoint", goto_waypoint);
	nh.getParam("simulate_explore_waypoint", explore_waypoint);
	nh.getParam("simulate_clear_object", clear_object);
	nh.getParam("simulate_classify_object", classify_object);
	nh.getParam("simulate_emote", emote);
	nh.getParam("simulate_put_object_in_box", put_object_in_box);
	nh.getParam("simulate_pickup_object", pickup_object);
	nh.getParam("simulate_drop_object", drop_object);
	nh.getParam("simulate_give_object", give_object);
	nh.getParam("simulate_take_object", take_object);
	nh.getParam("simulate_examine_object_in_hand", examine_object_in_hand);
	nh.getParam("simulate_follow_child", follow_child);

	KCL_rosplan::GotoPDDLAction* goto_action;
	KCL_rosplan::ExploreWaypointPDDLAction* explore_waypoint_action;
	KCL_rosplan::ClearObjectPDDLAction* clear_object_action;
	KCL_rosplan::ClassifyObjectPDDLAction* classify_object_action;
	KCL_rosplan::EmotePDDLAction* emote_action;
	KCL_rosplan::PutObjectInBoxPDDLAction* put_object_in_box_action;
	KCL_rosplan::PickupPDDLAction* pickup_action;
	KCL_rosplan::DropObjectPDDLAction* drop_object_action;
	KCL_rosplan::GiveObjectPDDLAction* give_object_action;
	KCL_rosplan::TakeObjectPDDLAction* take_object_action;
	KCL_rosplan::ChildrenPDDLAction* children_action;
	KCL_rosplan::ExamineObjectInHandPDDLAction* examine_object_in_hand_action;
	KCL_rosplan::SimulatedObservePDDLAction observe_actions(nh);
	KCL_rosplan::FollowChildPDDLAction* follow_child_action;

	// Setup all the simulated actions.
	if(goto_waypoint) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: goto_waypoint");
		goto_action = new KCL_rosplan::GotoPDDLAction(nh);
	}

	if(explore_waypoint) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: explore_waypoint");
		explore_waypoint_action = new KCL_rosplan::ExploreWaypointPDDLAction(nh);
	}
	if(clear_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: clear_object");
		clear_object_action = new KCL_rosplan::ClearObjectPDDLAction(nh);
	}
	if(classify_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: classify_object");
		classify_object_action = new KCL_rosplan::ClassifyObjectPDDLAction(nh, 0.5f);
	}
	if (emote) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: emote");
		emote_action = new KCL_rosplan::EmotePDDLAction(nh);
	}
	if(put_object_in_box) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: put_object_in_box");
		put_object_in_box_action = new KCL_rosplan::PutObjectInBoxPDDLAction(nh);
	}
	if(pickup_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: pickup_object");
		pickup_action = new KCL_rosplan::PickupPDDLAction(nh);
	}	
	if(drop_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: drop_object");
		drop_object_action = new KCL_rosplan::DropObjectPDDLAction(nh);
	}
	if(give_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: give_object");
		give_object_action = new KCL_rosplan::GiveObjectPDDLAction(nh);
	}
	if(take_object) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: take_object");
		take_object_action = new KCL_rosplan::TakeObjectPDDLAction(nh);
	}
	if (examine_object_in_hand) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: examine_object_in_hand");
		examine_object_in_hand_action = new KCL_rosplan::ExamineObjectInHandPDDLAction(nh, 0.0f);
	}
	if (follow_child) {
		ROS_INFO("KCL: (SimulatedPDDLActionsNode) Simulating: follow_child");
		follow_child_action = new KCL_rosplan::FollowChildPDDLAction(nh);
	}
		
	children_action = new KCL_rosplan::ChildrenPDDLAction(nh);

	KCL_rosplan::TidyObjectPDDLAction tidy_object_action(nh);
	// KCL_rosplan::PushObjectPDDLAction push_object_action;

	ROS_INFO("KCL: (SimulatedPDDLActionsNode) All simulated actions are ready to receive.");
	
	ros::spin();
	return 0;
}

