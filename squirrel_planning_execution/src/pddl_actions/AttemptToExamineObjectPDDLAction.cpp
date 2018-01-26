#include <sstream>
#include <complex>

#include <cstdlib>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <std_srvs/Empty.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <squirrel_planning_msgs/CallAction.h>

#include <diagnostic_msgs/KeyValue.h>

#include "AttemptToExamineObjectPDDLAction.h"
#include <squirrel_planning_execution/KnowledgeBase.h>

namespace KCL_rosplan
{

AttemptToExamineObjectPDDLAction::AttemptToExamineObjectPDDLAction(ros::NodeHandle& node_handle, KnowledgeBase& knowledge_base, mongodb_store::MessageStoreProxy& message_store)
	: knowledge_base_(&knowledge_base), action_client_("/move_base", true), message_store_(&message_store)
{
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::AttemptToExamineObjectPDDLAction::dispatchCallback, this);
	
	clear_costmaps_client = node_handle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
	call_examine_action_client_ = node_handle.serviceClient<squirrel_planning_msgs::CallAction>("/perception_action_examine_action");
}

AttemptToExamineObjectPDDLAction::~AttemptToExamineObjectPDDLAction()
{
	
}

bool AttemptToExamineObjectPDDLAction::moveTo(const std::string& wp)
{
	// get pose from message store
	std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
	if(message_store_->queryNamed<geometry_msgs::PoseStamped>(wp, results)) {
		if(results.size()<1) {
			ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) aborting action dispatch; no matching wpID %s.", wp.c_str());
			return false;
		}
		if(results.size()>1)
			ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) multiple waypoints share the same wpID.");

		ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) waiting for move_base action server to start.");
		action_client_.waitForServer();

		// dispatch MoveBase action
		move_base_msgs::MoveBaseGoal goal;
		geometry_msgs::PoseStamped &pose = *results[0];
		
		std::cout << "Send the pose: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ") to movebase for waypoint: " << wp << "." << std::endl;
		
		goal.target_pose = pose;
		action_client_.sendGoal(goal);

		bool finished_before_timeout = action_client_.waitForResult();
		if (finished_before_timeout) {

			actionlib::SimpleClientGoalState state = action_client_.getState();
			ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) action finished: %s.", state.toString().c_str());

			if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				// publish feedback (achieved)
				return true;
			} else {
				// clear costmaps
				std_srvs::Empty emptySrv;
				clear_costmaps_client.call(emptySrv);
				// publish feedback (failed)
				return false;
			}
		} else {
			// timed out (failed)
			action_client_.cancelAllGoals();
			ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) action timed out.");
			return false;
		}
	} else {
		// no KMS connection (failed)
		ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) aborting action dispatch; query to sceneDB failed; wp %s.", wp.c_str());
		return false;
	}
}

void AttemptToExamineObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "attempt_to_examine_object" || msg->parameters.size() != 4)
	{
		return;
	}
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& robot = msg->parameters[0].value;
	const std::string& view = msg->parameters[1].value;
	const std::string& from = msg->parameters[2].value;
	const std::string& object = msg->parameters[3].value;
	
	ROS_INFO("KCL: (AttemptToExamineObjectPDDLAction) Process the action: (%s %s %s %s %s)", normalised_action_name.c_str(), robot.c_str(), view.c_str(), from.c_str(), object.c_str());
	
	// Check if the object has already been examined, if so then we do not need to do anything.
	std::map<std::string, std::string> parameters;
	parameters["o"] = object;
	if (knowledge_base_->isFactTrue("examined", parameters, true))
	{
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub_.publish(fb);
		return;
	}
	
	// If the object has not yet been examined then we need to drive towards it and then examine it.
	// This movement is allowed to fail, so simply return 'action achieved' so the planner can continue.
	if (true || moveTo(from))
	{
		// Update the robot's location.
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> store;
		knowledge_base_->getFacts(store, "robot_at");
		for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = store.begin(); ci != store.end(); ++ci)
		{
			knowledge_base_->removeFact(*ci, KnowledgeBase::KB_REMOVE_KNOWLEDGE);
		}
		
		std::map<std::string, std::string> parameters;
		parameters["v"] = robot;
		parameters["wp"] = from;
		knowledge_base_->addFact("robot_at", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
		
		// After moving to the location, call the perception server.
		squirrel_planning_msgs::CallAction examine_action;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "view"; kv.value = view;
		examine_action.request.parameters.push_back(kv);
		kv.key = "from"; kv.value = from;
		examine_action.request.parameters.push_back(kv);
		kv.key = "o"; kv.value = object;
		examine_action.request.parameters.push_back(kv);
		
		if (!call_examine_action_client_.call(examine_action))
		{
			ROS_ERROR("KCL: (AttemptToExamineObjectPDDLAction) Failed to call the examine action server!");
			exit(-1);
		}
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
