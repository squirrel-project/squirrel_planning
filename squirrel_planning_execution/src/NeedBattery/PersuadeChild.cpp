#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include "../pddl_actions/PlannerInstance.h"
#include "PersuadeChild.h"

namespace KCL_rosplan
{

PersuadeChild::PersuadeChild(ros::NodeHandle& node_handle)
	: node_handle_(&node_handle), message_store_(node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	get_current_goals_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_goals");
	
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::PersuadeChild::dispatchCallback, this);
}

PersuadeChild::~PersuadeChild()
{
	
}

bool PersuadeChild::initialiseGoal()
{
	// Find all current goals.
	rosplan_knowledge_msgs::GetAttributeService gat;
	if (!get_current_goals_client_.call(gat))
	{
		ROS_ERROR("KCL: (PersuadeChild) Failed to get all current goals.");
		return false;
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_GOAL;
	//knowledge_update_service.request.knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	for (std::vector<rosplan_knowledge_msgs::KnowledgeItem>::const_iterator ci = gat.response.attributes.begin(); ci != gat.response.attributes.end(); ++ci)
	{
		knowledge_update_service.request.knowledge = *ci;
		if (!update_knowledge_client_.call(knowledge_update_service))
		{
			ROS_ERROR("KCL: (PersuadeChild) error removing all goals predicate");
			return false;
		}
	}
	
	// Add the new goal.
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	knowledge_item.attribute_name = "has_battery";
	knowledge_item.is_negative = false;
	
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_GOAL;
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (PersuadeChild) Could not add the fact (has_battery) to the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (PersuadeChild) Added the fact (has_battery) to the knowledge base.");
	return true;
}

void PersuadeChild::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "persuade-child-give-battery" || msg->parameters.size() != 4)
	{
		return;
	}
	
	ROS_INFO("KCL: (PersuadeChild) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	initialiseGoal();
	
	// Update the domain.ROBOT CHILD1_LOCATION TOY1 CHILD1
	const std::string& robot = msg->parameters[0].value;
	const std::string& child = msg->parameters[1].value;
	const std::string& object = msg->parameters[2].value;
	const std::string& child_waypoint = msg->parameters[3].value;
	
	ROS_INFO("KCL: (PersuadeChild) Process the action: %s, %s gives %s to %s at %s", normalised_action_name.c_str(), child.c_str(), object.c_str(), robot.c_str(), child_waypoint.c_str());
	bool actionAchieved = false;
	
	PlannerInstance& planner_instance = PlannerInstance::createInstance(*node_handle_, "ff");
	
	// Lets start the planning process.
	std::string data_path;
	node_handle_->getParam("/data_path", data_path);
	
	std::string planner_path;
	node_handle_->getParam("/planner_path", planner_path);
	
	std::string domain_path;
	node_handle_->getParam("/rosplan/domain_path", domain_path);
	
	//std::stringstream ss;
	//ss << data_path << "tactical-behaviour-problem.pddl";
	std::string domain_name = data_path + "robot-micro-behaviour-domain.pddl";
	std::string problem_name = data_path + "robot_micro_behaviour_problem.pddl";
	
	std::string planner_command;
	node_handle_->getParam("/rosplan_planning_system/planner_command", planner_command);
	
	//ss.str(std::string());
	//ss << "timeout 180 " << planner_path << "ff -o DOMAIN -f PROBLEM";
	//std::string planner_command = ss.str();
	/*
	// Before calling the planner we create the domain so it can be parsed.
	if (!createDomain())
	{
		ROS_ERROR("KCL: (ExamineAreaPDDLAction) failed to produce a domain at %s for action name %s.", domain_name.c_str(), action_name.c_str());
		return;
	}
	*/
	planner_instance.startPlanner(domain_name, problem_name, data_path, planner_command);
	
	// publish feedback (enabled)
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);

	// wait for action to finish
	ros::Rate loop_rate(1);
	while (ros::ok() && (planner_instance.getState() == actionlib::SimpleClientGoalState::ACTIVE || planner_instance.getState() == actionlib::SimpleClientGoalState::PENDING)) {
		ros::spinOnce();
		loop_rate.sleep();
	}

	actionlib::SimpleClientGoalState state = planner_instance.getState();
	ROS_INFO("KCL: (ExamineAreaPDDLAction) action finished: %s, %s", "persuade-child-give-battery", state.toString().c_str());

	if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		/*
		// Update the knowledge base with what has been achieved.
		const std::string& robot = msg->parameters[0].value;
		const std::string& area = msg->parameters[1].value;
		
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Process the action: %s, Examine %s by %s", action_name.c_str(), area.c_str(), robot.c_str());
		
		// Remove the old knowledge.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
		kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		kenny_knowledge.attribute_name = "examined";
		kenny_knowledge.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "a";
		kv.value = area;
		kenny_knowledge.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = kenny_knowledge;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (ExamineAreaPDDLAction) Could not add the (examined %s) predicate to the knowledge base.", area.c_str());
			exit(-1);
		}
		ROS_INFO("KCL: (ExamineAreaPDDLAction) Added the action (examined %s) predicate to the knowledge base.", area.c_str());
		kenny_knowledge.values.clear();
		*/
		// publish feedback (achieved)
		fb.action_id = msg->action_id;
		fb.status = "action achieved";
		action_feedback_pub_.publish(fb);
	}
	else
	{
		// publish feedback (failed)
		fb.action_id = msg->action_id;
		fb.status = "action failed";
		action_feedback_pub_.publish(fb);
	}
}

};
