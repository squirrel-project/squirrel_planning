#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "TakeObjectPDDLAction.h"

namespace KCL_rosplan
{

TakeObjectPDDLAction::TakeObjectPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::TakeObjectPDDLAction::dispatchCallback, this);
}

TakeObjectPDDLAction::~TakeObjectPDDLAction()
{
	
}

void TakeObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "take_object" || msg->parameters.size() != 4)
	{
		return;
	}
	
	ROS_INFO("KCL: (TakeObjectPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.ROBOT CHILD1_LOCATION TOY1 CHILD1
	const std::string& robot = msg->parameters[0].value;
	const std::string& child_waypoint = msg->parameters[1].value;
	const std::string& object = msg->parameters[2].value;
	const std::string& child = msg->parameters[3].value;
	
	ROS_INFO("KCL: (TakeObjectPDDLAction) Process the action: %s, Take %s from %s at %s", normalised_action_name.c_str(), object.c_str(), child_waypoint.c_str(), child.c_str());
	
	// Remove the old knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "child_is_holding";
	kenny_knowledge.is_negative = false;
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "c";
	kv.value = child;
	kenny_knowledge.values.push_back(kv);
	
	kv.key = "o";
	kv.value = object;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (TakeObjectPDDLAction) Could not remove the previous (child_is_holding %s %s) predicate from the knowledge base.", child.c_str(), object.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (TakeObjectPDDLAction) Removed the previous (child_is_holding %s %s) predicate from the knowledge base.", child.c_str(), object.c_str());
	kenny_knowledge.values.clear();
	
	kenny_knowledge.attribute_name = "gripper_empty";
	kenny_knowledge.is_negative = false;
	
	kv.key = "v";
	kv.value = robot;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (TakeObjectPDDLAction) Could not remove the previous (gripper_empty %s) predicate from the knowledge base.", robot.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (TakeObjectPDDLAction) Removed the previous (gripper_empty %s) predicate from the knowledge base.", robot.c_str());
	kenny_knowledge.values.clear();
	
	// Add the new knowledge.
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "holding";
	
	kv.key = "v";
	kv.value = robot;
	kenny_knowledge.values.push_back(kv);
	
	kv.key = "o";
	kv.value = object;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (TakeObjectPDDLAction) Could not add the new (holding %s %s) predicate to the knowledge base.", robot.c_str(), object.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (TakeObjectPDDLAction) Added the new (holding %s %s) predicate to the knowledge base.", robot.c_str(), object.c_str());
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
