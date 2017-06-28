#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "PickupPDDLAction.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"

namespace KCL_rosplan
{

PickupPDDLAction::PickupPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::PickupPDDLAction::dispatchCallback, this);
}

PickupPDDLAction::~PickupPDDLAction()
{
	
}

void PickupPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "pickup_object")// || msg->parameters.size() != 3)
	{
		return;
	}
	
	ROS_INFO("KCL: (PickupPDDLAction) Process the action: %s", normalised_action_name.c_str());
	/*
	for (std::vector<diagnostic_msgs::KeyValue>::const_iterator ci = msg->parameters.begin(); ci != msg->parameters.end(); ++ci)
	{
		const std::string& key = (*ci).key;
		const std::string& value = (*ci).value;
		
		ROS_INFO("KCL: (PickupPDDLAction) %s -> %s", key.c_str(), value.c_str());
	}
	*/
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& robot = msg->parameters[0].value;
	const std::string& object_waypoint = msg->parameters[1].value;
	const std::string& robot_waypoint = msg->parameters[2].value;
	const std::string& object = msg->parameters[3].value;
	//const std::string& type = msg->parameters[4].value;
	
	ROS_INFO("KCL: (PickupPDDLAction) Process the action: %s, Pickup %s by %s", normalised_action_name.c_str(), object.c_str(), robot.c_str());
	
	// Remove the old knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem kenny_knowledge;
	kenny_knowledge.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	kenny_knowledge.attribute_name = "object_at";
	kenny_knowledge.is_negative = false;
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "o";
	kv.value = object;
	kenny_knowledge.values.push_back(kv);
	
	kv.key = "wp";
	kv.value = object_waypoint;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (PickupPDDLAction) Could not remove the previous (object_at %s %s) predicate from the knowledge base.", object.c_str(), object_waypoint.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (PickupPDDLAction) Removed the previous (object_at %s %s) predicate from the knowledge base.", object.c_str(), object_waypoint.c_str());
	kenny_knowledge.values.clear();
	
	// Make the gripper not free.
	kenny_knowledge.attribute_name = "gripper_empty";
	
	kv.key = "v";
	kv.value = robot;
	kenny_knowledge.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = kenny_knowledge;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (PickupPDDLAction) Could not remove the previous (gripper_empty %s) predicate from the knowledge base.", robot.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (PickupPDDLAction) Removed the previous (gripper_empty %s) predicate from the knowledge base.", robot.c_str());
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
		ROS_ERROR("KCL: (PickupPDDLAction) Could not add the new (holding %s %s) predicate to the knowledge base.", robot.c_str(), object.c_str());
		exit(-1);
	}
	ROS_INFO("KCL: (PickupPDDLAction) Added the new (holding %s %s) predicate to the knowledge base.", robot.c_str(), object.c_str());
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
