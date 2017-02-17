#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "SimulatedObservePDDLAction.h"

namespace KCL_rosplan
{

SimulatedObservePDDLAction::SimulatedObservePDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::SimulatedObservePDDLAction::dispatchCallback, this);
}

SimulatedObservePDDLAction::~SimulatedObservePDDLAction()
{
	
}

void SimulatedObservePDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "observe-has_commanded" &&
	    normalised_action_name != "observe-is_of_type" &&
	    normalised_action_name != "observe-holding" &&
	    normalised_action_name != "observe-sorting_done" &&
	    normalised_action_name != "observe-is_examined" &&
	    normalised_action_name != "observe-belongs_in" &&
	    normalised_action_name != "jump")
	{
		return;
	}
	
	ROS_INFO("KCL: (SimulatedObservePDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	if (normalised_action_name == "observe-sorting_done")
	{
		// Add the new knowledge.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "sorting_done";
		
		float p = (float)rand() / (float)RAND_MAX;
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Done sorting? %f >= %f.", p, 0.5f);
		knowledge_item.is_negative = p >= 0.5f;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not add the sorting_done predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Added %s (sorting_done) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
		
		// Remove the opposite option from the knowledge base.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		knowledge_item.is_negative = !knowledge_item.is_negative;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not remove the sorting_done predicate to the knowledge base.");
	// 		exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Removed %s (sorting_done) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "");
		
		knowledge_item.values.clear();
	}
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};

