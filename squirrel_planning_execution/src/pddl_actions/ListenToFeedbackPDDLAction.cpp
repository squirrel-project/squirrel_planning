#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>

#include "ListenToFeedbackPDDLAction.h"

namespace KCL_rosplan
{

ListenToFeedbackPDDLAction::ListenToFeedbackPDDLAction(ros::NodeHandle& node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	
	query_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ListenToFeedbackPDDLAction::dispatchCallback, this);
}

ListenToFeedbackPDDLAction::~ListenToFeedbackPDDLAction()
{
	
}

void ListenToFeedbackPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "listen_to_feedback" || msg->parameters.size() != 5)
	{
		return;
	}
	
	const std::string& robot = msg->parameters[0].value;
	const std::string& object = msg->parameters[1].value;
	const std::string& box = msg->parameters[2].value;
	const std::string& object_wp = msg->parameters[3].value;
	const std::string& robot_wp = msg->parameters[4].value;
	
	ROS_INFO("KCL: (ListenToFeedbackPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	ros::Rate loop_rate(1);
	
	bool received_feedback = false;
	bool possitive_feedback = true;
	
	while (!received_feedback)
	{
		rosplan_knowledge_msgs::KnowledgeQueryService knowledge_query;
	
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "has_commanded";
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "k";
		kv.value = "kid_0";
		knowledge_item.values.push_back(kv);
		
		kv.key = "c";
		kv.value = "no";
		knowledge_item.values.push_back(kv);
		knowledge_item.is_negative = false;
		
		knowledge_query.request.knowledge.push_back(knowledge_item);
		
		knowledge_item.values.pop_back();
		kv.key = "c";
		kv.value = "yes";
		knowledge_item.values.push_back(kv);
		knowledge_item.is_negative = false;
		
		knowledge_query.request.knowledge.push_back(knowledge_item);
		
		// Check if any of these facts are true.
		if (!query_knowledge_client_.call(knowledge_query))
		{
			ROS_ERROR("KCL: (ObserveClassifiableOnAttemptPDDLAction) Could not call the query knowledge server.");
			exit(1);
		}
		
		if (knowledge_query.response.results[0] == 1)
		{
			possitive_feedback = false;
			received_feedback = true;
		}
		else if (knowledge_query.response.results[1] == 1)
		{
			possitive_feedback = true;
			received_feedback = true;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	knowledge_item.attribute_name = "belongs_in";
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "o";
	kv.value = object;
	knowledge_item.values.push_back(kv);
	
	kv.key = "b";
	kv.value = box;
	knowledge_item.values.push_back(kv);
	
	knowledge_item.is_negative = !possitive_feedback;
	knowledge_update_service.request.knowledge = knowledge_item;
	
	// Check if any of these facts are true.
	if (!update_knowledge_client_.call(knowledge_update_service))
	{
		ROS_ERROR("KCL: (ExamineObjectInHandPDDLAction) Could not add the (belongs_in %s %s) predicate to the knowledge base.", object.c_str(), box.c_str());
		exit(1);
	}
	ROS_INFO("KCL: (ListenFeedbackPDDLAction) Added the %s (belongs_in %s %s) predicate to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", object.c_str(), box.c_str());
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
