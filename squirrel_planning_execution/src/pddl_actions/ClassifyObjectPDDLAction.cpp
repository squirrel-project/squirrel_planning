#include <sstream>
#include <complex>

#include <cstdlib>

#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeQueryService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>

#include <diagnostic_msgs/KeyValue.h>

#include "ClassifyObjectPDDLAction.h"

namespace KCL_rosplan
{

ClassifyObjectPDDLAction::ClassifyObjectPDDLAction(ros::NodeHandle& node_handle, float classification_probability)
	: classification_probability_(classification_probability), ask_user_input_(false)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
	//query_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ClassifyObjectPDDLAction::dispatchCallback, this);
	
	// Initialise the random number generator with a fixed number so we can reproduce the same results.
	//srand (1234);
	srand(time(NULL));
	
	node_handle.getParam("/simulated_actions/query_user", ask_user_input_);
}

ClassifyObjectPDDLAction::~ClassifyObjectPDDLAction()
{
	
}

void ClassifyObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "observe-classifiable_from")
	{
		return;
	}
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& from = msg->parameters[0].value;
	const std::string& view = msg->parameters[1].value;
	const std::string& object = msg->parameters[2].value;
	
	ROS_INFO("KCL: (ClassifyObjectPDDLAction) Process the action: (%s %s %s %s)", normalised_action_name.c_str(), from.c_str(), view.c_str(), object.c_str());
	
	// Add the new knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	knowledge_item.attribute_name = "classifiable_from";
	
	if (ask_user_input_)
	{
		bool received_user_input = false;
		while (ros::ok() && !received_user_input)
		{
			char response;
			std::cout << "Should this sensing action succeed? (y/n)" << std::endl;
			std::cin >> response;
			received_user_input = true;
			if (response == 'n')
			{
				knowledge_item.is_negative = true;
			}
			else if (response == 'y')
			{
				knowledge_item.is_negative = false;
			}
			else
			{
				received_user_input = false;
			}
		}
	}
	else 
	{
		float p = (float)rand() / (float)RAND_MAX;
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Is negative? %f >= %f.", p, classification_probability_);
		knowledge_item.is_negative = (float)rand() / (float)RAND_MAX >= classification_probability_;
	}
	bool classification_succeeded = !knowledge_item.is_negative;
	
	diagnostic_msgs::KeyValue kv;
	kv.key = "from";
	kv.value = from;
	knowledge_item.values.push_back(kv);
	
	kv.key = "view";
	kv.value = view;
	knowledge_item.values.push_back(kv);
	
	kv.key = "o";
	kv.value = object;
	knowledge_item.values.push_back(kv);
	
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the classifiable_from predicate to the knowledge base.");
		exit(-1);
	}
	ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added %s (classifiable_from %s %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", from.c_str(), view.c_str(), object.c_str());
	
	// Remove the opposite option from the knowledge base.
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
	knowledge_item.is_negative = !knowledge_item.is_negative;
	knowledge_update_service.request.knowledge = knowledge_item;
	if (!update_knowledge_client_.call(knowledge_update_service)) {
		ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not remove the classifiable_from predicate to the knowledge base.");
// 		exit(-1);
	}
	ROS_INFO("KCL: (ClassifyObjectPDDLAction) Removed %s (classifiable_from %s %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", from.c_str(), view.c_str(), object.c_str());
	
	knowledge_item.values.clear();

	
	// If the fact is not negative, we need to decide upon the type of this object.
	if (classification_succeeded)
	{
		// Check if this object has been classified or not.
		rosplan_knowledge_msgs::GetInstanceService get_instance;
		get_instance.request.type_name = "type";
		
		if (!get_instance_client_.call(get_instance))
		{
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not get the instances of type 'type'.");
			exit(1);
		}
		if (get_instance.response.instances.empty())
		{
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Got instances of type 'type', but there are none!");
			exit(1);
		}
		
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		// Select one type at random.
		const std::string type = get_instance.response.instances[rand() % get_instance.response.instances.size()];
		
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "is_of_type";
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = object;
		knowledge_item.values.push_back(kv);
		
		kv.key = "t";
		kv.value = type;
		knowledge_item.values.push_back(kv);
		
		knowledge_item.is_negative = false;
		knowledge_update_service.request.knowledge = knowledge_item;
		
		// Check if any of these facts are true.
		if (!update_knowledge_client_.call(knowledge_update_service))
		{
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), type.c_str());
			exit(1);
		}
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), type.c_str());
		
		// Remove the negative option from the KB.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		knowledge_item.is_negative = true;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service))
		{
			ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not remove the (not (is_of_type %s %s)) predicate to the knowledge base.", object.c_str(), type.c_str());
			exit(1);
		}
		ROS_INFO("KCL: (ClassifyObjectPDDLAction) Removed the (not (is_of_type %s %s)) predicate to the knowledge base.", object.c_str(), type.c_str());
		
		// Make it NOT of the other types.
		for (std::vector<std::string>::const_iterator ci = get_instance.response.instances.begin(); ci != get_instance.response.instances.end(); ++ci)
		{
			const std::string& other_type = *ci;
			if (type != other_type)
			{
				knowledge_item.values.pop_back();
				kv.key = "t";
				kv.value = other_type;
				knowledge_item.values.push_back(kv);
				knowledge_item.is_negative = true;
				
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client_.call(knowledge_update_service))
				{
					ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not add the (not (is_of_type %s %s)) predicate to the knowledge base.", object.c_str(), other_type.c_str());
					exit(1);
				}
				ROS_INFO("KCL: (ClassifyObjectPDDLAction) Added the (not (is_of_type %s %s)) predicate to the knowledge base.", object.c_str(), other_type.c_str());
				
				knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
				
				knowledge_item.is_negative = false;
				knowledge_update_service.request.knowledge = knowledge_item;
				if (!update_knowledge_client_.call(knowledge_update_service))
				{
					ROS_ERROR("KCL: (ClassifyObjectPDDLAction) Could not remove the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), other_type.c_str());
					exit(1);
				}
				ROS_INFO("KCL: (ClassifyObjectPDDLAction) Removed the (is_of_type %s %s) predicate to the knowledge base.", object.c_str(), other_type.c_str());
			}
		}
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
