#include <sstream>
#include <complex>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "InspectObjectPDDLAction.h"

namespace KCL_rosplan
{

InspectObjectPDDLAction::InspectObjectPDDLAction(ros::NodeHandle& node_handle)
	: message_store_(node_handle)
{
	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::InspectObjectPDDLAction::dispatchCallback, this);
}

InspectObjectPDDLAction::~InspectObjectPDDLAction()
{
	
}

void InspectObjectPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "inspect_object" || msg->parameters.size() != 2)
	{
		return;
	}
	
	ROS_INFO("KCL: (InspectObjectPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& robot = msg->parameters[0].value;
	const std::string& object = msg->parameters[1].value;
	
	ROS_INFO("KCL: (InspectObjectPDDLAction) Process the action: %s, Inspect object %s by %s", normalised_action_name.c_str(), object.c_str(), robot.c_str());
	
	
	// Locate the location of the robot.
	tf::StampedTransform transform;
	tf::TransformListener tfl;
	try {
		tfl.waitForTransform("/map","/base_link", ros::Time::now(), ros::Duration(1.0));
		tfl.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	} catch ( tf::TransformException& ex ) {
		ROS_ERROR("KCL: (SimulatedObservePDDLAction) Error find the transform between /map and /base_link.");
		fb.action_id = msg->action_id;
		fb.status = "action failed";
		action_feedback_pub_.publish(fb);
		return;
	}
	
	std::string closest_box;
	geometry_msgs::PoseStamped closest_box_pose;
	float min_distance_from_robot = std::numeric_limits<float>::max();
	
	// Get all boxes and their poses, pick the one that is closest.
	rosplan_knowledge_msgs::GetInstanceService getInstances;
	getInstances.request.type_name = "box";
	if (!get_instance_client_.call(getInstances)) {
		ROS_ERROR("KCL: (InspectObjectPDDLAction) Failed to get all the box instances.");
		fb.action_id = msg->action_id;
		fb.status = "action failed";
		action_feedback_pub_.publish(fb);
		return;
	}

	ROS_INFO("KCL: (InspectObjectPDDLAction) Received all the box instances %zd.", getInstances.response.instances.size());
	for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
	{
		// fetch position of the box from message store
		std::stringstream ss;
		ss << *ci << "_location";
		std::string box_loc = ss.str();

		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store_.queryNamed<geometry_msgs::PoseStamped>(box_loc, results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (InspectObjectPDDLAction) aborting waypoint request; no matching boxID %s", box_loc.c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
				return;
			}
		} else {
			ROS_ERROR("KCL: (InspectObjectPDDLAction) could not query message store to fetch box pose %s", box_loc.c_str());
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}

		// request manipulation waypoints for object
		geometry_msgs::PoseStamped &box_pose = *results[0];
		float distance = (box_pose.pose.position.x - transform.getOrigin().getX()) * (box_pose.pose.position.x - transform.getOrigin().getX()) +
								(box_pose.pose.position.y - transform.getOrigin().getY()) * (box_pose.pose.position.y - transform.getOrigin().getY());
		
		if (distance < min_distance_from_robot)
		{
			min_distance_from_robot = distance;
			closest_box = *ci;
			closest_box_pose = box_pose;
		}
	}

	ROS_INFO("KCL: (InspectObjectPDDLAction) Closest box is: %s.", closest_box.c_str());
	
	// Add the fact belongs_in oject box.
	rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
	knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
	knowledge_item.attribute_name = "belongs_in";
	
	// Add the new knowledge.
	rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
	knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
	
	// Remove all the other facts.
	for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
	{
		const std::string& box_name = *ci;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "o";
		kv.value = object;
		knowledge_item.values.push_back(kv);
		
		kv.key = "b";
		kv.value = box_name;
		knowledge_item.values.push_back(kv);
		knowledge_item.is_negative = box_name != closest_box;
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not remove the belongs_in predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Add %s (belongs_in %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", object.c_str(), box_name.c_str());
		
		// Remove the opposite option from the knowledge base.
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE;
		
		knowledge_item.is_negative = !knowledge_item.is_negative;
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client_.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not remove the belongs_in predicate to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Removed %s (belongs_in %s %s) to the knowledge base.", knowledge_item.is_negative ? "NOT" : "", object.c_str(), box_name.c_str());
		
		knowledge_item.values.clear();
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
