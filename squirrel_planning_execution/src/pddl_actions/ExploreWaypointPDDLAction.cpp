#include <sstream>
#include <complex>
#include <geometry_msgs/Pose.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>
#include <squirrel_object_perception_msgs/SceneObject.h>
#include <squirrel_planning_execution/KnowledgeBase.h>
#include <geometry_msgs/PoseStamped.h>

#include "ExploreWaypointPDDLAction.h"

namespace KCL_rosplan
{

ExploreWaypointPDDLAction::ExploreWaypointPDDLAction(ros::NodeHandle& node_handle, mongodb_store::MessageStoreProxy& message_store, KnowledgeBase& knowledge_base)
	: message_store_(&message_store), knowledge_base_(&knowledge_base)
{
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::ExploreWaypointPDDLAction::dispatchCallback, this);
}

ExploreWaypointPDDLAction::~ExploreWaypointPDDLAction()
{
	
}

void ExploreWaypointPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "explore_waypoint" || msg->parameters.size() != 2)
	{
		return;
	}
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Process the action: %s", normalised_action_name.c_str());
	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Update the domain.
	const std::string& robot = msg->parameters[0].value;
	const std::string& explored_waypoint = msg->parameters[1].value;
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Process the action: %s, %s explored %s", normalised_action_name.c_str(), robot.c_str(), explored_waypoint.c_str());
	
	std::map<std::string, std::string> parameters;
	parameters["wp"] = explored_waypoint;
	knowledge_base_->addFact("explored", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
	
	std::vector<std::string> objects;
	knowledge_base_->getInstances(objects, "object");
	
	// Get all the objects from the knowledge base, so we can give our objects a unique name.
	// Check if this object has been classified or not.
	unsigned int object_nr = objects.size();
	
	// Simulate that we found some (or none!) objects at this waypoint.
	unsigned int new_objects =  rand() % 2;
	for (unsigned int i = 0; i < new_objects; ++i)
	{
		std::stringstream ss;
		ss << "object" << object_nr;
		std::string object_name = ss.str();
		std::string waypoint_name = "waypoint_" + ss.str();
		
		knowledge_base_->addInstance("object", object_name);
		knowledge_base_->addInstance("waypoint", waypoint_name);
		
		parameters.clear();
		parameters["o"] = object_name;
		parameters["wp"] = waypoint_name;
		knowledge_base_->addFact("object_at", parameters, true, KnowledgeBase::KB_ADD_KNOWLEDGE);
		++object_nr;
		
		// Insert this face waypoint into the message store.
		squirrel_object_perception_msgs::SceneObject so;
		so.header.frame_id = "/map";
		so.header.stamp = ros::Time::now();
		so.id = object_name;
		so.category = "battery_red";
		so.pose.position.x = 1;
		so.pose.position.y = 2;
		so.pose.position.z = 0.1;
		so.pose.orientation.x = 0;
		so.pose.orientation.y = 0;
		so.pose.orientation.z = 0;
		so.pose.orientation.w = 1;
		so.bounding_cylinder.height = 0.2;
		so.bounding_cylinder.diameter = 0.3;
		message_store_->insertNamed(object_name, so);
		
		geometry_msgs::PoseStamped ps;
		ps.header = so.header;
		ps.pose = so.pose;
		message_store_->insertNamed(waypoint_name, so);
	}
	
	ROS_INFO("KCL: (ExploreWaypointPDDLAction) Added %d new objects!", new_objects);
	
	fb.action_id = msg->action_id;
	fb.status = "action achieved";
	action_feedback_pub_.publish(fb);
}

};
