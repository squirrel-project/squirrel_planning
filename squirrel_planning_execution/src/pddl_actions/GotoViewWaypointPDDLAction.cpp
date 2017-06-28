#include <sstream>
#include <complex>
#include <limits>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>

#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_dispatch_msgs/ActionFeedback.h>


#include "GotoViewWaypointPDDLAction.h"

namespace KCL_rosplan
{

GotoViewWaypointPDDLAction::GotoViewWaypointPDDLAction(ros::NodeHandle& node_handle, const std::string &actionserver)
	 : message_store_(node_handle), action_client_(actionserver, true)
	{
	// costmap client
	clear_costmaps_client_ = node_handle.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

	// knowledge interface
	update_knowledge_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
	get_instance_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
	get_attribute_client_ = node_handle.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
	action_feedback_pub_ = node_handle.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);

	// Subscribe to the action feedback topic.
	dispatch_sub_ = node_handle.subscribe("/kcl_rosplan/action_dispatch", 1000, &KCL_rosplan::GotoViewWaypointPDDLAction::dispatchCallback, this);
}

GotoViewWaypointPDDLAction::~GotoViewWaypointPDDLAction()
{
	
}

void GotoViewWaypointPDDLAction::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
{
	std::string normalised_action_name = msg->name;
	std::transform(normalised_action_name.begin(), normalised_action_name.end(), normalised_action_name.begin(), tolower);
	
	// Check if this action is to be handled by this class.
	if (normalised_action_name != "goto_view_waypoint")
	{
		return;
	}
	
	ROS_INFO("KCL: (GotoViewWaypointPDDLAction) Process the action: %s", normalised_action_name.c_str());

	
	// Report this action is enabled and completed successfully.
	rosplan_dispatch_msgs::ActionFeedback fb;
	fb.action_id = msg->action_id;
	fb.status = "action enabled";
	action_feedback_pub_.publish(fb);
	
	// Check which box is closest and move to its nearest waypoint.

	// Locat the child.
	std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > child_results;
	if(!message_store_.queryNamed<geometry_msgs::PoseStamped>("child_location", child_results) ||
           child_results.size() != 1)
	{
		ROS_ERROR("KCL: (SimulatedObservePDDLAction) Could not find the location of the child.");
		exit(1);
	}
	geometry_msgs::PoseStamped child_location = *child_results[0];
	ROS_INFO("KCL: (SimulatedObservePDDLAction) Child is at (%f,%f,%f,)", child_location.pose.position.x, child_location.pose.position.y, child_location.pose.position.z);

/*
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
	ROS_INFO("KCL: (SimulatedObservePDDLAction) Robot is at (%f,%f,%f).", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
*/
	
	std::string closest_box;
	geometry_msgs::PoseStamped closest_box_pose;
	float min_distance_from_robot = std::numeric_limits<float>::max();
	
	// Get all boxes and their poses, pick the one that is closest.
	rosplan_knowledge_msgs::GetInstanceService getInstances;
	getInstances.request.type_name = "box";
	if (!get_instance_client_.call(getInstances)) {
		ROS_ERROR("KCL: (SimulatedObservePDDLAction) Failed to get all the box instances.");
		fb.action_id = msg->action_id;
		fb.status = "action failed";
		action_feedback_pub_.publish(fb);
		return;
	}
	ROS_INFO("KCL: (SimulatedObservePDDLAction) Received all the box instances.");
	for (std::vector<std::string>::const_iterator ci = getInstances.response.instances.begin(); ci != getInstances.response.instances.end(); ++ci)
	{
		// fetch position of the box from message store
		std::stringstream ss;
		ss << *ci << "_location";
		std::string box_loc = ss.str();

		std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
		if(message_store_.queryNamed<geometry_msgs::PoseStamped>(box_loc, results)) {
			if(results.size()<1) {
				ROS_ERROR("KCL: (SimulatedObservePDDLAction) aborting waypoint request; no matching boxID %s", box_loc.c_str());
				fb.action_id = msg->action_id;
				fb.status = "action failed";
				action_feedback_pub_.publish(fb);
				return;
			}
		} else {
			ROS_ERROR("KCL: (SimulatedObservePDDLAction) could not query message store to fetch box pose %s", box_loc.c_str());
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}

		// request manipulation waypoints for object
		geometry_msgs::PoseStamped &box_pose = *results[0];
//		float distance = (box_pose.pose.position.x - transform.getOrigin().getX()) * (box_pose.pose.position.x - transform.getOrigin().getX()) +
//								(box_pose.pose.position.y - transform.getOrigin().getY()) * (box_pose.pose.position.y - transform.getOrigin().getY());
		float distance = (box_pose.pose.position.x - child_location.pose.position.x) * (box_pose.pose.position.x - child_location.pose.position.x) +
								(box_pose.pose.position.y - child_location.pose.position.y) * (box_pose.pose.position.y - child_location.pose.position.y);
		
		ROS_INFO("KCL: (SimulatedObservePDDLAction) Box %s is at (%f,%f,%f), distance: %f.", ci->c_str(), box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z, distance);

		
		if (distance < min_distance_from_robot)
		{
			min_distance_from_robot = distance;
			closest_box = *ci;
			closest_box_pose = box_pose;
		}
	}
	
	// Send a movebase goal to get to this box.
	std::stringstream ss;
	ss << "near_" << closest_box;
	std::string wpID = ss.str();

	ROS_INFO("KCL: (SimulatedObservePDDLAction) Closest box is %s, move to %s", closest_box.c_str(), wpID.c_str());
	
	// get pose from message store
	std::vector< boost::shared_ptr<geometry_msgs::PoseStamped> > results;
	if(message_store_.queryNamed<geometry_msgs::PoseStamped>(wpID, results)) {
		if(results.size()<1) {
			ROS_INFO("KCL: (GotoViewWaypointPDDLAction) aborting action dispatch; no matching wpID %s", wpID.c_str());
			fb.action_id = msg->action_id;
			fb.status = "action failed";
			action_feedback_pub_.publish(fb);
			return;
		}
		if(results.size()>1)
			ROS_INFO("KCL: (GotoViewWaypointPDDLAction) multiple waypoints share the same wpID %s", wpID.c_str());

		ROS_INFO("KCL: (GotoViewWaypointPDDLAction) waiting for move_base action server to start");
		action_client_.waitForServer();

		// dispatch MoveBase action
		move_base_msgs::MoveBaseGoal goal;
		geometry_msgs::PoseStamped &pose = *results[0];
		
		std::cout << "Send the pose: (" << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << ") to movebase for waypoint: " << wpID << "." << std::endl;
		
		goal.target_pose = pose;
		action_client_.sendGoal(goal);

		bool finished_before_timeout = action_client_.waitForResult();
		if (finished_before_timeout) {

			actionlib::SimpleClientGoalState state = action_client_.getState();
			ROS_INFO("KCL: (GotoViewWaypointPDDLAction) action finished: %s", state.toString().c_str());

			if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {

				// publish feedback (achieved)
				fb.action_id = msg->action_id;
				fb.status = "action achieved";
				action_feedback_pub_.publish(fb);
				return;
			} else {

				// clear costmaps
				std_srvs::Empty emptySrv;
				clear_costmaps_client_.call(emptySrv);

			}
		} else {
			// timed out (failed)
			action_client_.cancelAllGoals();
			ROS_INFO("KCL: (GotoViewWaypointPDDLAction) action timed out");
		}
	} else {
		// no KMS connection (failed)
		ROS_INFO("KCL: (GotoViewWaypointPDDLAction) aborting action dispatch; query to sceneDB failed; wpID %s", wpID.c_str());
	}
	
	fb.action_id = msg->action_id;
	fb.status = "action failed";
	action_feedback_pub_.publish(fb);
}

};
