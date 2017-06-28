#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "mongodb_store/message_store.h"
#include "squirrel_hri_msgs/FollowChildAction.h"
#include "actionlib/client/simple_action_client.h"
#include <geometry_msgs/Pose2D.h>

#ifndef SQUIRREL_INTERFACE_HRI_FOLLOW_CHILD_ACTION_H
#define SQUIRREL_INTERFACE_HRI_FOLLOW_CHILD_ACTION_H

/**
 * This file defines the FollowChildAction class.
 * It executes the 'follow_child' planning action.
 *
 * Parameters:
 * - c (if it is CLOSEST_CHILD it is the closest child. Otherwise it 
 * should follow a specific child (as stored in the knowledge base).
 */
namespace KCL_rosplan {

	class FollowChildAction
	{

	private:
		
		// Scene database
		mongodb_store::MessageStoreProxy message_store;
		std::string last_stored_id;

		// Knowledge base
		ros::ServiceClient knowledgeInterface;
		ros::ServiceClient clear_cost_map_client;

		// action topics
		ros::Publisher action_feedback_pub;
		ros::Publisher head_tilt_pub;
		ros::Publisher head_nod_pub;
		ros::Publisher sound_pub_;
		
		// locations where we expect a child to go.
		std::vector<geometry_msgs::Point> child_destinations;

	public:

		/* constructor */
		FollowChildAction(ros::NodeHandle &nh, const std::string& follow_child_action_name, const std::vector<geometry_msgs::Point>& child_destinations);

		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		actionlib::SimpleActionClient<squirrel_hri_msgs::FollowChildAction> action_client;
	};
}
#endif

