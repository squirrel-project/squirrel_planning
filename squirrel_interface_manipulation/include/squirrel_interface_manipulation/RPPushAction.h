#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"

#include "squirrel_manipulation_msgs/PushAction.h"
#include "squirrel_manipulation_msgs/SmashAction.h"
#include "squirrel_manipulation_msgs/PutDownAction.h"
#include "kclhand_control/ActuateHandAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

#ifndef KCL_pushaction
#define KCL_pushaction

/**
 * This file defines the RPPushAction class.
 * RPPushAction is used by SQUIRREL to push objects about with the robotino.
 * PDDL "push_object" actions become "squirrel_manipulation_msgs::Push" actions.
 * Waypoint goals are fetched by name from the SceneDB (implemented by mongoDB).
 */
namespace KCL_rosplan {

	class RPPushAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::PushAction> push_action_client;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::SmashAction> smash_action_client;
		ros::Publisher action_feedback_pub;
		ros::ServiceClient update_knowledge_client;

		/* execute pushing actions */
		void dispatchPushAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void dispatchSmashAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* PDDL action feedback */
		void publishFeedback(int action_id, std::string feedback) {
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = action_id;
			fb.status = feedback;
			action_feedback_pub.publish(fb);
		}

	public:

		/* constructor */
		RPPushAction(ros::NodeHandle &nh, std::string &pushactionserver, std::string &smashactionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
