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
#include "squirrel_manipulation_msgs/PushAction.h"
#include "squirrel_manipulation_msgs/SmashAction.h"
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

		bool simulate_;

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::PushAction> push_action_client;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::SmashAction> smash_action_client;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> simulate_client;
		ros::Publisher action_feedback_pub;

		/* execute pushing actions */
		void dispatchPushAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void dispatchSmashAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

	public:

		/* constructor */
		RPPushAction(ros::NodeHandle &nh, std::string &pushactionserver, std::string &smashactionserver, bool simulate);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
