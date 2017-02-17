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

#include "squirrel_manipulation_msgs/HandoverAction.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"

#ifndef KCL_handoveraction
#define KCL_handoveraction

/**
 * This file defines the Handover class.
 * This can be used to exchange objects between the robot and child.
 */
namespace KCL_rosplan {

	class RPHandoverAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::HandoverAction> handover_action_client;
		ros::Publisher action_feedback_pub;
		ros::ServiceClient update_knowledge_client;

		/* execute pushing actions */
		void dispatchGiveAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void dispatchTakeAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* PDDL action feedback */
		void publishFeedback(int action_id, std::string feedback)
		{
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = action_id;
			fb.status = feedback;
			action_feedback_pub.publish(fb);
		}

	public:

		/* constructor */
		RPHandoverAction(ros::NodeHandle &nh, const std::string &handoveractionserver);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
