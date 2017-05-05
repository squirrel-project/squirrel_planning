#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <squirrel_manipulation_msgs/JointPtpAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "mongodb_store/message_store.h"

#include "geometry_msgs/PoseStamped.h"
#include "squirrel_object_perception_msgs/SceneObject.h"
#include "squirrel_manipulation_msgs/BlindGraspAction.h"
#include "squirrel_manipulation_msgs/PutDownAction.h"
#include "squirrel_manipulation_msgs/PtpAction.h"
#include "kclhand_control/graspPreparation.h"
#include "kclhand_control/ActuateHandAction.h"


#ifndef KCL_graspaction
#define KCL_graspaction

/**
 * This file defines the RPGraspAction class.
 * RPGraspAction is used by SQUIRREL to grasp and lift objects with the robotino.
 * PDDL "grasp_object" actions become one of the grasp actions.
 * (eg. BlindGrasp)
 */
namespace KCL_rosplan {

	class RPGraspAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::BlindGraspAction> blind_grasp_action_client;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::PutDownAction> putDownActionClient;
		actionlib::SimpleActionClient<kclhand_control::ActuateHandAction> kclhandGraspActionClient;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::JointPtpAction> ptpActionClient;
		ros::Publisher action_feedback_pub;
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient clear_cost_map_client;
		ros::Subscriber joint_state_sub;
        bool do_placement;

		/* execute pushing actions */
		bool dispatchBlindGraspAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		bool dispatchDropAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		bool dispatchDropActionCorrect(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* PDDL action feedback */
		void publishFeedback(int action_id, std::string feedback) {
			rosplan_dispatch_msgs::ActionFeedback fb;
			fb.action_id = action_id;
			fb.status = feedback;
			action_feedback_pub.publish(fb);
		}

		bool retractArm();
		void waitForArm(const std_msgs::Float64MultiArray& goal_state, float error);

		void jointCallback(const sensor_msgs::JointStateConstPtr& msg);
		sensor_msgs::JointState last_joint_state;

	public:

		/* constructor */
		RPGraspAction(ros::NodeHandle &nh, std::string &blindGraspActionServer);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif
