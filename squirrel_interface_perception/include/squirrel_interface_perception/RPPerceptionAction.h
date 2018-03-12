#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "squirrel_planning_knowledge_msgs/AddObjectService.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "mongodb_store/message_store.h"
#include <squirrel_manipulation_msgs/ManipulationAction.h>
//#include "squirrel_planning_execution/KnowledgeBase.h"
#include "../../../squirrel_planning_execution/include/squirrel_planning_execution/KnowledgeBase.h"
#include <squirrel_planning_msgs/CallAction.h>

#ifndef KCL_perception
#define KCL_perception

/**
 * This file defines the RPPerceptionAction class.
 * RPPerceptionAction is used to connect ROSPlan to the object perception in SQUIRREL
 */
namespace KCL_rosplan {

	class RPPerceptionAction
	{

	private:

		mongodb_store::MessageStoreProxy message_store;

		actionlib::SimpleActionClient<squirrel_object_perception_msgs::LookForObjectsAction> examine_action_client;
		//actionlib::SimpleActionClient<squirrel_object_perception_msgs::RecognizeObjectsAction> recognise_action_client;
		actionlib::SimpleActionClient<squirrel_manipulation_msgs::ManipulationAction> object_manipulation_client_;

		ros::ServiceClient find_dynamic_objects_client;
		ros::ServiceClient add_object_client;
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;
		ros::ServiceClient knowledge_query_client;
		ros::Publisher action_feedback_pub;

		ros::ServiceServer examine_action_service_;
		
		ros::Subscriber joint_state_sub;

		std::map<std::string,std::string> db_name_map;

		void publishFeedback(int action_id, std::string feedback);

		/* actions */
		void examineAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		//void examineObject(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void examineObjectInHandAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		void exploreAction(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		//void lookAtObject(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);

		/* objects to database */
		void updateType(const std::string& object_id, const std::string& object_rec_id);

		void addObject(squirrel_object_perception_msgs::SceneObject &object);
		void updateObject(squirrel_object_perception_msgs::SceneObject &object, std::string newWaypoint);
		void removeObject(squirrel_object_perception_msgs::SceneObject &object);

		void registerPoints(const sensor_msgs::PointCloud2::ConstPtr& msg);

		/* Arm manipulation */
		bool extendArm();
		bool retractArm();
		void waitForArm(const std_msgs::Float64MultiArray& goal_state, float error);

		void jointCallback(const sensor_msgs::JointStateConstPtr& msg);
		sensor_msgs::JointState last_joint_state;
		KnowledgeBase knowledge_base_;

	public:

		/* constructor */
		RPPerceptionAction(ros::NodeHandle &nh, const std::string &actionserver, const std::string& recogniseserver, const std::string& object_manipulation_topic);

		/* listen to and process action_dispatch topic */
		void dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
		
		bool examineAction(squirrel_planning_msgs::CallAction::Request& req,
		                   squirrel_planning_msgs::CallAction::Response& res);
	};
}
#endif
