#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <std_srvs/Empty.h>
#include <geometry_msgs/PointStamped.h>

#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/AddWaypoint.h"
#include <rosplan_knowledge_msgs/GetInstanceService.h>
#include <rosplan_knowledge_msgs/GetAttributeService.h>
#include <rosplan_knowledge_msgs/KnowledgeUpdateService.h>
#include <rosplan_knowledge_msgs/KnowledgeItem.h>
#include <rosplan_knowledge_msgs/CreatePRM.h>
#include <rosplan_knowledge_msgs/Filter.h>

#include "mongodb_store/message_store.h"

#ifndef KCL_simple_demo
#define KCL_simple_demo

/**
 * This file defines the SimpleDemoExecutor class.
 * This class uses ROSPlan and some SQUIRREL components to execute a simple demo.
 * The user points to two locations. The robot:
 * moves to the first;
 * explores;
 * if there are toys, moves them to the other location.
 */
namespace KCL_rosplan {

	class SimpleDemoExecutor
	{

	private:
		
		// planner control.
		ros::ServiceClient run_planner_client;

		// Scene database
		mongodb_store::MessageStoreProxy message_store;

		// Knowledge base
		ros::Publisher filter_publisher;
		ros::ServiceClient knowledge_update_client;
		ros::ServiceClient get_attribute_client;
		ros::ServiceClient get_instance_client;

		// ROSPlan roadmap
		ros::ServiceClient add_waypoint_client;
		ros::ServiceClient roadmap_service;

		// action topics
		ros::Publisher action_feedback_pub;
		ros::Publisher head_tilt_pub;
		ros::Publisher head_nod_pub;
		ros::Subscriber pointing_pose_sub;

		// points
		geometry_msgs::PointStamped received_point_;
		bool has_received_point_;

		// head tilt
		float head_down_angle;
		float head_up_angle;
		int count;

		geometry_msgs::PoseStamped goal_pose;

	public:

		/* constructor */
		SimpleDemoExecutor(ros::NodeHandle &nh);
		void receivePointLocation(const geometry_msgs::PointStamped::ConstPtr& ptr);
		void makePoint(std::string& wpID);
		void runDemo();
	};
}
#endif
