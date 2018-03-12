#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <boost/foreach.hpp>
#include "mongodb_store/message_store.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_srvs/Empty.h"
#include "diagnostic_msgs/KeyValue.h"
#include <actionlib/client/simple_action_client.h>

#include "rosplan_dispatch_msgs/PlanAction.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GenerateProblemService.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_dispatch_msgs/PlanningService.h"

#include "rosplan_planning_system/PlanningEnvironment.h"
#include "rosplan_planning_system/PDDLProblemGenerator.h"

#include "squirrel_waypoint_msgs/ExamineWaypoint.h"
#include "squirrel_object_perception_msgs/SceneObject.h"

#ifndef KCL_behaviour_and_emotion
#define KCL_behaviour_and_emotion

/**
 * This file defines the BehaviourAndEmotion class.
 * BehaviourAndEmotion creates domain and problem files to regulate 
 * emotions with children and create behaviour plans with high-level 
 * strategies in mind like reciprocity.
 */
namespace KCL_rosplan {

	class ViewConeGenerator;
	
	class BehaviourAndEmotion
	{

	private:
		ros::NodeHandle* node_handle;
		mongodb_store::MessageStoreProxy message_store;
		ros::Publisher action_feedback_pub;
		
		/* PDDL problem generation */
		

		/* knowledge service clients */
		ros::ServiceClient update_knowledge_client;
		ros::ServiceClient get_instance_client;
		ros::ServiceClient get_attribute_client;
		ros::ServiceClient query_knowledge_client;
		
		// waypoint request services
		ros::ServiceClient classify_object_waypoint_client;
		
		// server that generates the PDDL domain and problem files.
		ros::ServiceServer pddl_generation_service;
		
		// Cache the last message sent.
		std::vector<rosplan_dispatch_msgs::ActionDispatch> last_received_msg;
		
		// View point generator.
		ViewConeGenerator* view_cone_generator;
		
		bool initial_problem_generated;
		
		// Determine whether this is a simulation or not.
		bool simulated;
		
		// Expect s to be "(f,f,f)"
		geometry_msgs::Pose transformToPose(const std::string& s);
		
		// Publisher to the visualiser.
		ros::Publisher vis_pub;
		
		void sendMarker(const geometry_msgs::Pose& pose, const std::string& name, float size);

		void tokenise(const std::string& s, std::vector<std::string>& tokens);
		
	public:

		/**
		 * Constructor.
		 * @param nh The node handle.
		 * @param vis_pub The publisher where RVIZ data should be sent.
		 */
		BehaviourAndEmotion(ros::NodeHandle &nh, const std::string& vis_pub_topic);
		
		/**
		 * Setup the scenario by reading from a config file and outputting the data to RVIZ.
		 * @param config_file The location of the configuration file to read.
		 */
		void setupSimulation(const std::string& config_file);

		/* callback function from the ROSPlan planning system to generate the PDDL problem file (and domain in our case) */
		bool generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		bool generateContingentProblem(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		bool generateRegularProblem(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res);
		
		/* Fetch functions. */
		inline mongodb_store::MessageStoreProxy& getMessageStore() { return message_store; }
	};
}
#endif
