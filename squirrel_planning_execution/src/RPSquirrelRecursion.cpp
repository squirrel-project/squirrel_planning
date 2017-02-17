#include <std_msgs/Int8.h>

#include <ros/ros.h>
#include <map>
#include <algorithm>
#include <string>
#include <sstream>

#include "squirrel_planning_execution/RPSquirrelRecursion.h"
#include "squirrel_planning_execution/ContingentStrategicClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTacticalClassifyPDDLGenerator.h"
#include "squirrel_planning_execution/ContingentTidyPDDLGenerator.h"
#include "squirrel_planning_execution/ViewConeGenerator.h"
#include "squirrel_planning_execution/ClassicalTidyPDDLGenerator.h"
#include "pddl_actions/ShedKnowledgePDDLAction.h"
#include "pddl_actions/FinaliseClassificationPDDLAction.h"
#include "pddl_actions/PlannerInstance.h"
#include "pddl_actions/ExamineAreaPDDLAction.h"
#include "pddl_actions/ExploreAreaPDDLAction.h"
#include "pddl_actions/ObserveClassifiableOnAttemptPDDLAction.h"
#include "pddl_actions/TidyAreaPDDLAction.h"

#include "squirrel_object_perception_msgs/BCylinder.h"
#include "squirrel_object_perception_msgs/SceneObject.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


/* The implementation of RPSquirrelRecursion.h */
namespace KCL_rosplan {

	/*-------------*/
	/* constructor */
	/*-------------*/

	RPSquirrelRecursion::RPSquirrelRecursion(ros::NodeHandle &nh)
		: node_handle(&nh), message_store(nh), initial_problem_generated(false), simulated(true)
	{
		// knowledge interface
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>("/kcl_rosplan/update_knowledge_base");
		query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>("/kcl_rosplan/query_knowledge_base");
		
		// create the action feedback publisher
		//action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>("/kcl_rosplan/action_feedback", 10, true);
		
		get_instance_client = nh.serviceClient<rosplan_knowledge_msgs::GetInstanceService>("/kcl_rosplan/get_current_instances");
		get_attribute_client = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>("/kcl_rosplan/get_current_knowledge");
		
		//std::string classifyTopic("/squirrel_perception_examine_waypoint");
		//nh.param("squirrel_perception_classify_waypoint_service_topic", classifyTopic, classifyTopic);
		//classify_object_waypoint_client = nh.serviceClient<squirrel_waypoint_msgs::ExamineWaypoint>(classifyTopic);
		
		//pddl_generation_service = nh.advertiseService("/kcl_rosplan/generate_planning_problem", &KCL_rosplan::RPSquirrelRecursion::generatePDDLProblemFile, this);

		//nh.getParam("/squirrel_planning_execution/simulated", simulated);
		
		setupSimulation();
	}
	
	void RPSquirrelRecursion::setupSimulation()
	{
		// We will make some fictional objects and associated waypoints.
		rosplan_knowledge_msgs::KnowledgeUpdateService knowledge_update_service;
		knowledge_update_service.request.update_type = rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE;
		
		// Create some types for the toys.
		std::vector<std::string> toy_types;
		toy_types.push_back("dinosaur");
		toy_types.push_back("car");
		toy_types.push_back("unknown");
		
		for (std::vector<std::string>::const_iterator ci = toy_types.begin(); ci != toy_types.end(); ++ci)
		{
			const std::string& type_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "type";
			knowledge_item.instance_name = type_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the type %s to the knowledge base.", type_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", type_predicate.c_str());
		}
		
		std::vector<std::string> boxes;
		boxes.push_back("box1");
		boxes.push_back("box2");
		
		for (std::vector<std::string>::const_iterator ci = boxes.begin(); ci != boxes.end(); ++ci)
		{
			const std::string& box_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			
			knowledge_item.instance_type = "box";
			knowledge_item.instance_name = box_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the box %s to the knowledge base.", box_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", box_predicate.c_str());
			
			// Add waypoints for these boxes.
			knowledge_item.instance_type = "waypoint";
			std::stringstream ss;
			ss << box_predicate << "_location";
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
			
			// Link the boxes to these waypoints.
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
			knowledge_item.attribute_name = "box_at";
			knowledge_item.is_negative = false;
			
			diagnostic_msgs::KeyValue kv;
			kv.key = "b";
			kv.value = box_predicate;
			knowledge_item.values.push_back(kv);
			
			kv.key = "wp";
			kv.value = ss.str();
			knowledge_item.values.push_back(kv);
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (box_at %s %s) to the knowledge base.", box_predicate.c_str(), ss.str().c_str());
			knowledge_item.values.clear();
			
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			ss.str(std::string());
			ss << "near_" << box_predicate;
			knowledge_item.instance_name = ss.str();
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", ss.str().c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", ss.str().c_str());
		}
		
		std::vector<std::string> waypoints;
		waypoints.push_back("kenny_waypoint");
		waypoints.push_back("pickup_waypoint");
		waypoints.push_back("child_waypoint");
		
		for (std::vector<std::string>::const_iterator ci = waypoints.begin(); ci != waypoints.end(); ++ci)
		{
			const std::string& waypoint_predicate = *ci;
			rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
			knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::INSTANCE;
			knowledge_item.instance_type = "waypoint";
			knowledge_item.instance_name = waypoint_predicate;
			
			knowledge_update_service.request.knowledge = knowledge_item;
			if (!update_knowledge_client.call(knowledge_update_service)) {
				ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the waypoint %s to the knowledge base.", waypoint_predicate.c_str());
				exit(-1);
			}
			ROS_INFO("KCL: (RPSquirrelRecursion) Added %s to the knowledge base.", waypoint_predicate.c_str());
		}
		
		// Set kenny at it's starting waypoint.
		rosplan_knowledge_msgs::KnowledgeItem knowledge_item;
		knowledge_item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
		knowledge_item.attribute_name = "robot_at";
		knowledge_item.is_negative = false;
		
		diagnostic_msgs::KeyValue kv;
		kv.key = "v";
		kv.value = "robot";
		knowledge_item.values.push_back(kv);
		
		kv.key = "wp";
		kv.value = "kenny_waypoint";
		knowledge_item.values.push_back(kv);
		
		knowledge_update_service.request.knowledge = knowledge_item;
		if (!update_knowledge_client.call(knowledge_update_service)) {
			ROS_ERROR("KCL: (RPSquirrelRecursion) Could not add the fact (robot_at robot kenny_waypoint) to the knowledge base.");
			exit(-1);
		}
		ROS_INFO("KCL: (RPSquirrelRecursion) Added the fact (robot_at robot kenny_waypoint) to the knowledge base.");
		knowledge_item.values.clear();
	}
	
	/*--------------------*/
	/* problem generation */
	/*--------------------*/

	/**
	 * Generate a contingent problem.
	 */
	//bool RPSquirrelRecursion::generatePDDLProblemFile(rosplan_knowledge_msgs::GenerateProblemService::Request &req, rosplan_knowledge_msgs::GenerateProblemService::Response &res) {
	//	return true;
	//}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/
	
	void initMongoDBData(mongodb_store::MessageStoreProxy& message_store)
	{
		// Create a Scene Object for the toy.
		squirrel_object_perception_msgs::SceneObject scene_object;
		scene_object.id = "toy1";
		scene_object.category = "unknown";
		
		geometry_msgs::PoseStamped object_pose;
		object_pose.header.seq = 0;
		object_pose.header.stamp = ros::Time::now();
		object_pose.header.frame_id = "/map";
		object_pose.pose.position.x = -0.08f;
		object_pose.pose.position.y = -3.88f;
		object_pose.pose.position.z = 0.0f;
		
		object_pose.pose.orientation.x = 0.0f;
		object_pose.pose.orientation.y = 0.0f;
		object_pose.pose.orientation.z = 0.0f;
		object_pose.pose.orientation.w = 1.0f;
		scene_object.pose = object_pose.pose;
		
		squirrel_object_perception_msgs::BCylinder c;
		c.diameter = 0.3f;
		c.height = 0.2f;
		scene_object.bounding_cylinder = c;
		std::string near_waypoint_mongodb_id(message_store.insertNamed("toy1", scene_object));
		
		// Create the location of both boxes.
		geometry_msgs::PoseStamped box1_pose;
		box1_pose.header.seq = 0;
		box1_pose.header.stamp = ros::Time::now();
		box1_pose.header.frame_id = "/map";
		box1_pose.pose.position.x = 4.09f;
		box1_pose.pose.position.y = -3.88f;
		box1_pose.pose.position.z = 0.0f;
		
		box1_pose.pose.orientation.x = 0.0f;
		box1_pose.pose.orientation.y = 0.0f;
		box1_pose.pose.orientation.z = 0.0f;
		box1_pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id2(message_store.insertNamed("box1", box1_pose));
		
		geometry_msgs::PoseStamped box2_pose;
		box2_pose.header.seq = 0;
		box2_pose.header.stamp = ros::Time::now();
		box2_pose.header.frame_id = "/map";
		box2_pose.pose.position.x = 3.71f;
		box2_pose.pose.position.y = -7.55f;
		box2_pose.pose.position.z = 0.0f;
		
		box2_pose.pose.orientation.x = 0.0f;
		box2_pose.pose.orientation.y = 0.0f;
		box2_pose.pose.orientation.z = 0.0f;
		box2_pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2", box2_pose));
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -0.243f;
		pose.pose.position.y = 4.43f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box1_location", box1_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -0.243f;
		pose.pose.position.y = 4.03f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_box1", box1_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.5f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2_location", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.3f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_box2", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = -1.71f;
		pose.pose.position.y = 1.5f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("box2_location", box2_pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = 2.41f;
		pose.pose.position.y = 0.292f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("near_child1", pose));
		}
		
		{
		geometry_msgs::PoseStamped pose;
		pose.header.seq = 0;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "/map";
		pose.pose.position.x = 2.41f;
		pose.pose.position.y = 0.292f;
		pose.pose.position.z = 0.0f;
		
		pose.pose.orientation.x = 0.0f;
		pose.pose.orientation.y = 0.0f;
		pose.pose.orientation.z = 0.0f;
		pose.pose.orientation.w = 1.0f;
		std::string near_waypoint_mongodb_id3(message_store.insertNamed("child1_location", pose));
		}
	}

	int main(int argc, char **argv) {

		ros::init(argc, argv, "rosplan_interface_RPSquirrelRecursion");
		ros::NodeHandle nh;

		// create PDDL action subscriber
		KCL_rosplan::RPSquirrelRecursion rpsr(nh);
		
		// Setup the environment.
		initMongoDBData(rpsr.getMessageStore());
		
		// Setup all the simulated actions.
		KCL_rosplan::ShedKnowledgePDDLAction shed_knowledge_action(nh);
		KCL_rosplan::FinaliseClassificationPDDLAction finalise_classify_action(nh);
		
		// Setup the recursive actions.
		KCL_rosplan::ExamineAreaPDDLAction examine_area_action(nh);
		KCL_rosplan::ExploreAreaPDDLAction explore_area_action(nh);
		KCL_rosplan::ObserveClassifiableOnAttemptPDDLAction observe_classifiable_on_attempt_action(nh);
		KCL_rosplan::TidyAreaPDDLAction tidy_are_action(nh);
		
		// Lets start the planning process.
		std::string data_path;
		nh.getParam("/data_path", data_path);
		
		std::string planner_path;
		nh.getParam("/planner_path", planner_path);
		
		std::stringstream ss;
		//ss << data_path << "tidy_room_domain-nt.pddl";
		ss << data_path << "template_robot_knows_domain.pddl";
		std::string domain_path = ss.str();
		
		ss.str(std::string());
		ss << data_path << "tidy_room_problem.pddl";
		std::string problem_path = ss.str();
		
		std::string planner_command;
		nh.getParam("/squirrel_planning_execution/planner_command", planner_command);
		
		rosplan_dispatch_msgs::PlanGoal psrv;
		psrv.domain_path = domain_path;
		psrv.problem_path = problem_path;
		psrv.data_path = data_path;
		psrv.planner_command = planner_command;
		psrv.start_action_id = 0;

		ROS_INFO("KCL: (RPSquirrelRecursion) Start plan action");
		actionlib::SimpleActionClient<rosplan_dispatch_msgs::PlanAction> plan_action_client("/kcl_rosplan/start_planning", true);

		plan_action_client.waitForServer();
		ROS_INFO("KCL: (RPSquirrelRecursion) Start planning server found");
		
		// send goal
		plan_action_client.sendGoal(psrv);
		ROS_INFO("KCL: (RPSquirrelRecursion) Goal sent");
		
		ros::spin();
		return 0;
	}
